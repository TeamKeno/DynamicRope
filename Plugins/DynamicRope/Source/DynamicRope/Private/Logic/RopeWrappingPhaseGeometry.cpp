// Copyright Epic Games, Inc. All Rights Reserved.

#include "Logic/RopeWrappingPhase.h"
#include "Components/SceneComponent.h"
// ResolveBindingWorld — 랩 바인딩(본/소켓/컴포넌트) 트랜스폼 해석의 단일 지점(seam A).
#include "Core/RopeWrapTarget.h"
#include "DynamicRopeLog.h"
#include "Collision/RopeCollider.h"
// TRACE_CPUPROFILER_EVENT_SCOPE (Unreal Insights)
#include "ProfilingDebugging/CpuProfilerTrace.h"
// RopeMath::AnyTangentFromNormal (unity 빌드 중복 정의 방지)
#include "RopeMathHelpers.h"

#pragma region Wrapping Geometry and Axis Resolution

bool FRopeWrappingPhase::ComputeBuiltPathWrapAngle(const FRopeSimState& Sim, const FContext& Ctx, float& OutAngleDeg) const
{
	OutAngleDeg = 0.0f;
	if (State.Anchors.Num() == 0 && State.Path.Num() == 0)
	{
		return false;
	}

	// Sequential SurfaceVectorField와 Composite AnalyticHelix 모두 경로 빌드 중 누적한 실제 위상을
	// 우선 사용한다. 아직 한 스텝도 진행하지 못한 경로만 아래의 단일 축 근사로 폴백한다.
	if (State.PathAccumulatedAngleRad > KINDA_SMALL_NUMBER)
	{
		OutAngleDeg = FMath::RadiansToDegrees(State.PathAccumulatedAngleRad);
		return true;
	}

	const FRopeSurfaceAnchor& LatchAnchor = State.LatchAnchor;
	const USceneComponent* Mesh = ResolveWrappingMesh(State, LatchAnchor);
	if (!Mesh || LatchAnchor.Bone.IsNone())
	{
		return false;
	}

	FVector AxisOrigin = FVector::ZeroVector;
	FVector AxisDirection = FVector::ForwardVector;
	if (!ResolveWrappingAxis(LatchAnchor, Ctx, AxisOrigin, AxisDirection))
	{
		return false;
	}
	OrientWrappingAxisByTail(LatchAnchor, Sim, Mesh, AxisDirection);

	const FTransform BoneXform = ResolveBindingWorld(Mesh, LatchAnchor.Bone);
	const FVector LatchSurfaceWorld = BoneXform.TransformPosition(LatchAnchor.LocalSurfacePosition);
	const float LatchAxisDistance = FVector::DotProduct(LatchSurfaceWorld - AxisOrigin, AxisDirection);
	const FVector LatchAxisPoint = AxisOrigin + AxisDirection * LatchAxisDistance;
	const float HelixRadius = (LatchSurfaceWorld - LatchAxisPoint).Size();
	if (HelixRadius <= KINDA_SMALL_NUMBER)
	{
		return false;
	}

	float LastBuiltDistance = 0.0f;
	for (const FRopeSurfaceAnchor& Anchor : State.Anchors)
	{
		LastBuiltDistance = FMath::Max(LastBuiltDistance, Anchor.RopeDistance);
	}
	if (State.Path.Num() > 0)
	{
		LastBuiltDistance = FMath::Max(LastBuiltDistance, State.Path.Last().DistanceFromLatch);
	}

	// 실제 SurfaceVectorField 경로가 얼마나 울퉁불퉁했는지와 별개로, 실패 판정은 helix 기준 누적
	// 감싼 각도만 본다. 각도(도) 반환 — 회전 수(=각도/360)는 2πr 로프를 요구해 대상 크기에 비례하는
	// 기준이 되므로 쓰지 않는다(FRopeWrapConfig::FailedWrapMinAngleDeg 주석 참고).
	if (State.bPathUsesPoseSpaceIsland && State.Path.Num() > 1)
	{
		// 자동 pitch와 진입 반지름을 사용한 독립 helix는 이미 실제 누적 위상을 저장한다.
		// 고정 config pitch로 다시 역산하면 커밋 로그/품질 관문의 각도가 경로와 달라진다.
		OutAngleDeg = FMath::RadiansToDegrees(
			FMath::Abs(State.PathCompositeSweepAngleRad));
		return true;
	}

	const float PitchScale = Ctx.Config.WrappingHelixPitchScale;
	const float LengthScale = FMath::Sqrt(1.0f + PitchScale * PitchScale);
	const float CircumferenceDistance = LastBuiltDistance / FMath::Max(LengthScale, KINDA_SMALL_NUMBER);
	const float AngleRadians = CircumferenceDistance / FMath::Max(HelixRadius, KINDA_SMALL_NUMBER);
	OutAngleDeg = FMath::RadiansToDegrees(FMath::Abs(AngleRadians));
	return true;
}

bool FRopeWrappingPhase::ComputeWrapEnclosureCoverage(float& OutCoverageDeg) const
{
	OutCoverageDeg = 0.0f;
	if (State.Path.Num() < 2)
	{
		return false;
	}

	const FVector Axis = State.PathAxisDirection.GetSafeNormal();
	if (Axis.IsNearlyZero())
	{
		return false;
	}

	const auto ComputeRadial = [this, &Axis](const FVector& Point, FVector& OutRadial) -> bool
	{
		const FVector Offset = Point - State.PathAxisOrigin;
		OutRadial = Offset - Axis * FVector::DotProduct(Offset, Axis);
		return OutRadial.Normalize(KINDA_SMALL_NUMBER);
	};

	// 각 경로점의 축 둘레 각도(첫 비축퇴 점 기준, (-180,180]). 브리지 점도 포함한다 —
	// chord가 가로지른 방향도 로프가 막고 있는 방향이다.
	FVector RefRadial = FVector::ZeroVector;
	TArray<float, TInlineAllocator<128>> AngleDegrees;
	for (const FRopeWrapPathPoint& Point : State.Path)
	{
		FVector Radial = FVector::ZeroVector;
		if (!ComputeRadial(Point.SurfaceWorld, Radial))
		{
			continue;
		}

		if (RefRadial.IsNearlyZero())
		{
			RefRadial = Radial;
		}

		const float AngleRad = FMath::Atan2(
			static_cast<float>(FVector::DotProduct(Axis, FVector::CrossProduct(RefRadial, Radial))),
			static_cast<float>(FVector::DotProduct(RefRadial, Radial)));
		AngleDegrees.Add(FMath::RadiansToDegrees(AngleRad));
	}

	if (AngleDegrees.Num() < 2)
	{
		return false;
	}

	// 정렬 후 최대 각도 공백(이웃 간 + 양끝 wrap-around)을 찾는다. 커버리지 = 360 − 최대 공백:
	// 점들이 축 둘레를 빈틈없이 두르면 공백이 스텝 각 수준으로 작아 360에 수렴하고,
	// 반쪽 훅이면 반대편이 통째로 비어 커버리지가 그만큼 낮다.
	AngleDegrees.Sort();
	float MaxGapDeg = 360.0f - (AngleDegrees.Last() - AngleDegrees[0]);
	for (int32 Index = 1; Index < AngleDegrees.Num(); ++Index)
	{
		MaxGapDeg = FMath::Max(MaxGapDeg, AngleDegrees[Index] - AngleDegrees[Index - 1]);
	}

	OutCoverageDeg = FMath::Clamp(360.0f - MaxGapDeg, 0.0f, 360.0f);
	return true;
}

bool FRopeWrappingPhase::FindGuidePlaneAxis(const FRopeSurfaceAnchor& LatchAnchor, const FContext& Ctx,
	const USceneComponent* Mesh, FVector& OutAxisOrigin, FVector& OutAxisDirection)
{
	if (!Mesh || LatchAnchor.Bone.IsNone() || !Ctx.bHasGuidePlaneNormal)
	{
		return false;
	}

	const FVector GuidePlaneNormal = Ctx.GuidePlaneNormal.GetSafeNormal();
	if (GuidePlaneNormal.IsNearlyZero())
	{
		return false;
	}

	// CaptureTravelPlane: 축 origin을 latch 본 위치가 아니라 캡처 순간의 접촉 영역 중심에 둔다 —
	// 여러 본/대상에 걸친 접촉(양다리)에서 감김 반경이 한쪽 대상이 아닌 쌍의 중심을 기준으로 잡힌다.
	// origin은 캡처 시점 고정값이라 본 전환 재시드(rolling axis)에서도 움직이지 않는다.
	// BoneCenteredGuidePlane은 종전처럼 본 위치를 쓴다 — Assisted 단일 본 동작 불변.
	if (Ctx.Config.WrappingAxisSource == ERopeWrappingAxisSource::CaptureTravelPlane &&
		Ctx.TravelFrame && Ctx.TravelFrame->bValid)
	{
		FVector Origin = Ctx.TravelFrame->RegionCenter;

		// 접촉 군집 보정(브리징 = 분리 대상 랩 모드에서만): RegionCenter는 "첫 접촉" 순간의 접촉점
		// 평균이라, 캡처가 첫 다리에 닿는 즉시 일어나면(MinLatchNodes 기본 1) 한쪽 다리 위에 있다 —
		// 축이 대상 안을 지나면 그 대상만 도는 궤도가 winding 순방향이 되어 이탈 관문이 침묵하고,
		// 반대쪽 다리로 못 건너간다(PIE 실측 2026-07-13: 한 다리 1725° 나선, coverage는 축이 대상
		// 안이라 무의미하게 높음). 접촉 못 한 이웃 대상도 collider 스냅샷에는 있으므로, 같은 mesh의
		// 근방(브리지 거리) collider 중심들을 평균해 축이 군집(양다리 쌍)의 중심을 지나게 한다.
		// 축 방향 성분은 버린다 — 축은 선이라 수직 성분만 의미가 있다.
		const float ClusterRadius = Ctx.Config.WrappingMaxGapBridgeDistance;
		if (ClusterRadius > 0.0f)
		{
			FVector CenterSum = FVector::ZeroVector;
			int32 CenterCount = 0;
			for (const IRopeCollider* Collider : Ctx.Colliders)
			{
				if (!Collider)
				{
					continue;
				}

				FName ColliderBone = NAME_None;
				const USceneComponent* ColliderMesh = nullptr;
				Collider->GetGPUAttribution(ColliderBone, ColliderMesh);
				if (Mesh != nullptr && ColliderMesh != nullptr && ColliderMesh != Mesh)
				{
					continue;
				}

				FVector Center = FVector::ZeroVector;
				if (!GetColliderCenter(*Collider, Center))
				{
					continue;
				}

				const FVector Delta = Center - Origin;
				const float AlongAxis = static_cast<float>(FVector::DotProduct(Delta, GuidePlaneNormal));
				if (FMath::Abs(AlongAxis) > ClusterRadius ||
					(Delta - GuidePlaneNormal * AlongAxis).Size() > ClusterRadius)
				{
					continue;
				}

				CenterSum += Center;
				++CenterCount;
			}

			if (CenterCount > 0)
			{
				const FVector ClusterDelta = CenterSum / static_cast<float>(CenterCount) - Origin;
				Origin += ClusterDelta - GuidePlaneNormal *
					static_cast<float>(FVector::DotProduct(ClusterDelta, GuidePlaneNormal));
			}
		}

		OutAxisOrigin = Origin;
	}
	else
	{
		OutAxisOrigin = ResolveBindingWorld(Mesh, LatchAnchor.Bone).GetLocation();
	}
	OutAxisDirection = GuidePlaneNormal;
	return true;
}

bool FRopeWrappingPhase::GetColliderCenter(const IRopeCollider& Collider, FVector& OutCenter)
{
	FVector CapA = FVector::ZeroVector;
	FVector CapB = FVector::ZeroVector;
	float CapRadius = 0.0f;
	if (Collider.GetGPUCapsule(CapA, CapB, CapRadius))
	{
		OutCenter = (CapA + CapB) * 0.5f;
		return true;
	}

	FVector BoxCenter = FVector::ZeroVector;
	FVector BoxHalf = FVector::ZeroVector;
	FQuat BoxRot = FQuat::Identity;
	if (Collider.GetGPUBox(BoxCenter, BoxRot, BoxHalf))
	{
		OutCenter = BoxCenter;
		return true;
	}

	FRopeSDFColliderView SDFView;
	if (Collider.GetGPUSDF(SDFView))
	{
		OutCenter = SDFView.BoneToWorld.TransformPosition(SDFView.LocalMin + SDFView.LocalSize * 0.5);
		return true;
	}

	// 랩 가능 convex(WrapTarget 전체 세트): 로컬 bounds 중심을 강체로 월드 변환.
	TConstArrayView<FPlane> ConvexPlanes;
	FBox ConvexLocalBounds(ForceInit);
	FQuat ConvexRot = FQuat::Identity;
	FQuat ConvexPrevRot = FQuat::Identity;
	FVector ConvexTrans = FVector::ZeroVector;
	FVector ConvexPrevTrans = FVector::ZeroVector;
	float ConvexInvDt = 0.0f;
	if (Collider.GetGPUConvex(ConvexPlanes, ConvexLocalBounds, ConvexRot, ConvexTrans,
			ConvexPrevRot, ConvexPrevTrans, ConvexInvDt)
		&& ConvexLocalBounds.IsValid)
	{
		OutCenter = ConvexRot.RotateVector(ConvexLocalBounds.GetCenter()) + ConvexTrans;
		return true;
	}

	return false;
}

bool FRopeWrappingPhase::ResolveWrappingAxis(const FRopeSurfaceAnchor& LatchAnchor, const FContext& Ctx,
	FVector& OutAxisOrigin, FVector& OutAxisDirection) const
{
	const auto LogAxisSource = [&](const TCHAR* Source, const USceneComponent* MeshForLog)
	{
		UE_LOG(LogRopeWrap, VeryVerbose,
			TEXT("[%s] Wrapping axis: source=%s, bone=%s, mesh=%s, origin=%s, dir=%s"),
			*Ctx.OwnerName,
			Source,
			*LatchAnchor.Bone.ToString(),
			*GetNameSafe(MeshForLog),
			*OutAxisOrigin.ToString(),
			*OutAxisDirection.ToString());
	};

	// CaptureTravelPlane: 진행 평면 normal 축을 캡처 접촉 영역/collider 군집 중심에 고정한다.
	// 여러 본에 걸친 Composite wrapping이 특정 latch 본의 위치에 끌려가지 않도록 하는 모드다.
	bool bTriedCaptureTravelPlane = false;
	if (Ctx.Config.WrappingAxisSource == ERopeWrappingAxisSource::CaptureTravelPlane)
	{
		bTriedCaptureTravelPlane = true;
		const USceneComponent* CaptureMesh = ResolveWrappingMesh(State, LatchAnchor);
		if (CaptureMesh && FindGuidePlaneAxis(LatchAnchor, Ctx, CaptureMesh, OutAxisOrigin, OutAxisDirection))
		{
			LogAxisSource(TEXT("CaptureTravelPlane"), CaptureMesh);
			return true;
		}
	}

	const USceneComponent* Mesh = ResolveWrappingMesh(State, LatchAnchor);
	if (!Mesh || LatchAnchor.Bone.IsNone())
	{
		return false;
	}

	const FName ParentBone = RopeWrapTargets::GetParentTargetKey(Mesh, LatchAnchor.Bone);
	const FVector BoneLocation = ResolveBindingWorld(Mesh, LatchAnchor.Bone).GetLocation();

	// BoneCenteredGuidePlane: 같은 진행 평면 normal을 latch 본 위치에 세운다.
	// CaptureTravelPlane이었다면 이미 위에서 시도·실패한 것이므로 같은 입력을 재시도하지 않는다.
	if (!bTriedCaptureTravelPlane && FindGuidePlaneAxis(LatchAnchor, Ctx, Mesh, OutAxisOrigin, OutAxisDirection))
	{
		LogAxisSource(TEXT("BoneCenteredGuidePlane"), Mesh);
		return true;
	}

	// rope 평면 축을 만들 수 없으면 스켈레탈 대상은 bone-parent 축으로 폴백한다.
	if (!ParentBone.IsNone())
	{
		const FVector ParentLocation = ResolveBindingWorld(Mesh, ParentBone).GetLocation();
		const FVector Axis = BoneLocation - ParentLocation;
		if (!Axis.IsNearlyZero())
		{
			OutAxisOrigin = ParentLocation;
			OutAxisDirection = Axis.GetSafeNormal();
			LogAxisSource(TEXT("BoneParentAxis"), Mesh);
			return true;
		}
	}

	const FTransform BoneXform = ResolveBindingWorld(Mesh, LatchAnchor.Bone);

	// 정적/비-스켈레탈 대상(피드백 5): 본 그래프가 없어 축을 컴포넌트 기저에서 유도한다. 컴포넌트
	// 기저축(X/Y/Z) 중 latch 표면 normal에 가장 수직인 축을 감김 축으로 고른다 — 원기둥/캡슐의 장축은
	// 반경 방향(표면 normal)에 수직이므로, 축정렬 랩 캡슐(기둥=Z, 가로보=X/Y)에서 올바른 감김 축이
	// 자동 선택된다(부모가 있는 스켈레탈은 위에서 이미 반환됨 — 스켈레탈 루트 본은 아래 로컬 X 폴백).
	if (!RopeWrapTargets::IsSkeletalTarget(Mesh))
	{
		const FVector NormalWorld = BoneXform.TransformVectorNoScale(LatchAnchor.LocalNormal)
			.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
		FVector BestAxis = BoneXform.GetUnitAxis(EAxis::Z);
		float BestParallel = TNumericLimits<float>::Max();
		for (const EAxis::Type CandidateAxis : { EAxis::X, EAxis::Y, EAxis::Z })
		{
			const FVector AxisWorld = BoneXform.GetUnitAxis(CandidateAxis);
			const float ParallelToNormal = FMath::Abs(FVector::DotProduct(AxisWorld, NormalWorld));
			if (ParallelToNormal < BestParallel)
			{
				BestParallel = ParallelToNormal;
				BestAxis = AxisWorld;
			}
		}
		OutAxisOrigin = BoneLocation;
		OutAxisDirection = BestAxis.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
		LogAxisSource(TEXT("StaticBasisFallback"), Mesh);
		return true;
	}

	OutAxisOrigin = BoneLocation;
	OutAxisDirection = BoneXform.GetUnitAxis(EAxis::X).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
	LogAxisSource(TEXT("BoneLocalXFallback"), Mesh);
	return true;
}

void FRopeWrappingPhase::OrientWrappingAxisByTail(const FRopeSurfaceAnchor& LatchAnchor, const FRopeSimState& Sim,
	const USceneComponent* Mesh, FVector& InOutAxisDirection) const
{
	if (!Mesh || LatchAnchor.Bone.IsNone())
	{
		return;
	}

	const FName ParentBone = RopeWrapTargets::GetParentTargetKey(Mesh, LatchAnchor.Bone);
	if (ParentBone.IsNone())
	{
		return;
	}

	float ParentScore = 0.0f;
	float BoneScore = 0.0f;
	float TotalWeight = 0.0f;
	const FVector ParentWorld = ResolveBindingWorld(Mesh, ParentBone).GetLocation();
	const FVector BoneWorld = ResolveBindingWorld(Mesh, LatchAnchor.Bone).GetLocation();

	const auto AddProbe = [&](int32 NodeIndex, float Weight)
	{
		if (!Sim.Positions.IsValidIndex(NodeIndex) || Weight <= 0.0f)
		{
			return;
		}

		const FVector ProbeWorld = Sim.Positions[NodeIndex];
		ParentScore += FVector::DistSquared(ProbeWorld, ParentWorld) * Weight;
		BoneScore += FVector::DistSquared(ProbeWorld, BoneWorld) * Weight;
		TotalWeight += Weight;
	};

	AddProbe(LatchAnchor.NodeIndex + 1, 2.0f);
	AddProbe(Sim.Num() - 1, 1.0f);

	if (TotalWeight <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	ParentScore /= TotalWeight;
	BoneScore /= TotalWeight;

	if (ParentScore < BoneScore)
	{
		InOutAxisDirection *= -1.0f;
	}
}

void FRopeWrappingPhase::ReseedWrappingAxisOnBoneTransition(FName Bone, const USceneComponent* Mesh, const FContext& Ctx)
{
	if (Bone.IsNone())
	{
		return;
	}

	// ResolveWrappingAxis의 입력 계약(본/메시 + 본 로컬 표면 프레임)만 채운 합성 anchor. 현재 경로
	// 지점의 표면 프레임을 새 본 로컬로 옮겨 담아, collider 형상 축 실패 시의 폴백(guide 평면/기저축)도
	// 새 본 위치·현재 normal 기준으로 동작하게 한다.
	FRopeSurfaceAnchor AxisAnchor;
	AxisAnchor.Bone = Bone;
	AxisAnchor.Mesh = Mesh;
	const FTransform BoneXform = ResolveBindingWorld(Mesh, Bone);
	AxisAnchor.LocalSurfacePosition = BoneXform.InverseTransformPosition(State.PathSurfaceWorld);
	AxisAnchor.LocalNormal = BoneXform.InverseTransformVectorNoScale(State.PathNormalWorld)
		.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
	AxisAnchor.LocalTangent = BoneXform.InverseTransformVectorNoScale(State.PathTangentWorld)
		.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);

	FVector NewAxisOrigin = State.PathAxisOrigin;
	FVector NewAxisDirection = State.PathAxisDirection;
	if (!ResolveWrappingAxis(AxisAnchor, Ctx, NewAxisOrigin, NewAxisDirection))
	{
		// 새 본 축 유도 실패 — 직전 본 축으로 계속 진행한다(종전 단일 축 동작과 동일한 폴백).
		return;
	}

	// 부호 정렬: 체인 본의 이웃 축은 대체로 이어지므로 이전 축과 반대면 뒤집는다 — 피치 드리프트
	// 방향(감기며 축을 따라 미끄러지는 쪽)이 전환점에서 반전되지 않게 한다. OrientWrappingAxisByTail은
	// latch 시점 tail 위치 휴리스틱이라 경로 중간 재해석에는 부적합하다.
	if (FVector::DotProduct(NewAxisDirection, State.PathAxisDirection) < 0.0f)
	{
		NewAxisDirection *= -1.0f;
	}

	// 현재 표면점 기준 radial/원주 재계산 + winding 재선출: 새 필드가 지금 진행 방향(tangent) 그대로
	// 새 축 주위를 돌게 한다. 여기서 winding을 다시 뽑지 않으면 전환 지점의 기하에 따라 감김 방향이
	// 뒤집힐 수 있다.
	const float AxisDistance = FVector::DotProduct(State.PathSurfaceWorld - NewAxisOrigin, NewAxisDirection);
	const FVector AxisPoint = NewAxisOrigin + NewAxisDirection * AxisDistance;
	const FVector Radial = (State.PathSurfaceWorld - AxisPoint)
		.GetSafeNormal(KINDA_SMALL_NUMBER, State.PathNormalWorld);
	FVector CircumferenceDir = FVector::CrossProduct(NewAxisDirection, Radial)
		.GetSafeNormal(KINDA_SMALL_NUMBER, State.PathCircumferenceDir);
	const float NewWindingSign =
		FVector::DotProduct(CircumferenceDir, State.PathTangentWorld) < 0.0f ? -1.0f : 1.0f;

	State.PathAxisOrigin = NewAxisOrigin;
	State.PathAxisDirection = NewAxisDirection;
	State.PathLatchRadial = Radial;
	State.PathWindingSign = NewWindingSign;
	State.PathCircumferenceDir = CircumferenceDir * NewWindingSign;

	UE_LOG(LogRopeWrap, VeryVerbose,
		TEXT("[%s] Wrapping axis re-seeded on bone transition: bone=%s, origin=%s, dir=%s, winding=%+.0f"),
		*Ctx.OwnerName, *Bone.ToString(),
		*State.PathAxisOrigin.ToString(), *State.PathAxisDirection.ToString(), NewWindingSign);
}

#pragma endregion
