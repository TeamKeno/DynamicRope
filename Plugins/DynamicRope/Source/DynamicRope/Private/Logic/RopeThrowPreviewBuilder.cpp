// Copyright Epic Games, Inc. All Rights Reserved.

#include "Logic/RopeThrowPreviewBuilder.h"

#include "Collision/RopeCollider.h"
// ResolveBindingWorld — 랩 바인딩(본/소켓/컴포넌트) 트랜스폼 해석의 단일 지점(seam A).
#include "Core/RopeWrapTarget.h"
#include "Logic/RopeFlightContactDetector.h"
#include "Logic/RopeWhipGuide.h"
#include "Logic/RopeWrappingPhase.h"
#include "RopeMathHelpers.h"
#include "Components/SkeletalMeshComponent.h"

namespace
{
	const TArray<IRopeCollider*>& GetColliders(const FRopeThrowPreviewBuilder::FInput& Input)
	{
		static const TArray<IRopeCollider*> EmptyColliders;
		return Input.Colliders ? *Input.Colliders : EmptyColliders;
	}

	bool BuildThrowArcPreview(const FRopeThrowPreviewBuilder::FInput& Input, FRopeArcPreviewData& OutPreview,
		FString* OutFailureReason)
	{
		OutPreview = FRopeArcPreviewData();

		const FRopeSimState* Sim = Input.Sim;
		const float ClampedReachScale = FMath::Max(Input.ReachScale, 0.0f);
		if (ClampedReachScale <= KINDA_SMALL_NUMBER)
		{
			RopeMath::SetPreviewFailureReason(OutFailureReason,
				FString::Printf(TEXT("throw arc preview build failed: reach scale too small (reachScale=%.2f)"),
					Input.ReachScale));
			return false;
		}

		const FRopeThrowContext& ThrowContext = Input.ThrowContext;
		const FRopeWhipGuide::FSwingBasis SwingBasis = FRopeWhipGuide::ResolveSwingBasis(
			ThrowContext, ThrowContext.SwingPlane, ThrowContext.CustomSwingPlaneNormal);
		const float SourceRopeLength = FMath::Max(Sim ? Sim->RopeLength : 0.0f, Input.RopeLength);

		OutPreview.Origin = ThrowContext.Origin;
		OutPreview.AimDir = SwingBasis.AimDir;
		OutPreview.GuideUp = SwingBasis.GuideUp;
		OutPreview.Radius = SourceRopeLength * ClampedReachScale;
		OutPreview.SweepAngleDegrees = FMath::Clamp(Input.SweepAngleDegrees, 1.0f, 180.0f);
		OutPreview.SegmentCount = FMath::Clamp(Input.SegmentCount, 1, 128);
		if (OutPreview.Radius <= KINDA_SMALL_NUMBER)
		{
			RopeMath::SetPreviewFailureReason(OutFailureReason,
				FString::Printf(TEXT("throw arc preview build failed (reachScale=%.2f, segmentCount=%d, ropeLength=%.2f)"),
					Input.ReachScale, Input.SegmentCount, SourceRopeLength));
			return false;
		}
		return true;
	}

	struct FThrowPreviewContactCandidate
	{
		FRopeContactCandidate Candidate;
		float AngleAlpha = 1.0f;
		float DistanceAlpha = 1.0f;
		float ArcStartAngleDegrees = 180.0f;
		FVector Direction = FVector::ForwardVector;
		// 준비된 preview에서만 origin->hit 직선을 사용한다. 실제 Flight 노드를 hit에 고정하는 옵션이 아니다.
		bool bForceDirectPathToContact = false;
	};

	bool IsBetterThrowPreviewContactCandidate(const FThrowPreviewContactCandidate& Candidate,
		const FThrowPreviewContactCandidate& Best)
	{
		if (!FMath::IsNearlyEqual(Candidate.ArcStartAngleDegrees, Best.ArcStartAngleDegrees, 0.1f))
		{
			return Candidate.ArcStartAngleDegrees < Best.ArcStartAngleDegrees;
		}
		if (!FMath::IsNearlyEqual(Candidate.DistanceAlpha, Best.DistanceAlpha, 0.01f))
		{
			return Candidate.DistanceAlpha < Best.DistanceAlpha;
		}
		return Candidate.AngleAlpha < Best.AngleAlpha;
	}

	bool BuildAimGuideHitCandidate(const FRopeArcPreviewData& Preview, const FRopeThrowContext& ThrowContext,
		const FRopeSimState& Sim, FThrowPreviewContactCandidate& OutCandidate)
	{
		OutCandidate = FThrowPreviewContactCandidate();
		const USceneComponent* GuideMesh = ThrowContext.AimGuideMesh.Get();
		if (!ThrowContext.bHasAimGuideHit || !GuideMesh || ThrowContext.AimGuideBone.IsNone() || Sim.Num() < 2)
		{
			return false;
		}

		const FVector ToHit = ThrowContext.AimGuideHitWorldPos - Preview.Origin;
		const float HitDistance = ToHit.Size();
		const FVector HitDir = ToHit.GetSafeNormal(KINDA_SMALL_NUMBER, Preview.AimDir);
		if (HitDistance <= KINDA_SMALL_NUMBER || HitDir.IsNearlyZero())
		{
			return false;
		}

		const float SegmentLength = FMath::Max(Sim.SegmentLength, 1.0f);
		const int32 DistanceNodeIndex = FMath::Clamp(FMath::RoundToInt(HitDistance / SegmentLength), 1, Sim.Num() - 1);
		// LockAlpha는 spline의 공간 보간 구간일 뿐 latch 위치가 아니다. 실제 hit 거리의 노드를 사용한다.
		const int32 AimGuideNodeIndex = DistanceNodeIndex;

		// aim ray 조준은 이미 SDF/collider swept query로 본을 고른 상태다.
		// 여기서 arc 전체를 다시 뒤지면 다른 본/다른 방향 후보가 선택될 수 있으므로 ray hit를 직접 prepared 후보로 쓴다.
		// SurfacePoint는 SDF 투영점이라 ray 위의 노란 hit와 다를 수 있다. spline 방향 기준은 반드시 HitWorldPos다.
		// 이 후보의 node는 실제 hit 거리로만 정한다. AimGuideLockAlpha/DirectionBias는 물리 Flight의
		// 곡선 보간 설정이며 prepared latch 위치를 바꾸지 않는다.
		FRopeContactCandidate Candidate;
		Candidate.bValid = true;
		Candidate.NodeIndex = AimGuideNodeIndex;
		Candidate.Bone = ThrowContext.AimGuideBone;
		Candidate.Mesh = GuideMesh;
		Candidate.Source = ERopeContactCandidateSource::PredictiveFree;
		Candidate.SourceMask = static_cast<uint8>(ERopeContactCandidateSource::PredictiveFree);
		Candidate.WorldPoint = ThrowContext.AimGuideHitWorldPos;
		Candidate.Normal = ThrowContext.AimGuideNormal.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
		Candidate.Penetration = 0.0f;
		Candidate.WrapDirectionScore = 0.0f;

		OutCandidate.Candidate = Candidate;
		OutCandidate.AngleAlpha = 1.0f;
		OutCandidate.DistanceAlpha = FMath::Clamp(HitDistance / FMath::Max(Preview.Radius, KINDA_SMALL_NUMBER), 0.0f, 1.0f);
		OutCandidate.ArcStartAngleDegrees = 0.0f;
		OutCandidate.Direction = HitDir;
		OutCandidate.bForceDirectPathToContact = true;
		return true;
	}

	bool FindThrowPreviewContactCandidate(const FRopeArcPreviewData& Preview, const TArray<IRopeCollider*>& Colliders,
		float RopeRadius, const FRopeWrapConfig& WrapConfig, const FRopeSimState& Sim,
		float SampleStep, float QueryRadius,
		const TFunction<bool(const USceneComponent*, FName)>& CanWrapTarget,
		FThrowPreviewContactCandidate& OutCandidate,
		FString* OutFailureReason = nullptr)
	{
		OutCandidate = FThrowPreviewContactCandidate();
		if (Preview.Radius <= KINDA_SMALL_NUMBER)
		{
			RopeMath::SetPreviewFailureReason(OutFailureReason,
				FString::Printf(TEXT("free search rejected: arc radius too small (radius=%.2f)"), Preview.Radius));
			return false;
		}
		if (Colliders.Num() == 0)
		{
			RopeMath::SetPreviewFailureReason(OutFailureReason, TEXT("free search rejected: no frame colliders"));
			return false;
		}
		if (Sim.Num() < 2)
		{
			RopeMath::SetPreviewFailureReason(OutFailureReason,
				FString::Printf(TEXT("free search rejected: rope sim has too few nodes (nodes=%d)"), Sim.Num()));
			return false;
		}

		constexpr int32 MaxPreviewAngleSamples = 24;
		constexpr int32 MaxPreviewRadialSamples = 16;
		constexpr int32 MaxPreviewTotalSamples = 256;

		const int32 AngleSamples = FMath::Clamp(Preview.SegmentCount, 1, MaxPreviewAngleSamples);
		const float RadialStep = FMath::Max(SampleStep, 1.0f);
		const int32 RequestedRadialSamples = FMath::Max(1, FMath::CeilToInt(Preview.Radius / RadialStep));
		const int32 TotalLimitedRadialSamples = FMath::Max(1, MaxPreviewTotalSamples / (AngleSamples + 1));
		const int32 RadialSamples = FMath::Clamp(RequestedRadialSamples, 1,
			FMath::Min(MaxPreviewRadialSamples, TotalLimitedRadialSamples));
		const float EffectiveQueryRadius = QueryRadius > KINDA_SMALL_NUMBER
			? QueryRadius
			: FMath::Max(RopeRadius, WrapConfig.ContactQueryRadius);

		FBox PreviewBounds(EForceInit::ForceInit);
		PreviewBounds += Preview.Origin;
		for (int32 AngleIndex = 0; AngleIndex <= AngleSamples; ++AngleIndex)
		{
			const float AngleAlpha = static_cast<float>(AngleIndex) / static_cast<float>(AngleSamples);
			const FVector Direction = RopeMath::ArcDirectionAtAlpha(Preview.AimDir, Preview.GuideUp, Preview.SweepAngleDegrees, AngleAlpha);
			if (!Direction.IsNearlyZero())
			{
				PreviewBounds += Preview.Origin + Direction * Preview.Radius;
			}
		}
		PreviewBounds = PreviewBounds.ExpandBy(EffectiveQueryRadius);

		TArray<IRopeCollider*, TInlineAllocator<8>> CandidateColliders;
		for (IRopeCollider* Collider : Colliders)
		{
			if (Collider && Collider->GetWorldBounds().ExpandBy(EffectiveQueryRadius).Intersect(PreviewBounds))
			{
				CandidateColliders.Add(Collider);
			}
		}
		if (CandidateColliders.Num() == 0)
		{
			RopeMath::SetPreviewFailureReason(OutFailureReason,
				FString::Printf(TEXT("free search found no colliders inside arc bounds (frameColliders=%d, radius=%.1f, queryRadius=%.1f)"),
					Colliders.Num(), Preview.Radius, EffectiveQueryRadius));
			return false;
		}

		const float SegmentLength = FMath::Max(Sim.SegmentLength, 1.0f);
		const FVector ArcStartDirection = RopeMath::ArcDirectionAtAlpha(Preview.AimDir, Preview.GuideUp, Preview.SweepAngleDegrees, 0.0f);
		bool bFoundCandidate = false;
		FThrowPreviewContactCandidate BestCandidate;
		int32 HitCount = 0;
		int32 InvalidHitCount = 0;
		for (int32 AngleIndex = 0; AngleIndex <= AngleSamples; ++AngleIndex)
		{
			const float AngleAlpha = static_cast<float>(AngleIndex) / static_cast<float>(AngleSamples);
			const FVector Direction = RopeMath::ArcDirectionAtAlpha(Preview.AimDir, Preview.GuideUp, Preview.SweepAngleDegrees, AngleAlpha);
			if (Direction.IsNearlyZero())
			{
				continue;
			}

			for (int32 RadialIndex = 1; RadialIndex <= RadialSamples; ++RadialIndex)
			{
				const float DistanceAlpha = static_cast<float>(RadialIndex) / static_cast<float>(RadialSamples);
				const float Distance = Preview.Radius * DistanceAlpha;
				const FVector SamplePoint = Preview.Origin + Direction * Distance;

				for (const IRopeCollider* Collider : CandidateColliders)
				{
					const FRopeContact Contact = Collider->Query(SamplePoint, EffectiveQueryRadius);
					if (!Contact.bHit || Contact.Bone.IsNone() || !Contact.SourceMesh)
					{
						if (Contact.bHit)
						{
							++InvalidHitCount;
						}
						continue;
					}
					// 서브클래스 wrap 대상 게이트 — aim 경로(FindAimRayBoneHit)와 같은 기준으로 거른다.
					// 이 검사가 빠지면 aim이 금지한 대상을 arc 탐색이 주워 preview와 aim의 판정이 갈린다
					// (throw 진입점이 결국 거부하므로 증상은 "보이는데 안 던져짐"이 된다).
					// 미설정(단위 테스트/게이트 없는 호출자)이면 전부 허용 — CanWrapTarget 기본 구현과 같다.
					if (CanWrapTarget && !CanWrapTarget(Contact.SourceMesh, Contact.Bone))
					{
						++InvalidHitCount;
						continue;
					}
					++HitCount;

					const FVector OriginToContact = Contact.SurfacePoint - Preview.Origin;
					const FVector ContactDirection = OriginToContact.GetSafeNormal(KINDA_SMALL_NUMBER, Direction);
					const float ArcStartDot = FMath::Clamp(FVector::DotProduct(ArcStartDirection, ContactDirection),
						-1.0f, 1.0f);

					FThrowPreviewContactCandidate Candidate;
					Candidate.Candidate = FRopeFlightContactDetector::MakeCandidate(
						FMath::Clamp(FMath::RoundToInt(Distance / SegmentLength), 1, Sim.Num() - 1),
						Contact);
					Candidate.Candidate.Source = ERopeContactCandidateSource::PredictiveFree;
					Candidate.Candidate.SourceMask = static_cast<uint8>(ERopeContactCandidateSource::PredictiveFree);
					Candidate.AngleAlpha = AngleAlpha;
					Candidate.DistanceAlpha = DistanceAlpha;
					Candidate.ArcStartAngleDegrees = FMath::RadiansToDegrees(FMath::Acos(ArcStartDot));
					Candidate.Direction = Direction;
					if (Candidate.Candidate.bValid &&
						(!bFoundCandidate || IsBetterThrowPreviewContactCandidate(Candidate, BestCandidate)))
					{
						BestCandidate = Candidate;
						bFoundCandidate = true;
					}
				}
			}
		}

		if (!bFoundCandidate)
		{
			RopeMath::SetPreviewFailureReason(OutFailureReason,
				FString::Printf(TEXT("free search found no valid contact (candidateColliders=%d, angleSamples=%d, radialSamples=%d, hits=%d, invalidHits=%d, queryRadius=%.1f)"),
					CandidateColliders.Num(), AngleSamples, RadialSamples, HitCount, InvalidHitCount, EffectiveQueryRadius));
			return false;
		}

		OutCandidate = BestCandidate;
		return true;
	}

	FRopeSimState BuildThrowPreviewSim(const FRopeSimState& SourceSim, const FRopeArcPreviewData& Preview,
		const FThrowPreviewContactCandidate& ContactCandidate)
	{
		FRopeSimState PreviewSim = SourceSim;
		const int32 NumNodes = SourceSim.Num();
		if (NumNodes < 2)
		{
			return PreviewSim;
		}

		PreviewSim.Positions.SetNum(NumNodes);
		PreviewSim.PrevPositions.SetNum(NumNodes);
		PreviewSim.InvMass.SetNum(NumNodes);

		const int32 LatchNode = FMath::Clamp(ContactCandidate.Candidate.NodeIndex, 1, NumNodes - 1);
		const float SegmentLength = FMath::Max(SourceSim.SegmentLength, 1.0f);
		const FVector SurfacePoint = ContactCandidate.Candidate.WorldPoint;
		const FVector ToSurface = SurfacePoint - Preview.Origin;
		const float SurfaceDistance = FMath::Max(ToSurface.Size(), SegmentLength);
		const FVector ApproachDir = ToSurface.GetSafeNormal(KINDA_SMALL_NUMBER, ContactCandidate.Direction);
		const FVector TailDir = ContactCandidate.Direction.GetSafeNormal(KINDA_SMALL_NUMBER, ApproachDir);

		for (int32 NodeIndex = 0; NodeIndex < NumNodes; ++NodeIndex)
		{
			FVector Position = FVector::ZeroVector;
			if (NodeIndex <= LatchNode)
			{
				const float Alpha = LatchNode > 0
					? static_cast<float>(NodeIndex) / static_cast<float>(LatchNode)
					: 0.0f;
				if (NodeIndex == LatchNode)
				{
					Position = SurfacePoint;
				}
				else if (ContactCandidate.bForceDirectPathToContact)
				{
					// aim ray 조준에서는 preview arc를 섞지 않는다.
					// hit 방향이 이미 확정된 상태이므로 origin->hit 직선이 spline prefix의 권위 있는 모양이다.
					Position = FMath::Lerp(Preview.Origin, SurfacePoint, Alpha);
				}
				else
				{
					const float ArcAlpha = FMath::Lerp(0.0f, ContactCandidate.AngleAlpha, Alpha);
					const FVector ArcDirection = RopeMath::ArcDirectionAtAlpha(Preview.AimDir, Preview.GuideUp, Preview.SweepAngleDegrees, ArcAlpha);
					const FVector ArcPosition = Preview.Origin + ArcDirection * (SurfaceDistance * Alpha);
					const FVector ContactLinePosition = FMath::Lerp(Preview.Origin, SurfacePoint, Alpha);
					Position = FMath::Lerp(ArcPosition, ContactLinePosition, Alpha * Alpha);
				}
			}
			else
			{
				Position = SurfacePoint + TailDir * (static_cast<float>(NodeIndex - LatchNode) * SegmentLength);
			}

			PreviewSim.Positions[NodeIndex] = Position;
			PreviewSim.PrevPositions[NodeIndex] = Position;
			PreviewSim.InvMass[NodeIndex] = (NodeIndex == 0 && SourceSim.bStartPinned) ? 0.0f : 1.0f;
		}

		PreviewSim.SegmentLength = SegmentLength;
		PreviewSim.RopeLength = SegmentLength * static_cast<float>(NumNodes - 1);
		return PreviewSim;
	}

	// OutColliderStorage는 호출자가 소유한다 — FContext가 배열을 참조로 들기 때문에 임시 저장소를
	// 여기서 만들면 dangling이 된다. 런타임(URopeComponent::MakeWrappingContext)과 같은 게이트를
	// 태워, preview가 고른 대상과 실제 감김 경로가 같은 집합을 보게 한다.
	FRopeWrappingPhase::FContext MakeWrappingContext(const FRopeThrowPreviewBuilder::FInput& Input,
		TArray<IRopeCollider*>& OutColliderStorage)
	{
		RopeWrapTargets::FilterWrappableColliders(GetColliders(Input),
			[&Input](const USceneComponent* Mesh, FName Bone)
			{
				// 미설정(단위 테스트/게이트 없는 호출자)이면 전부 허용 — CanWrapTarget 기본 구현과 같다.
				return !Input.CanWrapTarget || Input.CanWrapTarget(Mesh, Bone);
			},
			OutColliderStorage);

		FRopeWrappingPhase::FContext Ctx{
			Input.WrapConfig,
			OutColliderStorage,
			Input.RopeRadius,
			Input.OwnerName
		};
		Ctx.ResolveMode = Input.ResolveMode;
		return Ctx;
	}

	void BuildPreparedAnchorsFromCenterline(const TArray<FVector>& Centerline, const FRopeContactCandidate& Candidate,
		const USceneComponent* Mesh, TArray<FRopeSurfaceAnchor>& OutAnchors)
	{
		OutAnchors.Reset();
		if (!Mesh || Candidate.Bone.IsNone())
		{
			return;
		}

		const FTransform BoneXform = ResolveBindingWorld(Mesh, Candidate.Bone);
		const int32 FirstNode = FMath::Clamp(Candidate.NodeIndex, 1, Centerline.Num() - 1);
		for (int32 NodeIndex = FirstNode; NodeIndex < Centerline.Num(); ++NodeIndex)
		{
			FRopeSurfaceAnchor Anchor;
			Anchor.NodeIndex = NodeIndex;
			Anchor.Bone = Candidate.Bone;
			Anchor.Mesh = Mesh;
			Anchor.LocalSurfacePosition = BoneXform.InverseTransformPosition(Centerline[NodeIndex]);
			Anchor.LocalNormal = FVector::UpVector;
			Anchor.LocalTangent = FVector::ForwardVector;
			if (Centerline.IsValidIndex(NodeIndex + 1))
			{
				Anchor.LocalTangent = BoneXform.InverseTransformVectorNoScale(
					Centerline[NodeIndex + 1] - Centerline[NodeIndex]).GetSafeNormal(
						KINDA_SMALL_NUMBER, FVector::ForwardVector);
			}
			Anchor.StartWorldPosition = Centerline[NodeIndex];
			Anchor.SurfaceOffset = 0.0f;
			Anchor.RopeDistance = 0.0f;
			OutAnchors.Add(Anchor);
		}
	}

	void ForceAimGuidePrefixToHit(const FRopeThrowPreviewBuilder::FInput& Input,
		const FRopeContactCandidate& Candidate, TArray<FVector>& InOutPreviewPoints)
	{
		if (!Input.ThrowContext.bHasAimGuideHit ||
			!InOutPreviewPoints.IsValidIndex(0) ||
			!InOutPreviewPoints.IsValidIndex(Candidate.NodeIndex))
		{
			return;
		}

		const FVector Origin = Input.ThrowContext.Origin;
		const FVector Hit = Input.ThrowContext.AimGuideHitWorldPos;
		if ((Hit - Origin).IsNearlyZero())
		{
			return;
		}

		// 주의: 이 함수는 ③의 확정 RenderPreview 전용이다. 물리 Flight의 WhipGuide에는
		// 호출되지 않으므로, 여기서 HitPoint를 고정해도 Flight 노드가 미리 고정되는 현상과는 무관하다.
		// BuildPreviewCenterline은 latch 이후 wrapping path를 만들면서 latch node를 표면 path로 다시 덮을 수 있다.
		// aim ray 조준에서는 화면에 보이는 spline prefix가 반드시 ray hit point를 향해야 하므로
		// 최종 렌더 포인트 생성 후에도 시작점부터 latch node까지를 Origin->Hit 직선으로 고정한다.
		const int32 LastPrefixNode = FMath::Clamp(Candidate.NodeIndex, 1, InOutPreviewPoints.Num() - 1);
		for (int32 NodeIndex = 0; NodeIndex <= LastPrefixNode; ++NodeIndex)
		{
			const float Alpha = static_cast<float>(NodeIndex) / static_cast<float>(LastPrefixNode);
			InOutPreviewPoints[NodeIndex] = FMath::Lerp(Origin, Hit, Alpha);
		}
	}

	// Pierce(꽂힘): 밧줄 끝(팁=창)이 꽂힘 지점에 오도록 손 원점 → 꽂힘 지점 직선으로 전체 노드를 편다.
	// 팁 뒤로 남는 로프는 없다 — 여분 길이는 손~팁 사이 slack이며 물리(솔버)가 drape로 처리한다.
	void BuildPierceStraightCenterline(const FRopeThrowPreviewBuilder::FInput& Input,
		const FRopeContactCandidate& Candidate, const FRopeSimState& SourceSim, TArray<FVector>& OutCenterline)
	{
		OutCenterline = SourceSim.Positions;
		const int32 N = OutCenterline.Num();
		if (N < 2)
		{
			return;
		}

		const FVector Origin = Input.ThrowContext.Origin;
		const FVector Hit = Candidate.WorldPoint;
		const int32 Last = N - 1;

		// 손(0)~팁(Last) 전체를 직선으로. 팁(마지막 노드)이 꽂힘 지점 = 창이 박히는 곳. 팁 뒤 여분 없음.
		for (int32 NodeIndex = 0; NodeIndex <= Last; ++NodeIndex)
		{
			const float Alpha = static_cast<float>(NodeIndex) / static_cast<float>(Last);
			OutCenterline[NodeIndex] = FMath::Lerp(Origin, Hit, Alpha);
		}
	}

	bool BuildPreparedFromCandidate(const FRopeThrowPreviewBuilder::FInput& Input,
		const FRopeContactCandidate& Candidate, const FRopeSimState& SourceSim,
		FRopePreparedThrowPreview& OutPrepared, FString* OutFailureReason)
	{
		OutPrepared.Reset();
		const USceneComponent* Mesh = Candidate.Mesh;
		if (!Candidate.bValid || !Mesh || Candidate.Bone.IsNone() ||
			!SourceSim.Positions.IsValidIndex(Candidate.NodeIndex))
		{
			RopeMath::SetPreviewFailureReason(OutFailureReason,
				FString::Printf(TEXT("wrap preview candidate invalid (valid=%d, mesh=%s, bone=%s, node=%d, sourceNodes=%d)"),
					Candidate.bValid ? 1 : 0, *GetNameSafe(Mesh), *Candidate.Bone.ToString(), Candidate.NodeIndex,
					SourceSim.Num()));
			return false;
		}

		const FVector NormalWorld = Candidate.Normal.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
		FVector TangentWorld = FVector::ForwardVector;
		if (SourceSim.Positions.IsValidIndex(Candidate.NodeIndex + 1))
		{
			TangentWorld = SourceSim.Positions[Candidate.NodeIndex + 1] - SourceSim.Positions[Candidate.NodeIndex];
		}
		else if (SourceSim.Positions.IsValidIndex(Candidate.NodeIndex - 1))
		{
			TangentWorld = SourceSim.Positions[Candidate.NodeIndex] - SourceSim.Positions[Candidate.NodeIndex - 1];
		}
		else
		{
			TangentWorld = FRopeFlightContactDetector::ExpectedWrapTangent(SourceSim, Candidate, Input.FallbackForward);
		}
		TangentWorld = (TangentWorld - FVector::DotProduct(TangentWorld, NormalWorld) * NormalWorld)
			.GetSafeNormal(KINDA_SMALL_NUMBER, RopeMath::AnyTangentFromNormal(NormalWorld));

		const FTransform BoneXform = ResolveBindingWorld(Mesh, Candidate.Bone);

		FRopeSurfaceAnchor LatchAnchor;
		LatchAnchor.NodeIndex = Candidate.NodeIndex;
		LatchAnchor.Bone = Candidate.Bone;
		LatchAnchor.Mesh = Mesh;
		LatchAnchor.LocalSurfacePosition = BoneXform.InverseTransformPosition(Candidate.WorldPoint);
		LatchAnchor.LocalNormal = BoneXform.InverseTransformVectorNoScale(NormalWorld)
			.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);
		LatchAnchor.LocalTangent = BoneXform.InverseTransformVectorNoScale(TangentWorld)
			.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
		LatchAnchor.StartWorldPosition = SourceSim.Positions[Candidate.NodeIndex];
		LatchAnchor.SurfaceOffset = FMath::Max(0.0f, Input.RopeRadius);
		LatchAnchor.RopeDistance = 0.0f;

		// Pierce(③ 전용): 감김 경로 빌드(BuildPreviewCenterline)와 경로 앵커 확장을 건너뛰고,
		// aim-hit 접점에 단일 앵커로 성립한다. RenderPreview는 손→꽂힘 지점 직선(연출용).
		// 이후 FinishGuidedThrow가 Anchors(=1개)를 그대로 Wrapped seed로 승격한다(커밋 경로 무변경).
		if (Input.TipEngagement == ERopeTipEngagement::Pierce)
		{
			// 창(팁)이 꽂히는 것이므로 앵커는 거리 기반 접점 노드(Candidate.NodeIndex)가 아니라
			// 밧줄 끝(마지막 노드 = 팁 mesh 위치)이어야 한다. 그러지 않으면 안쪽 노드가 고정되고
			// 팁 + 여분 로프가 접점 아래로 늘어진다. 앵커 로컬 위치는 이미 꽂힘 지점(Candidate.WorldPoint)이다.
			LatchAnchor.NodeIndex = SourceSim.Num() - 1;
			LatchAnchor.StartWorldPosition = Candidate.WorldPoint;

			TArray<FVector> StraightPoints;
			BuildPierceStraightCenterline(Input, Candidate, SourceSim, StraightPoints);

			OutPrepared.RenderPreview.Points = MoveTemp(StraightPoints);
			OutPrepared.RenderPreview.Radius = FMath::Max(0.1f, Input.RopeRadius * 1.05f);
			OutPrepared.RenderPreview.NumSides = FMath::Clamp(Input.RopeNumSides, 3, 32);
			if (!OutPrepared.RenderPreview.IsValid())
			{
				RopeMath::SetPreviewFailureReason(OutFailureReason,
					FString::Printf(TEXT("pierce preview output invalid (points=%d, node=%d, sourceNodes=%d)"),
						OutPrepared.RenderPreview.Points.Num(), Candidate.NodeIndex, SourceSim.Num()));
				return false;
			}

			OutPrepared.bValid = true;
			OutPrepared.ThrowContext = Input.ThrowContext;
			OutPrepared.PreviewSim = SourceSim;
			OutPrepared.Contact = Candidate;
			OutPrepared.LatchAnchor = LatchAnchor;
			OutPrepared.Mesh = Mesh;
			OutPrepared.Bone = Candidate.Bone;
			OutPrepared.BuildTimeSeconds = FPlatformTime::Seconds();
			OutPrepared.Anchors.Reset();
			OutPrepared.Anchors.Add(LatchAnchor); // 단일 앵커 = Pierce의 정상 형태(AnchorCount=1)

			return OutPrepared.IsValid();
		}

		TArray<FVector> PreviewPoints;
		FRopeWrappingPhase PreviewWrappingPhase;
		// 게이트 통과 collider 저장소 — FContext보다 오래 살아야 한다(참조 보유).
		TArray<IRopeCollider*> WrappableColliders;
		if (!PreviewWrappingPhase.BuildPreviewCenterline(LatchAnchor, Mesh, Candidate.Bone,
			SourceSim, MakeWrappingContext(Input, WrappableColliders), PreviewPoints))
		{
			RopeMath::SetPreviewFailureReason(OutFailureReason,
				FString::Printf(TEXT("wrap preview centerline build failed (mesh=%s, bone=%s, node=%d, sourceNodes=%d)"),
				*GetNameSafe(Mesh), *Candidate.Bone.ToString(), Candidate.NodeIndex, SourceSim.Num()));
			return false;
		}
		ForceAimGuidePrefixToHit(Input, Candidate, PreviewPoints);

		OutPrepared.RenderPreview.Points = MoveTemp(PreviewPoints);
		OutPrepared.RenderPreview.Radius = FMath::Max(0.1f, Input.RopeRadius * 1.05f);
		OutPrepared.RenderPreview.NumSides = FMath::Clamp(Input.RopeNumSides, 3, 32);
		if (!OutPrepared.RenderPreview.IsValid())
		{
			RopeMath::SetPreviewFailureReason(OutFailureReason,
				FString::Printf(TEXT("wrap preview output invalid (points=%d, radius=%.2f, sides=%d)"),
					OutPrepared.RenderPreview.Points.Num(), OutPrepared.RenderPreview.Radius,
					OutPrepared.RenderPreview.NumSides));
			return false;
		}

		OutPrepared.bValid = true;
		OutPrepared.ThrowContext = Input.ThrowContext;
		OutPrepared.PreviewSim = SourceSim;
		OutPrepared.Contact = Candidate;
		OutPrepared.LatchAnchor = LatchAnchor;
		OutPrepared.Mesh = Mesh;
		OutPrepared.Bone = Candidate.Bone;
		OutPrepared.BuildTimeSeconds = FPlatformTime::Seconds();
		BuildPreparedAnchorsFromCenterline(OutPrepared.RenderPreview.Points, Candidate, Mesh, OutPrepared.Anchors);
		if (OutPrepared.Anchors.Num() == 0)
		{
			OutPrepared.Anchors.Add(LatchAnchor);
		}

		return OutPrepared.IsValid();
	}
}

bool FRopeThrowPreviewBuilder::BuildFreePreparedPreview(const FInput& Input, FRopePreparedThrowPreview& OutPrepared,
	FString* OutFailureReason)
{
	OutPrepared.Reset();
	const FRopeSimState* Sim = Input.Sim;
	if (!Sim)
	{
		RopeMath::SetPreviewFailureReason(OutFailureReason, TEXT("free search rejected: no rope sim"));
		return false;
	}

	FRopeArcPreviewData ArcPreview;
	if (!BuildThrowArcPreview(Input, ArcPreview, OutFailureReason))
	{
		return false;
	}

	FThrowPreviewContactCandidate ContactCandidate;
	if (!BuildAimGuideHitCandidate(ArcPreview, Input.ThrowContext, *Sim, ContactCandidate))
	{
		// aim hit이 없을 때 arc 전체를 다시 뒤지면 "조준하지 않은" 옆 대상이 선택된다 —
		// BuildAimGuideHitCandidate 주석이 hit 경로에 대해 이미 경고한 그 위험이 miss 경로로 샌 것이다.
		// 아래 두 경우엔 재탐색하지 않고 실패로 끝낸다. 그러면 호출자(ThrowWithContext ③ 분기)가
		// StartFreeGuidedThrow(레이 끝점 허공 아치)로 가고, 조준이 빗나가면 안 꽂히는 게 정상 결과다.
		//   - 조준 ray가 돌았는데 대상을 못 잡음: ③ 계약상 보장 대상은 "조준한 대상"뿐이다.
		//   - Pierce: 창은 조준한 곳에 꽂히는 것이 전부라 arc 탐색(최대 SweepAngleDegrees 폭)이
		//     의미를 갖지 않는다. 조준 ray가 없는 BP 직행/AI라도 방향만 보고 옆 대상에 꽂으면 안 된다.
		// 남은 하나(조준 없는 BP 직행/AI + Cinch)만 arc 탐색으로 대상을 찾는다 — "이 방향으로 던져
		// 거기 있는 걸 감아라"는 감김 모델에서는 성립하는 요청이다.
		if (Input.ThrowContext.bAimRayEvaluated)
		{
			RopeMath::SetPreviewFailureReason(OutFailureReason,
				TEXT("prepared preview rejected: aim ray found no target (aimed throw does not re-search the arc)"));
			return false;
		}
		if (Input.TipEngagement == ERopeTipEngagement::Pierce)
		{
			RopeMath::SetPreviewFailureReason(OutFailureReason,
				TEXT("prepared preview rejected: pierce requires an aim hit (arc search is wrap-only)"));
			return false;
		}
		if (!FindThrowPreviewContactCandidate(ArcPreview, GetColliders(Input), Input.RopeRadius, Input.WrapConfig, *Sim,
			Input.SampleStep, Input.QueryRadius, Input.CanWrapTarget, ContactCandidate, OutFailureReason))
		{
			return false;
		}
	}

	FRopeSimState PreviewSim = BuildThrowPreviewSim(*Sim, ArcPreview, ContactCandidate);
	ContactCandidate.Candidate.NodeIndex = FMath::Clamp(ContactCandidate.Candidate.NodeIndex, 1, PreviewSim.Num() - 1);
	return BuildPreparedFromCandidate(
		Input, ContactCandidate.Candidate, PreviewSim, OutPrepared, OutFailureReason);
}

