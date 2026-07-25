// Copyright Epic Games, Inc. All Rights Reserved.

#include "Logic/RopeThrowPreviewBuilder.h"

#include "Collision/RopeCollider.h"
// ResolveBindingWorld — 랩 바인딩(본/소켓/컴포넌트) 트랜스폼 해석의 단일 지점(seam A).
#include "Core/RopeWrapTarget.h"
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

	struct FThrowPreviewContactCandidate
	{
		FRopeContactCandidate Candidate;
		/** 접점 뒤로 남는 노드를 펼칠 방향(origin→hit). 접점이 마지막 노드면 쓰이지 않는다. */
		FVector Direction = FVector::ForwardVector;
	};

	bool BuildAimGuideHitCandidate(const FRopeThrowContext& ThrowContext,
		const FRopeSimState& Sim, FThrowPreviewContactCandidate& OutCandidate)
	{
		OutCandidate = FThrowPreviewContactCandidate();
		const USceneComponent* GuideMesh = ThrowContext.AimGuideMesh.Get();
		if (!ThrowContext.bHasAimGuideHit || !GuideMesh || ThrowContext.AimGuideBone.IsNone() || Sim.Num() < 2)
		{
			return false;
		}

		const FVector ToHit = ThrowContext.AimGuideHitWorldPos - ThrowContext.Origin;
		const float HitDistance = ToHit.Size();
		const FVector HitDir = ToHit.GetSafeNormal();
		if (HitDistance <= KINDA_SMALL_NUMBER || HitDir.IsNearlyZero())
		{
			return false;
		}

		const float SegmentLength = FMath::Max(Sim.SegmentLength, 1.0f);
		const int32 DistanceNodeIndex = FMath::Clamp(FMath::RoundToInt(HitDistance / SegmentLength), 1, Sim.Num() - 1);
		// LockAlpha는 spline의 공간 보간 구간일 뿐 latch 위치가 아니다. 실제 hit 거리의 노드를 사용한다.
		const int32 AimGuideNodeIndex = DistanceNodeIndex;

		// aim ray 조준은 이미 SDF/collider swept query로 본을 고른 상태다.
		// prepared 후보는 그 hit을 그대로 쓴다 — 던지기 방향 주변을 다시 훑으면 다른 본/다른 방향이 뽑힌다.
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
		OutCandidate.Direction = HitDir;
		return true;
	}

	FRopeSimState BuildThrowPreviewSim(const FRopeSimState& SourceSim, const FVector& Origin,
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
		const FVector ToSurface = SurfacePoint - Origin;
		const FVector ApproachDir = ToSurface.GetSafeNormal(KINDA_SMALL_NUMBER, ContactCandidate.Direction);
		const FVector TailDir = ContactCandidate.Direction.GetSafeNormal(KINDA_SMALL_NUMBER, ApproachDir);

		for (int32 NodeIndex = 0; NodeIndex < NumNodes; ++NodeIndex)
		{
			FVector Position = FVector::ZeroVector;
			if (NodeIndex <= LatchNode)
			{
				// 조준 hit 방향이 이미 확정돼 있으므로 origin->hit 직선이 spline prefix의 권위 있는 모양이다.
				const float Alpha = LatchNode > 0
					? static_cast<float>(NodeIndex) / static_cast<float>(LatchNode)
					: 0.0f;
				Position = (NodeIndex == LatchNode)
					? SurfacePoint
					: FMath::Lerp(Origin, SurfacePoint, Alpha);
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

		// ③(Guaranteed) = Pierce: 감김 경로 빌드(BuildPreviewCenterline)와 경로 앵커 확장을 건너뛰고,
		// aim-hit 접점에 단일 앵커로 성립한다. RenderPreview는 손→꽂힘 지점 직선(연출용).
		// 이후 FinishGuidedThrow가 Anchors(=1개)를 그대로 Wrapped seed로 승격한다(커밋 경로 무변경).
		if (Input.ResolveMode == ERopeWrapResolveMode::GuaranteedWrap)
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
			OutPrepared.LatchAnchor = LatchAnchor;
			OutPrepared.Mesh = Mesh;
			OutPrepared.Bone = Candidate.Bone;
			OutPrepared.Anchors.Reset();
			OutPrepared.Anchors.Add(LatchAnchor); // 단일 앵커 = Pierce의 정상 형태(AnchorCount=1)

			return OutPrepared.IsValid();
		}

		TArray<FVector> PreviewPoints;
		FRopeWrappingPhase PreviewWrappingPhase;
		// 게이트 통과 collider 저장소 — FContext보다 오래 살아야 한다(참조 보유).
		TArray<IRopeCollider*> WrappableColliders;
		if (!PreviewWrappingPhase.BuildPreviewCenterline(LatchAnchor,
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
		OutPrepared.LatchAnchor = LatchAnchor;
		OutPrepared.Mesh = Mesh;
		OutPrepared.Bone = Candidate.Bone;
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
		RopeMath::SetPreviewFailureReason(OutFailureReason, TEXT("prepared preview rejected: no rope sim"));
		return false;
	}

	FThrowPreviewContactCandidate ContactCandidate;
	if (!BuildAimGuideHitCandidate(Input.ThrowContext, *Sim, ContactCandidate))
	{
		// prepared preview는 **조준한 대상**에만 성립한다. aim hit이 없다고 던지기 방향 주변을 훑어
		// 후보를 고르면 조준과 무관한 옆 대상이 뽑히고, 화면의 miss 표시와 preview 연결선이 어긋난다.
		// 그래서 대안 탐색 없이 여기서 끝낸다 — 호출자(ThrowWithContext ③ 분기)가 StartFreeGuidedThrow
		// (레이 끝점 허공 아치)로 폴백하며, 조준이 빗나가면 안 꽂히는 게 정상 결과다.
		// 사유는 "조준이 돌았는데 빗나감"과 "조준 흐름 자체가 없음(BP 직행/AI)"을 구분한다 — 로그만 보고
		// 조준 설정 문제인지 호출 경로 문제인지 갈라야 하기 때문이다.
		RopeMath::SetPreviewFailureReason(OutFailureReason, Input.ThrowContext.bAimRayEvaluated
			? TEXT("prepared preview rejected: aim ray found no target")
			: TEXT("prepared preview rejected: requires an aim hit"));
		return false;
	}

	FRopeSimState PreviewSim = BuildThrowPreviewSim(*Sim, Input.ThrowContext.Origin, ContactCandidate);
	ContactCandidate.Candidate.NodeIndex = FMath::Clamp(ContactCandidate.Candidate.NodeIndex, 1, PreviewSim.Num() - 1);
	return BuildPreparedFromCandidate(
		Input, ContactCandidate.Candidate, PreviewSim, OutPrepared, OutFailureReason);
}

