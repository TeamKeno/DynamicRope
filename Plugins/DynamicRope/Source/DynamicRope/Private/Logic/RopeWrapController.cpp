// Copyright Epic Games, Inc. All Rights Reserved.

#include "Logic/RopeWrapController.h"
// FRopeBindingFrame + ResolveBindingWorld (바인딩 배선 seam A)
#include "Core/RopeWrapTarget.h"
#include "DynamicRopeLog.h"

void FRopeWrapController::BeginWrap(const FRopeSimState& Sim, const FRopeWrapState& Seed, FRopeNodeOverrideFrame& OutFrame)
{
	State = Seed;

	// 붙잡힌 bone 을 소유한 mesh 는 시드에 실려 온다(접촉의 SourceMesh 에서 전파 — cross-actor 포함).
	// 없으면 잘못된 시드다: 아무것도 latch 하지 않고 상태를 비워 "감긴 척"하는 상태를 남기지 않는다.
	const USceneComponent* Mesh = State.Mesh.Get();
	if (!Mesh)
	{
		UE_LOG(LogRopeWrap, Warning, TEXT("BeginWrap aborted: no mesh for bone %s (seed has no mesh) — nodes stay dynamic."),
			*State.BoneName.ToString());
		State.Reset();
		return;
	}

	UE_LOG(LogRopeWrap, Log, TEXT("BeginWrap: bone=%s, %d latched node(s), mesh=%s"),
		*State.BoneName.ToString(), State.Latched.Num(), *Mesh->GetName());

	// 각 접촉 노드의 현재 월드 위치를 bone-local 로 변환하여 동결한다(InvMass 0).
	// 이 시점부터 노드는 솔버가 아니라 logic(skinning 된 bone)에 의해 구동된다.
// Anchor가 없는 legacy seed면 Latched에서 임시 Anchor를 만든다.
	if (State.Anchors.Num() == 0)
	{
		for (const FRopeLatchNode& Latch : State.Latched)
		{
			if (!Sim.Positions.IsValidIndex(Latch.NodeIndex))
			{
				continue;
			}

			const FName Bone = Latch.Bone.IsNone() ? State.BoneName : Latch.Bone;
			const FTransform BoneXform = ResolveBindingWorld(Mesh, Bone);
			const FVector World = Sim.Positions[Latch.NodeIndex];

			FRopeSurfaceAnchor Anchor;
			Anchor.NodeIndex = Latch.NodeIndex;
			Anchor.Bone = Bone;
			Anchor.Mesh = Mesh;
			Anchor.LocalSurfacePosition = BoneXform.InverseTransformPosition(World);
			Anchor.LocalNormal = FVector::UpVector;
			Anchor.LocalTangent = FVector::ForwardVector;
			Anchor.StartWorldPosition = World;
			Anchor.SurfaceOffset = 0.0f;
			Anchor.RopeDistance = static_cast<float>(Latch.NodeIndex) * Sim.SegmentLength;

			State.Anchors.Add(Anchor);
		}
	}


	int32 ValidAnchorCount = 0;

	for (FRopeSurfaceAnchor& Anchor : State.Anchors)
	{
		if (!Sim.Positions.IsValidIndex(Anchor.NodeIndex) ||
			!Sim.PrevPositions.IsValidIndex(Anchor.NodeIndex) ||
			!Sim.InvMass.IsValidIndex(Anchor.NodeIndex))
		{
			continue;
		}

		if (Anchor.Bone.IsNone())
		{
			Anchor.Bone = State.BoneName;
		}

		if (!Anchor.Mesh.IsValid())
		{
			Anchor.Mesh = Mesh;
		}

		const USceneComponent* AnchorComp = Anchor.Mesh.Get();
		if (!AnchorComp)
		{
			AnchorComp = Mesh;
		}

		// 바인딩 배선(seam A): 대상 트랜스폼을 단일 지점에서 해석 — 스켈레탈=스키닝 소켓, 정적=컴포넌트 트랜스폼.
		FRopeBindingFrame Binding;
		Binding.Component = AnchorComp;
		Binding.SocketOrBone = Anchor.Bone;
		const FTransform BoneXform = ResolveBindingWorld(Binding);

		const FVector SurfaceWorld =
			BoneXform.TransformPosition(Anchor.LocalSurfacePosition);

		const FVector NormalWorld =
			BoneXform.TransformVectorNoScale(Anchor.LocalNormal)
			.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);

		const FVector World =
			SurfaceWorld + NormalWorld * Anchor.SurfaceOffset;

		OutFrame.EnsureSize(Sim.Num());
		OutFrame.SetPosition(Anchor.NodeIndex, World, /*bZeroVelocity*/ true);
		OutFrame.SetInvMass(Anchor.NodeIndex, 0.0f);

		++ValidAnchorCount;
	}

	if (ValidAnchorCount == 0)
	{
		UE_LOG(LogRopeWrap, Warning, TEXT("BeginWrap aborted: no valid anchors for bone %s"),
			*State.BoneName.ToString());
		State.Reset();
		return;
	}

	UE_LOG(LogRopeWrap, Log, TEXT("BeginWrap: bone=%s, anchors=%d, mesh=%s"),
		*State.BoneName.ToString(), State.Anchors.Num(), *Mesh->GetName());
}

bool FRopeWrapController::Hold(const FRopeSimState& Sim, float Dt, FRopeNodeOverrideFrame& OutFrame)
{
	// bone 이 붙잡힌 mesh 를 따라간다. State.Mesh 는 BeginWrap 에서 확정되어 weak 포인터로
	// 영속화된다(cross-actor 대상일 수 있다). 대상 액터가 파괴되면 weak 가 null 이 되어
	// raw 포인터 역참조(use-after-free) 없이 안전하게 감지된다 — 엉뚱한 bone 으로 노드를
	// 끌어당기지 않도록 폴백 없이 false 를 반환해 호출자가 release 하게 한다.
	const USceneComponent* Mesh = State.Mesh.Get();
	if (!Mesh)
	{
		return false;
	}

	// 새 방식: surface anchor 기반 hold
	if (State.Anchors.Num() > 0)
	{
		for (const FRopeSurfaceAnchor& Anchor : State.Anchors)
		{
			if (!Sim.Positions.IsValidIndex(Anchor.NodeIndex) ||
				!Sim.PrevPositions.IsValidIndex(Anchor.NodeIndex) ||
				!Sim.InvMass.IsValidIndex(Anchor.NodeIndex))
			{
				continue;
			}

			const USceneComponent* AnchorComp = Anchor.Mesh.Get();
			if (!AnchorComp)
			{
				AnchorComp = Mesh;
			}

			const FName Bone = Anchor.Bone.IsNone() ? State.BoneName : Anchor.Bone;
			// 바인딩 배선(seam A): 매 프레임 대상 추종을 단일 지점에서 — 스켈레탈=스키닝 소켓, 정적=컴포넌트 트랜스폼.
			FRopeBindingFrame Binding;
			Binding.Component = AnchorComp;
			Binding.SocketOrBone = Bone;
			const FTransform BoneXform = ResolveBindingWorld(Binding);

			const FVector SurfaceWorld =
				BoneXform.TransformPosition(Anchor.LocalSurfacePosition);

			const FVector NormalWorld =
				BoneXform.TransformVectorNoScale(Anchor.LocalNormal)
				.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);

			const FVector World =
				SurfaceWorld + NormalWorld * Anchor.SurfaceOffset;

			OutFrame.EnsureSize(Sim.Num());
			OutFrame.SetPosition(Anchor.NodeIndex, World, /*bZeroVelocity*/ true);
			OutFrame.SetInvMass(Anchor.NodeIndex, 0.0f);
		}

		return true;
	}

	// 기존 방식 fallback
	// wrap 이 skinning 을 타고 가도록 매 프레임 각 latched 노드를 자신의 (애니메이션된) bone 위에 재배치한다.
	// bone 의 움직임이 솔버로 주입되지 않도록 노드의 속도를 0 으로 둔다(Prev = Pos).
	for (const FRopeLatchNode& Latch : State.Latched)
	{
		if (!Sim.Positions.IsValidIndex(Latch.NodeIndex))
		{
			continue;
		}
		const FTransform BoneXform = ResolveBindingWorld(Mesh, Latch.Bone);
		const FVector World = BoneXform.TransformPosition(Latch.BoneLocalPos);
		OutFrame.EnsureSize(Sim.Num());
		OutFrame.SetPosition(Latch.NodeIndex, World, /*bZeroVelocity*/ true);
		OutFrame.SetInvMass(Latch.NodeIndex, 0.0f);
	}

	return true;
}

bool FRopeWrapController::ComputePull(const FRopeSimState& Sim, float BendThresholdDeg, FRopePullSample& Out) const
{
	Out = FRopePullSample();
	if (!State.IsWrapped())
	{
		return false;
	}

	// 손 쪽 첫 앵커(최소 노드 인덱스): 손~앵커 사이 자유 구간의 장력이 여기로 전달된다.
	// 새 방식(Anchors) 우선, legacy(Latched) 폴백 — Hold와 동일한 우선순위.
	int32 AnchorNode = INDEX_NONE;
	FName AnchorBone = NAME_None;
	for (const FRopeSurfaceAnchor& Anchor : State.Anchors)
	{
		if (Sim.Positions.IsValidIndex(Anchor.NodeIndex)
			&& (AnchorNode == INDEX_NONE || Anchor.NodeIndex < AnchorNode))
		{
			AnchorNode = Anchor.NodeIndex;
			AnchorBone = Anchor.Bone.IsNone() ? State.BoneName : Anchor.Bone;
		}
	}
	if (AnchorNode == INDEX_NONE)
	{
		for (const FRopeLatchNode& Latch : State.Latched)
		{
			if (Sim.Positions.IsValidIndex(Latch.NodeIndex)
				&& (AnchorNode == INDEX_NONE || Latch.NodeIndex < AnchorNode))
			{
				AnchorNode = Latch.NodeIndex;
				AnchorBone = Latch.Bone.IsNone() ? State.BoneName : Latch.Bone;
			}
		}
	}

	// 앵커가 노드 0(손 핀 자체)이면 손 쪽 세그먼트가 없다 → 당김 없음.
	if (AnchorNode <= 0)
	{
		return false;
	}

	// 당김 방향 = 앵커에서 손 쪽으로 로프를 따라 걸으며 찾은 "첫 직선 다리"의 끝 노드를 향하는 방향.
	// 각 스텝에서 다음 세그먼트가 지금까지의 누적 다리 방향(앵커→현재 조준노드 chord)에서 임계 이상 꺾이면
	// 멈춘다. 곧으면 손(노드 0)까지 걸어가 정확히 chord가 되고, 벽/모서리에선 그 직전에 멈춰 첫 다리를 따른다.
	// 누적 chord 기준이라 90도 코너는 뚜렷이 감지하면서 한 노드의 처짐엔 둔감하다.
	//
	// 단, 조준을 anchor-1(세그먼트 1개)에서 시작하면 첫 스텝의 chord도 세그먼트 1개라 노드 노이즈에 취약해,
	// 팽팽한 로프에서도 인접 두 세그먼트가 임계를 넘겨 anchor-1에 조기 종료 → baseline이 1세그먼트로 짧아져
	// 방향 각도 지터가 폭증했다(어제 pull 지터의 주범). 인접 2세그먼트를 무조건 포함해 baseline을 확보한 뒤
	// 코너 판정을 시작한다(손이 더 가까우면 손까지). 잔여 시간 지터/이산 홉은 호출자의 fractional 스무딩이 흡수.
	const float CosThresh = FMath::Cos(FMath::DegreesToRadians(FMath::Clamp(BendThresholdDeg, 1.0f, 179.0f)));
	const FVector AnchorPos = Sim.Positions[AnchorNode];
	// 다리(leg) 단위 walk: 앵커에서 손(노드 0)까지 코너마다 다리를 끊어 걷는다. 첫 다리의 끝(AimNode)은
	// 방향/테더 overshoot의 조준(종전과 동일 산출)이고, 모든 다리의 chord 합(TautChordLen)은 "전 체인 팽팽"
	// 판정의 관측치다 — 처짐은 chord를 rest보다 짧게 만들고, 코너에 걸린 팽팽한 로프는 다리별 chord가
	// rest에 근접해 팽팽으로 남는다(코너는 손해가 아니다).
	int32 LegStart = AnchorNode;
	// 첫 다리는 2세그먼트 시드(가능하면) — 첫 스텝 단일 세그먼트 노이즈 회피(종전 동작).
	int32 LegEnd = FMath::Max(AnchorNode - 2, 0);
	int32 AimNode = INDEX_NONE; // 첫 다리의 끝(아래 첫 바퀴에 확정)
	float ChordSum = 0.0f;
	while (true)
	{
		for (int32 j = LegEnd - 1; j >= 0; --j)
		{
			// LegSoFar = 누적 다리 chord(긴 baseline), NextSeg = 다음 세그먼트.
			const FVector LegSoFar = (Sim.Positions[LegEnd] - Sim.Positions[LegStart]).GetSafeNormal();
			const FVector NextSeg  = (Sim.Positions[j] - Sim.Positions[LegEnd]).GetSafeNormal();
			if (LegSoFar.IsNearlyZero() || NextSeg.IsNearlyZero()
				|| FVector::DotProduct(NextSeg, LegSoFar) < CosThresh)
			{
				// 코너(또는 축퇴) — 직전 노드(LegEnd)가 이 다리의 끝.
				break;
			}
			LegEnd = j;
		}
		if (AimNode == INDEX_NONE)
		{
			AimNode = LegEnd;
		}
		ChordSum += static_cast<float>((Sim.Positions[LegEnd] - Sim.Positions[LegStart]).Size());
		if (LegEnd <= 0)
		{
			break;
		}
		// 다음 다리: 코너 노드에서 재시작(1세그먼트 시드 — chord 합엔 baseline 노이즈 영향이 미미하다).
		LegStart = LegEnd;
		LegEnd = LegEnd - 1;
	}
	const FVector Along = (Sim.Positions[AimNode] - AnchorPos).GetSafeNormal();
	if (Along.IsNearlyZero())
	{
		// 축퇴(조준 노드와 앵커 겹침) — 방향 정의 불가.
		return false;
	}

	Out.bValid = true;
	Out.AnchorNode = AnchorNode;
	Out.AimNode = AimNode;
	Out.Bone = AnchorBone;
	Out.WorldPoint = Sim.Positions[AnchorNode];
	Out.Direction = Along;
	// 앵커-손 쪽 인접 세그먼트(인덱스 AnchorNode-1)의 장력. 아직 솔브 전이면(배열 비어 있음) 0.
	Out.Tension = Sim.SegmentTension.IsValidIndex(AnchorNode - 1) ? Sim.SegmentTension[AnchorNode - 1] : 0.0f;
	// 전 체인 팽팽 관측치: 다리 chord 합 + 자유 구간 rest 길이(소비 = 컴포넌트의 EvaluateChainTautGate).
	Out.TautChordLen = ChordSum;
	Out.FreeRestLen = static_cast<float>(AnchorNode) * Sim.SegmentLength;
	return true;
}

void FRopeWrapController::Release(ERopeReleaseReason Reason)
{
	UE_LOG(LogRopeWrap, Log, TEXT("Release: bone=%s, reason=%d"), *State.BoneName.ToString(), static_cast<int32>(Reason));
	State.Reset();
}
