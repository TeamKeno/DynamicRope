// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeComponent.h"

#include "Components/SkeletalMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Core/RopeWrapTarget.h"
#include "DynamicRopeLog.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Logic/RopeTipPlacement.h"
#include "RopeComponentInternal.h"

using RopeComponentPrivate::ResolveAimGuideHitWorld;

#pragma region Tip_Public_API

FTransform URopeComponent::GetLoadedTipTransform() const
{
	// 기본 구현: Owner의 스켈레탈 메시에서 LoadedHandSocket 소켓 트랜스폼. 없으면 컴포넌트(손) 트랜스폼.
	if (const AActor* Owner = GetOwner())
	{
		if (const USkeletalMeshComponent* Mesh = Owner->FindComponentByClass<USkeletalMeshComponent>())
		{
			if (!LoadedHandSocket.IsNone() && Mesh->DoesSocketExist(LoadedHandSocket))
			{
				return Mesh->GetSocketTransform(LoadedHandSocket);
			}
		}
	}
	return GetComponentTransform();
}
#pragma endregion Tip_Public_API

#pragma region Tip_Mesh

// ===== 팁 부착물(표시 전용) =================================================

void URopeComponent::EnsureTipMesh()
{
	// BeginPlay에서 호출(런타임에 bUseTipMesh를 켜는 경로 대비로 던지기/Loaded 진입에서도 호출 — idempotent).
	// ① Owner에 붙은 태그 컴포넌트를 우선 재사용(파괴 안 함) → ② 없고 TipMesh 에셋이 있으면 스폰(파괴는
	// 우리 몫). 이미 확보돼 있으면 no-op. 질량·충돌 없는 표시 전용이다.
	// 팁 확보의 유일한 경로라, 여기서 막으면 팁 서브시스템 전체가 꺼진다(나머지는 TipMeshComponent 널 가드).
	if (!bUseTipMesh || TipMeshComponent)
	{
		return;
	}

	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}

	// ① 외부 컴포넌트 탐색(태그) — 있으면 재사용하고 소유는 취하지 않는다.
	if (!TipMeshComponentTag.IsNone())
	{
		TArray<UActorComponent*> Tagged =
			Owner->GetComponentsByTag(UStaticMeshComponent::StaticClass(), TipMeshComponentTag);
		if (Tagged.Num() > 0)
		{
			TipMeshComponent = Cast<UStaticMeshComponent>(Tagged[0]);
			bTipMeshSpawnedByUs = false;
			// 저작 기준선 캡처 — 배치(SetWorldTransform)가 덮어쓰기 *전*의 값이어야 한다. 해제 시
			// TipMeshAuthoredRelative로 원상 복구하므로(Teardown), 재획득이 오염된 값을 다시 캡처하지 않는다.
			TipMeshAuthoredScale = TipMeshComponent ? TipMeshComponent->GetComponentScale() : FVector::OneVector;
			TipMeshAuthoredRelative = TipMeshComponent ? TipMeshComponent->GetRelativeTransform() : FTransform::Identity;
			// 저작 충돌 캡처 후 bTipMeshCollision 반영(기본 끔). Teardown에서 이 값으로 복원한다.
			TipMeshAuthoredCollision = TipMeshComponent ? TipMeshComponent->GetCollisionEnabled() : ECollisionEnabled::QueryAndPhysics;
			ApplyTipMeshCollision();
			// 태그 컴포넌트가 TipMesh 에셋보다 우선하고 외부 컴포넌트의 메시는 바꾸지 않는다(인스턴스 소유).
			// 프리셋 전환에서 "TipMesh가 적용 안 된다"로 보이는 침묵을 없애기 위해 알린다.
			if (TipMesh)
			{
				UE_LOG(LogDynamicRope, Log,
					TEXT("[%s] 팁: 태그('%s') 컴포넌트가 TipMesh 에셋('%s')보다 우선한다 — 프리셋/에셋이 팁 메시를 소유하려면 TipMeshComponentTag를 비울 것."),
					*GetName(), *TipMeshComponentTag.ToString(), *TipMesh->GetName());
			}
			return;
		}
	}

	// ② 스폰(에셋 있을 때만). Wielder의 PreviewComponent 생성 idiom과 동형.
	if (!TipMesh)
	{
		return;
	}

	const FName TipName = MakeUniqueObjectName(Owner, UStaticMeshComponent::StaticClass(), TEXT("RopeTipMesh"));
	UStaticMeshComponent* Spawned =
		NewObject<UStaticMeshComponent>(Owner, UStaticMeshComponent::StaticClass(), TipName);
	if (!Spawned)
	{
		return;
	}
	Owner->AddInstanceComponent(Spawned);
	Spawned->SetupAttachment(this);
	Spawned->SetStaticMesh(TipMesh);
	Spawned->RegisterComponent();

	TipMeshComponent = Spawned;
	bTipMeshSpawnedByUs = true;
	TipMeshAuthoredScale = FVector::OneVector;
	// 스폰분의 충돌은 bTipMeshCollision을 따른다(기본 끔 = NoCollision, 켬 = QueryAndPhysics).
	ApplyTipMeshCollision();
}

void URopeComponent::TeardownSpawnedTipMesh()
{
	// 스폰분만 파괴한다 — 외부(태그로 찾은) 컴포넌트는 소유가 아니므로 놔둔다.
	if (TipMeshComponent && bTipMeshSpawnedByUs)
	{
		TipMeshComponent->DestroyComponent();
	}
	else if (TipMeshComponent)
	{
		// 외부(태그) 컴포넌트 해제: 매 프레임 배치(SetWorldTransform)가 덮어쓴 트랜스폼을 저작 원본으로
		// 되돌린다. 안 돌리면 다음 획득(프리셋 전환)이 "저작값×직전 프리셋 스케일"을 새 기준선으로 캡처해
		// 스케일이 누적 오염된다. 월드가 아닌 *상대* 트랜스폼 복원 — 부모가 움직였어도 저작 자세가 유지된다.
		TipMeshComponent->SetRelativeTransform(TipMeshAuthoredRelative);
		// 저작 충돌 복원 — bTipMeshCollision=false로 우리가 껐을 수 있으므로(외부 컴포넌트 소유 존중).
		TipMeshComponent->SetCollisionEnabled(TipMeshAuthoredCollision);
	}
	TipMeshComponent = nullptr;
	bTipMeshSpawnedByUs = false;
	TipMeshAuthoredScale = FVector::OneVector;
	TipMeshAuthoredRelative = FTransform::Identity;
	TipMeshAuthoredCollision = ECollisionEnabled::QueryAndPhysics;
}

void URopeComponent::ApplyTipMeshCollision()
{
	if (!TipMeshComponent)
	{
		return;
	}
	// 기본은 충돌 끔 — 표시 전용 팁의 충돌 바디가 로프 충돌 질의/캐릭터·월드와 간섭하는 것을 막는다.
	// 켜면 우리가 스폰한 팁은 전체 충돌(QueryAndPhysics)을, 태그로 재사용한 외부 컴포넌트는 저작 충돌
	// (획득 시점 값)을 되살린다 — 외부 컴포넌트에 우리가 임의의 프로파일을 강제하지 않는다.
	const ECollisionEnabled::Type Target = bTipMeshCollision
		? (bTipMeshSpawnedByUs ? ECollisionEnabled::QueryAndPhysics : TipMeshAuthoredCollision.GetValue())
		: ECollisionEnabled::NoCollision;
	TipMeshComponent->SetCollisionEnabled(Target);
}

void URopeComponent::UpdateTipMeshTransform()
{
	// 위치 = 끝 노드(자유단), 회전 = 마지막 세그먼트 방향을 X축으로. 노드 위치와 정확히 일치하도록
	// 상대가 아닌 월드 트랜스폼으로 배치한다(this에 부착돼 있어도 컴포넌트 트랜스폼 영향을 받지 않게).
	if (!TipMeshComponent)
	{
		return;
	}

	// Loaded(장전) 상태에서는 창을 손 소켓에 든다(마지막 노드가 아니라 GetLoadedTipTransform — override 가능).
	// Free 게이트보다 앞: Loaded은 ③의 손 소켓 고정이라 bSyncTipMeshOnFree와 무관하게 항상 유효해야 한다.
	if (Phase == ERopePhase::Loaded)
	{
		TipMeshComponent->SetWorldTransform(MakeTipWorldTransform(GetLoadedTipTransform()));
		return;
	}

	// Free 확장점: bSyncTipMeshOnFree를 끄면 팁 배치를 게임 코드에 넘긴다(GetTipMeshComponent) —
	// 여기서 트랜스폼을 건드리지 않는다. Free 외 페이즈는 이 플래그와 무관하게 항상 추종한다.
	if (Phase == ERopePhase::Free && !bSyncTipMeshOnFree)
	{
		return;
	}

	// Pierce 임베드 활성 조건(Pierce 결착 + 소켓 옵트인 + 소켓 실존)은 ReadTipSocketLocal이 전부 판정한다.
	// 아니면 아래 세그먼트-추종 폴백.
	const bool bPierceSocket = HasTipSocket(TipSocketName);

	// 꽂힌 뒤(Wrapped): 얼린 bone-local 메쉬 자세를 본에서 복원 — 회전 완전 고정 + 대상 애니메이션 추종.
	if (bPierceSocket && Phase == ERopePhase::Wrapped && WrapController.State.Anchors.Num() > 0)
	{
		const FRopeSurfaceAnchor& Anchor = WrapController.State.Anchors[0];
		if (const USceneComponent* Mesh = WrapController.State.Mesh.Get()) // cross-actor 대상 파괴 방어.
		{
			const FTransform BoneXform = ResolveBindingWorld(Mesh, Anchor.Bone);
			const FTransform MeshWorld = Anchor.LocalMeshTransform * BoneXform;
			TipMeshComponent->SetWorldTransform(MakeTipWorldTransform(MeshWorld));
			return;
		}
	}

	// 던지는 중(조준 GuidedThrow): 비행 중반까진 세그먼트 pitch + 조준 yaw 자세(A), Alpha 0.7~1.0에서
	// 최종 임베드 자세(B)로 slerp/lerp. Alpha=1의 B는 커밋 프레임 Wrapped 자세와 일치하므로 착지 시 팝이 없다.
	if (bPierceSocket && Phase == ERopePhase::GuidedThrow &&
		GuidedThrowState.bActive && !GuidedThrowState.bFreeThrow && Sim.Num() >= 2)
	{
		const int32 LastNode = Sim.Num() - 1;
		const FRopePreparedThrowPreview& Prepared = GuidedThrowState.Prepared;
		FVector HitPoint = Prepared.LatchAnchor.StartWorldPosition; // 빌더가 꽂힘 지점으로 세팅.
		ResolvePreparedPierceHitPoint(Prepared, HitPoint);
		FVector PierceDir = Prepared.ThrowContext.FrameForward.GetSafeNormal();
		if (PierceDir.IsNearlyZero())
		{
			PierceDir = (HitPoint - Sim.Positions[0]).GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
		}

		FTransform EmbedWorld;
		FVector TailWorld;
		if (ComputePierceEmbed(HitPoint, PierceDir, EmbedWorld, TailWorld))
		{
			const FVector SegDir = (Sim.Positions[LastNode] - Sim.Positions[LastNode - 1])
				.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
			const FVector AimYawLockedSegDir = FRopeTipPlacement::MakeAimYawLockedDirection(
				SegDir, PierceDir, Prepared.ThrowContext.FrameUp);
			FTransform SegFollow;
			ComputeTipFollowTransform(Sim.Positions[LastNode], AimYawLockedSegDir, SegFollow);

			const float Alpha = FMath::Clamp(
				GuidedThrowState.Elapsed / FMath::Max(GuidedThrowState.Duration, 0.01f), 0.0f, 1.0f);
			const float T = FMath::SmoothStep(0.7f, 1.0f, Alpha);
			const FQuat Rot = FQuat::Slerp(SegFollow.GetRotation(), EmbedWorld.GetRotation(), T);
			const FVector Loc = FMath::Lerp(SegFollow.GetLocation(), EmbedWorld.GetLocation(), T);
			TipMeshComponent->SetWorldTransform(MakeTipWorldTransform(FTransform(Rot, Loc)));
			return;
		}
	}

	const int32 N = Sim.Num();
	if (N < 2)
	{
		return;
	}

	// 위치 = 끝 노드(자유단), 회전 = 마지막 세그먼트 방향을 X축으로(비-Pierce · 소켓 미설정 폴백).
	const FVector TipPos = Sim.Positions[N - 1];
	const FVector SegDir = (Sim.Positions[N - 1] - Sim.Positions[N - 2])
		.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
	FVector FollowDir = SegDir;
	if (Phase == ERopePhase::GuidedThrow && GuidedThrowState.bActive && GuidedThrowState.bFreeThrow)
	{
		const FRopePreparedThrowPreview& Prepared = GuidedThrowState.Prepared;
		const FVector EndpointWorld = Prepared.ResolveGuidePointWorld(N - 1);
		const FVector HandWorld = Sim.Positions.IsValidIndex(0) ? Sim.Positions[0] : Prepared.ResolveGuideOriginWorld();
		FVector FreeAimDir = (EndpointWorld - HandWorld).GetSafeNormal();
		if (FreeAimDir.IsNearlyZero())
		{
			FreeAimDir = Prepared.ThrowContext.FrameForward.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
		}
		FollowDir = FRopeTipPlacement::MakeAimYawLockedDirection(
			SegDir, FreeAimDir, Prepared.ThrowContext.FrameUp);
	}
	FTransform TipFollow;
	ComputeTipFollowTransform(TipPos, FollowDir, TipFollow);

	TipMeshComponent->SetWorldTransform(MakeTipWorldTransform(TipFollow));
}

#pragma endregion Tip_Mesh

#pragma region Pierce_Embedding

// ===== Pierce 임베드 헬퍼 ====================================================

bool URopeComponent::HasTipSocket(FName Socket) const
{
	return IsTipSocketPlacementActive() && !Socket.IsNone() && TipMeshComponent &&
		TipMeshComponent->DoesSocketExist(Socket);
}

bool URopeComponent::ReadTipSocketLocal(FName Socket, FTransform& OutLocal) const
{
	// 존재/활성 조건은 HasTipSocket 한 곳에서 판정한다. 소켓 보정을 끄면 호출부가 각자의 기존 폴백으로 떨어진다.
	if (!HasTipSocket(Socket))
	{
		return false;
	}
	// RTS_Component = 메쉬 원점 기준 소켓 로컬 트랜스폼. UStaticMeshComponent가 UStaticMesh 소켓을 조회한다.
	OutLocal = TipMeshComponent->GetSocketTransform(Socket, RTS_Component);
	return true;
}

FTransform URopeComponent::MakeTipPlacementTransform() const
{
	return FTransform(FQuat::Identity, FVector::ZeroVector, TipMeshAuthoredScale) * TipMeshRelativeTransform;
}

FTransform URopeComponent::MakeTipPlacementSocketLocal(const FTransform& SocketLocal) const
{
	return SocketLocal * MakeTipPlacementTransform();
}

FTransform URopeComponent::MakeTipWorldTransform(const FTransform& BaseWorld) const
{
	return MakeTipPlacementTransform() * BaseWorld;
}

void URopeComponent::ComputeTipFollowTransform(const FVector& RopeAttachWorld, const FVector& ForwardDir,
	FTransform& OutComponentWorld) const
{
	FTransform TailSocketLocal;
	const bool bHasTail = ReadTipSocketLocal(TipRopeSocketName, TailSocketLocal);
	FTransform TipSocketLocal;
	const bool bHasTip = ReadTipSocketLocal(TipSocketName, TipSocketLocal);
	const FTransform RopeSocketLocal = bHasTail
		? MakeTipPlacementSocketLocal(TailSocketLocal)
		: MakeTipPlacementTransform();
	const FTransform HeadSocketLocal = bHasTip
		? MakeTipPlacementSocketLocal(TipSocketLocal)
		: FTransform::Identity;
	FRopeTipPlacement::SolveSocketFollow(RopeAttachWorld, ForwardDir, RopeSocketLocal,
		/*bHasHeadSocket*/ bHasTail && bHasTip, HeadSocketLocal, OutComponentWorld);
}

FVector URopeComponent::ResolveTipRopeAttachWorld(const FTransform& ComponentWorld) const
{
	if (!TipMeshComponent)
	{
		return ComponentWorld.GetLocation();
	}

	FTransform TailSocketLocal;
	if (ReadTipSocketLocal(TipRopeSocketName, TailSocketLocal))
	{
		return (MakeTipPlacementSocketLocal(TailSocketLocal) * ComponentWorld).GetLocation();
	}
	return MakeTipWorldTransform(ComponentWorld).GetLocation();
}

bool URopeComponent::ResolvePreparedPierceHitPoint(const FRopePreparedThrowPreview& Prepared, FVector& OutHitPoint) const
{
	if (Prepared.ThrowContext.bHasAimGuideHit)
	{
		// Aim guide로 만든 Pierce prepared는 조준 레이가 선택한 hit를 Head 기준점으로 유지한다.
		// 대상 본 기준으로 복원해 조준 이후 대상이 움직여도 조준한 신체 지점을 따라간다.
		OutHitPoint = ResolveAimGuideHitWorld(Prepared.ThrowContext);
		return true;
	}

	const FRopeSurfaceAnchor* Anchor = Prepared.Anchors.Num() > 0 ? &Prepared.Anchors[0] : &Prepared.LatchAnchor;
	if (!Anchor || Anchor->NodeIndex == INDEX_NONE)
	{
		return false;
	}

	const USceneComponent* Mesh = Anchor->Mesh.IsValid() ? Anchor->Mesh.Get() : Prepared.Mesh.Get();
	if (Mesh)
	{
		const FName Bone = Anchor->Bone.IsNone() ? Prepared.Bone : Anchor->Bone;
		if (!Bone.IsNone())
		{
			const FTransform BoneXform = ResolveBindingWorld(Mesh, Bone);
			OutHitPoint = BoneXform.TransformPosition(Anchor->LocalSurfacePosition);
			return true;
		}
	}

	OutHitPoint = Anchor->StartWorldPosition;
	return true;
}

void URopeComponent::ApplyPierceSocketTargetsToPrepared(FRopePreparedThrowPreview& InOutPrepared) const
{
	if (ResolveMode != ERopeWrapResolveMode::GuaranteedWrap || !InOutPrepared.RenderPreview.IsValid())
	{
		return;
	}

	const int32 LastPoint = InOutPrepared.RenderPreview.Points.Num() - 1;
	FVector HitPoint = InOutPrepared.RenderPreview.Points[LastPoint];
	ResolvePreparedPierceHitPoint(InOutPrepared, HitPoint);
	InOutPrepared.LatchAnchor.StartWorldPosition = HitPoint;
	if (InOutPrepared.Anchors.Num() > 0)
	{
		InOutPrepared.Anchors[0].StartWorldPosition = HitPoint;
	}

	FVector PierceDir = InOutPrepared.ThrowContext.FrameForward.GetSafeNormal();
	if (PierceDir.IsNearlyZero())
	{
		PierceDir = (HitPoint - InOutPrepared.ThrowContext.Origin)
			.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
	}

	FTransform ComponentWorld;
	FVector TailWorld;
	if (ComputePierceEmbed(HitPoint, PierceDir, ComponentWorld, TailWorld))
	{
		const FVector Origin = InOutPrepared.ThrowContext.Origin;
		for (int32 PointIndex = 0; PointIndex <= LastPoint; ++PointIndex)
		{
			const float Alpha = static_cast<float>(PointIndex) / static_cast<float>(LastPoint);
			InOutPrepared.RenderPreview.Points[PointIndex] = FMath::Lerp(Origin, TailWorld, Alpha);
		}
	}
}

bool URopeComponent::ComputePierceEmbed(const FVector& HitPoint, const FVector& PierceDir,
	FTransform& OutComponentWorld, FVector& OutTailWorld) const
{
	FTransform TipSocketLocal;
	if (!ReadTipSocketLocal(TipSocketName, TipSocketLocal))
	{
		return false; // 팁 소켓 필수 — 없으면 Pierce 임베드 비활성(호출부가 폴백).
	}
	FTransform TailSocketLocal;
	const bool bHasTail = ReadTipSocketLocal(TipRopeSocketName, TailSocketLocal);
	const FTransform EffectiveTipSocketLocal = MakeTipPlacementSocketLocal(TipSocketLocal);
	const FTransform EffectiveTailSocketLocal = bHasTail
		? MakeTipPlacementSocketLocal(TailSocketLocal)
		: MakeTipPlacementTransform();
	FRopeTipPlacement::SolvePierceEmbed(
		HitPoint, PierceDir, EffectiveTipSocketLocal, bHasTail, EffectiveTailSocketLocal,
		OutComponentWorld, OutTailWorld);
	if (!bHasTail)
	{
		OutTailWorld = MakeTipWorldTransform(OutComponentWorld).GetLocation();
	}
	return true;
}

#pragma endregion Pierce_Embedding
