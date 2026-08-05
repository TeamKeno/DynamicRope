// Copyright 2026 TeamKeno. All Rights Reserved.

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
	// The default implementation: the transform of the LoadedHandSocket on the owner's skeletal mesh,
	// falling back to the component transform, that is the hand, when there is none.
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

#pragma region Tip_Loaded_Placement

FTransform URopeComponent::MakeLoadedTipBaseWorld() const
{
	// The authored offset is expressed in the hand socket's frame, so it composes *before* the socket
	// transform. Applying it on top of the virtual keeps overridden placements offset-aware for free.
	return LoadedTipRelativeTransform * GetLoadedTipTransform();
}

#pragma endregion Tip_Loaded_Placement

#pragma region Tip_Mesh

// ===== The tip attachment, display only =====================================

void URopeComponent::EnsureTipMesh()
{
	// Called from BeginPlay, and also on a throw and on entering Loaded to cover enabling bUseTipMesh at
	// runtime; it is idempotent.
	// It first reuses a tagged component on the owner, which it never destroys, and otherwise spawns one
	// where a tip mesh asset is set, which it does own and destroy. With one already secured it is a
	// no-op. The attachment has no mass and no collision and is display only.
	// It is the only path that secures a tip, so blocking here disables the whole tip subsystem; everything
	// else simply null-guards the component.
	if (!bUseTipMesh || TipMeshComponent)
	{
		return;
	}

	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}

	// First, look for an external component by tag; where one exists it is reused and ownership is not
	// taken.
	if (!TipMeshComponentTag.IsNone())
	{
		TArray<UActorComponent*> Tagged =
			Owner->GetComponentsByTag(UStaticMeshComponent::StaticClass(), TipMeshComponentTag);
		if (Tagged.Num() > 0)
		{
			TipMeshComponent = Cast<UStaticMeshComponent>(Tagged[0]);
			bTipMeshSpawnedByUs = false;
			// Capture the authored baseline before placement overwrites the transform. Teardown restores it,
			// so reacquiring never captures an already-polluted value as the new baseline.
			TipMeshAuthoredScale = TipMeshComponent ? TipMeshComponent->GetComponentScale() : FVector::OneVector;
			TipMeshAuthoredRelative = TipMeshComponent ? TipMeshComponent->GetRelativeTransform() : FTransform::Identity;
			// Capture the authored collision, then apply bTipMeshCollision, which is off by default.
			// Teardown restores this value.
			TipMeshAuthoredCollision = TipMeshComponent ? TipMeshComponent->GetCollisionEnabled() : ECollisionEnabled::QueryAndPhysics;
			ApplyTipMeshCollision();
			// A tagged component takes priority over the tip mesh asset, and the external component's mesh
			// is never changed, since the instance owns it.
			// This is reported so that switching presets does not silently look like the tip mesh failing to
			// apply.
			if (TipMesh)
			{
				UE_LOG(LogDynamicRope, Log,
					TEXT("[%s] Tip: the tagged component ('%s') takes priority over the TipMesh asset ('%s'). Clear TipMeshComponentTag to let the preset or asset own the tip mesh."),
					*GetName(), *TipMeshComponentTag.ToString(), *TipMesh->GetName());
			}
			return;
		}
	}

	// Second, spawn one, only where an asset is set. It mirrors the idiom the wielder uses to create its
	// preview component.
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
	// A spawned tip's collision follows bTipMeshCollision, which is off by default and otherwise enables
	// full collision.
	ApplyTipMeshCollision();
}

void URopeComponent::TeardownSpawnedTipMesh()
{
	// Only a spawned tip is destroyed; an external component found by tag is not owned and is left alone.
	if (TipMeshComponent && bTipMeshSpawnedByUs)
	{
		TipMeshComponent->DestroyComponent();
	}
	else if (TipMeshComponent)
	{
		// Releasing an external tagged component: the transform that per-frame placement overwrote is
		// returned to the authored original. Without that, the next acquisition, as on a preset switch,
		// would capture the authored value multiplied by the previous preset's scale as its new baseline and
		// the scale would compound. It restores the relative rather than the world transform, so the
		// authored pose survives even if the parent moved.
		TipMeshComponent->SetRelativeTransform(TipMeshAuthoredRelative);
		// Restore the authored collision, since bTipMeshCollision may have made us disable it, out of
		// respect for the external component's ownership.
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
	// Collision is off by default, which stops a display-only tip's collision body interfering with rope
	// collision queries, the character and the world.
	// Enabling it gives a tip we spawned full collision, while an external component reused by tag has its
	// authored collision, as captured on acquisition, restored instead; no arbitrary profile is forced onto
	// an external component.
	const ECollisionEnabled::Type Target = bTipMeshCollision
		? (bTipMeshSpawnedByUs ? ECollisionEnabled::QueryAndPhysics : TipMeshAuthoredCollision.GetValue())
		: ECollisionEnabled::NoCollision;
	TipMeshComponent->SetCollisionEnabled(Target);
}

void URopeComponent::UpdateTipMeshTransform()
{
	// The position is the end node, that is the free end, and the rotation puts the last segment's
	// direction on the X axis. It is placed with a world rather than a relative transform so it matches the
	// node position exactly and is unaffected by the component transform even though it is attached here.
	if (!TipMeshComponent)
	{
		return;
	}

	// While Loaded the tip is held at the hand socket, from GetLoadedTipTransform, which can be overridden,
	// rather than at the last node.
	// It comes before the Free gate because Loaded pins it to the hand socket for GuaranteedWrap and has to
	// apply regardless of bSyncTipMeshOnFree.
	if (Phase == ERopePhase::Loaded)
	{
		TipMeshComponent->SetWorldTransform(MakeTipWorldTransform(MakeLoadedTipBaseWorld()));
		return;
	}

	// The Free extension point: turning off bSyncTipMeshOnFree hands tip placement to game code, through
	// GetTipMeshComponent, and the transform is left untouched here. Every phase other than Free follows
	// the rope regardless of that flag.
	if (Phase == ERopePhase::Free && !bSyncTipMeshOnFree)
	{
		return;
	}

	// ReadTipSocketLocal decides every condition for the pierce embed being active, namely a pierce
	// binding, the socket opt-in and the socket actually existing. Otherwise it falls through to the
	// segment-following fallback below.
	const bool bPierceSocket = HasTipSocket(TipSocketName);

	// Once embedded, while wrapped: the frozen bone-local mesh pose is restored from the bone, which fixes
	// the rotation completely while following the target's animation.
	if (bPierceSocket && Phase == ERopePhase::Wrapped && WrapController.State.Anchors.Num() > 0)
	{
		const FRopeSurfaceAnchor& Anchor = WrapController.State.Anchors[0];
		if (const USceneComponent* Mesh = WrapController.State.Mesh.Get()) // Guards against a destroyed cross-actor target.
		{
			const FTransform BoneXform = ResolveBindingWorld(Mesh, Anchor.Bone);
			const FTransform MeshWorld = Anchor.LocalMeshTransform * BoneXform;
			TipMeshComponent->SetWorldTransform(MakeTipWorldTransform(MeshWorld));
			return;
		}
	}

	// Mid-throw, during an aimed guided throw: for most of the flight it uses the segment pitch with the
	// aimed yaw, and over the last part of the flight it interpolates to the final embedded pose. At the end
	// that pose matches the wrapped pose of the commit frame, so there is no pop on landing.
	if (bPierceSocket && Phase == ERopePhase::GuidedThrow &&
		GuidedThrowState.bActive && !GuidedThrowState.bFreeThrow && Sim.Num() >= 2)
	{
		const int32 LastNode = Sim.Num() - 1;
		const FRopePreparedThrowPreview& Prepared = GuidedThrowState.Prepared;
		FVector HitPoint = Prepared.LatchAnchor.StartWorldPosition; // Set to the embed point by the builder.
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

	// The position is the end node, that is the free end, and the rotation puts the last segment's
	// direction on the X axis. This is the fallback for a non-pierce binding or an unconfigured socket.
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

// ===== Pierce embed helpers ==================================================

bool URopeComponent::HasTipSocket(FName Socket) const
{
	return IsTipSocketPlacementActive() && !Socket.IsNone() && TipMeshComponent &&
		TipMeshComponent->DoesSocketExist(Socket);
}

bool URopeComponent::ReadTipSocketLocal(FName Socket, FTransform& OutLocal) const
{
	// Whether it exists and is active is decided in the single place HasTipSocket. Turning off the socket
	// correction lets each call site fall through to its own existing fallback.
	if (!HasTipSocket(Socket))
	{
		return false;
	}
	// Component space gives the socket's local transform relative to the mesh origin; the static mesh
	// component looks the socket up on the static mesh.
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
		// A pierce prepared throw built from an aim guide keeps the hit the aiming ray selected as the head
		// reference point.
		// Restoring it against the target bone makes it follow the body point that was aimed at even after
		// the target moves.
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
		return false; // A tip socket is required; without one the pierce embed is inactive and the call site falls back.
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
