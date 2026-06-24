// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeComponent.h"
#include "Collision/RopeCollider.h"
#include "Collision/RopeColliderProvider.h"
#include "Render/RopeSceneProxy.h"
#include "Components/SkeletalMeshComponent.h"
#include "GameFramework/Actor.h"
#include "Engine/Engine.h"
#include "DrawDebugHelpers.h"

URopeComponent::URopeComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;

	// Movable so the primitive outputs motion vectors (TAA/TSR keeps the moving rope).
	Mobility = EComponentMobility::Movable;
}

void URopeComponent::InitRope()
{
	const int32 N = FMath::Max(2, NumParticles);
	Sim.Positions.SetNum(N);
	Sim.PrevPositions.SetNum(N);
	Sim.InvMass.SetNum(N);
	Sim.RopeLength = RopeLength;
	Sim.SegmentLength = RopeLength / static_cast<float>(N - 1);

	const FVector Start = GetComponentLocation();
	const FVector End = Start + GetForwardVector() * RopeLength;
	for (int32 i = 0; i < N; ++i)
	{
		const float Alpha = static_cast<float>(i) / static_cast<float>(N - 1);
		Sim.Positions[i] = FMath::Lerp(Start, End, Alpha);
		Sim.PrevPositions[i] = Sim.Positions[i];
		Sim.InvMass[i] = 1.0f;
	}

	// Pin the start to the component (hand/socket); the solver sweeps it across substeps.
	Sim.InvMass[0] = 0.0f;
	Sim.bStartPinned = true;
	Sim.StartPinTarget = Start;
	Sim.StartPinPrev = Start;
}

void URopeComponent::GatherFrameColliders(TArray<IRopeCollider*>& OutColliders) const
{
	OutColliders.Reset();
	FBox RopeBounds(ForceInit);
	for (const FVector& P : Sim.Positions)
	{
		RopeBounds += P;
	}
	// Expand by the contact reach: a tight box around a near-straight rope is ~zero-thickness and
	// would wrongly cull capsules that are actually within contact distance. Match the narrow phase
	// (node ContactRadius; the capsule's own radius is already in its GetWorldBounds()).
	if (RopeBounds.IsValid)
	{
		RopeBounds = RopeBounds.ExpandBy(Radius + WrapConfig.ContactRadius + 5.0f);
	}

#if !UE_BUILD_SHIPPING
	if (bDrawDebugCenterline && RopeBounds.IsValid)
	{
		DrawDebugBox(GetWorld(), RopeBounds.GetCenter(), RopeBounds.GetExtent(), FColor::Orange, false, -1.0f, 0, 0.5f);
	}
#endif

	for (const TScriptInterface<IRopeColliderProvider>& Provider : ColliderProviders)
	{
		if (IRopeColliderProvider* Raw = Provider.GetInterface())
		{
			Raw->GatherColliders(RopeBounds, OutColliders);
		}
	}

#if !UE_BUILD_SHIPPING
	if (bDrawDebugCenterline && GEngine)
	{
		GEngine->AddOnScreenDebugMessage(reinterpret_cast<uint64>(this), 0.0f, FColor::Yellow,
			FString::Printf(TEXT("[Rope] phase=%d providers=%d colliders=%d wrapBone=%s"),
				static_cast<int32>(Phase), ColliderProviders.Num(), OutColliders.Num(),
				*WrapController.State.BoneName.ToString()));
	}
#endif
}

void URopeComponent::EnsureColliderProviders()
{
	if (ColliderProviders.Num() > 0)
	{
		return;
	}

	auto AddFrom = [this](AActor* Actor)
	{
		if (!Actor)
		{
			return;
		}
		for (UActorComponent* Comp : Actor->GetComponentsByInterface(URopeColliderProvider::StaticClass()))
		{
			ColliderProviders.AddUnique(TScriptInterface<IRopeColliderProvider>(Comp));
		}
	};

	// Cross-actor: when the rope is anchored to one actor but should catch a *different* body, the
	// provider lives on that other actor. Use the explicit list when set; otherwise default to our
	// own owner (same-actor case).
	if (ColliderSourceActors.Num() > 0)
	{
		for (AActor* Actor : ColliderSourceActors)
		{
			AddFrom(Actor);
		}
	}
	else
	{
		AddFrom(GetOwner());
	}

	// Only follow an explicitly-assigned wrap-target mesh's owner; never auto-resolve here (that
	// would re-add our own owner and let the rope latch onto itself).
	if (WrapTargetMesh)
	{
		AddFrom(WrapTargetMesh->GetOwner());
	}
}

USkeletalMeshComponent* URopeComponent::ResolveWrapTargetMesh()
{
	if (!WrapTargetMesh)
	{
		if (AActor* Owner = GetOwner())
		{
			WrapTargetMesh = Owner->FindComponentByClass<USkeletalMeshComponent>();
		}
	}
	return WrapTargetMesh;
}

void URopeComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (Sim.Num() == 0)
	{
		InitRope();
	}

	// Advance the pinned-start target; the solver sweeps Prev->Target across substeps so a fast
	// character move doesn't yank (and explode) the chain.
	if (Sim.bStartPinned)
	{
		Sim.StartPinPrev = Sim.StartPinTarget;
		Sim.StartPinTarget = GetComponentLocation();
	}

	EnsureColliderProviders();

	switch (Phase)
	{
	case ERopePhase::Free:        // dangles from the hand and follows the character
	case ERopePhase::Flight:
	case ERopePhase::Contacting:
	{
		TArray<IRopeCollider*> Colliders;
		GatherFrameColliders(Colliders);
		Solver.Step(Sim, SolverConfig, Colliders, DeltaTime);

		// Contact decision (physics → logic gate). Only after a throw; a freely dangling rope
		// brushing the body shouldn't latch.
		if (Phase != ERopePhase::Free)
		{
			FRopeWrapState Seed;
			if (WrapController.DecideWrap(Sim, Colliders, WrapConfig, DeltaTime, Seed))
			{
				WrapController.BeginWrap(Sim, Seed, ResolveWrapTargetMesh());
				Phase = ERopePhase::Wrapped;
				OnRopeWrapped.Broadcast(WrapController.State.BoneName);
			}
		}
		break;
	}
	case ERopePhase::Wrapped:
	{
		// Logic owns the latched nodes (ride the skinned bone); the solver still settles the free
		// span so the rope drapes and stays attached to the hand at node 0.
		WrapController.Hold(Sim, ResolveWrapTargetMesh(), DeltaTime);
		TArray<IRopeCollider*> Colliders;
		GatherFrameColliders(Colliders);
		Solver.Step(Sim, SolverConfig, Colliders, DeltaTime);
		break;
	}
	case ERopePhase::Releasing:
		// Hand every node back to the solver (keep only the hand pin), then resume free simulation.
		for (int32 i = 0; i < Sim.Num(); ++i)
		{
			Sim.InvMass[i] = (i == 0 && Sim.bStartPinned) ? 0.0f : 1.0f;
		}
		Phase = ERopePhase::Free;
		break;
	default:
		break;
	}

	// Push the new centerline to the render proxy and refresh bounds.
	MarkRenderDynamicDataDirty();
	MarkRenderTransformDirty();

	if (bDrawDebugCenterline)
	{
		if (const UWorld* World = GetWorld())
		{
			for (int32 i = 0; i < Sim.Num(); ++i)
			{
				DrawDebugPoint(World, Sim.Positions[i], 6.0f, FColor::Yellow, false, -1.0f, SDPG_Foreground);
				if (i + 1 < Sim.Num())
				{
					DrawDebugLine(World, Sim.Positions[i], Sim.Positions[i + 1], FColor::Cyan, false, -1.0f, SDPG_Foreground, 0.5f);
				}
			}
		}
	}
}

void URopeComponent::SendRenderDynamicData_Concurrent()
{
	Super::SendRenderDynamicData_Concurrent();

	if (!SceneProxy || Sim.Num() < 2)
	{
		return;
	}

	// Send the centerline in component-local space; the proxy renders via GetLocalToWorld().
	const FTransform Xform = GetComponentTransform();
	FRopeDynamicData* DynamicData = new FRopeDynamicData;
	DynamicData->Points.SetNumUninitialized(Sim.Num());
	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		DynamicData->Points[i] = Xform.InverseTransformPosition(Sim.Positions[i]);
	}

	FRopeSceneProxy* Proxy = static_cast<FRopeSceneProxy*>(SceneProxy);
	ENQUEUE_RENDER_COMMAND(RopeUpdateCenterline)(
		[Proxy, DynamicData](FRHICommandListBase& RHICmdList)
		{
			Proxy->SetDynamicData_RenderThread(RHICmdList, DynamicData);
		});
}

void URopeComponent::Throw(const FVector& AimDir)
{
	if (Sim.Num() == 0)
	{
		InitRope();
	}

	Phase = ERopePhase::Flight;

	// Scaffold launch: give the free tip an initial velocity via Verlet prev-position offset.
	const int32 Last = Sim.Num() - 1;
	if (Last > 0)
	{
		Sim.InvMass[Last] = 1.0f;
		const FVector Velocity = AimDir.GetSafeNormal() * ThrowParams.ThrowSpeed * (1.0f / 60.0f);
		Sim.PrevPositions[Last] = Sim.Positions[Last] - Velocity;
	}
}

bool URopeComponent::DebugForceWrap()
{
	if (Sim.Num() == 0)
	{
		InitRope();
	}

	EnsureColliderProviders();
	TArray<IRopeCollider*> Colliders;
	GatherFrameColliders(Colliders);

	// Relax the decision gate so a single touching node commits this frame (DecideWrap commits when
	// the candidate's accumulated time >= WrapDecisionTime; 0 means "right now").
	FRopeWrapConfig Relaxed = WrapConfig;
	Relaxed.WrapDecisionTime = 0.0f;
	Relaxed.MinLatchNodes = 1;

	FRopeWrapState Seed;
	if (!WrapController.DecideWrap(Sim, Colliders, Relaxed, 0.0f, Seed))
	{
		return false; // nothing in contact; move the rope/capsules so they overlap first
	}

	WrapController.BeginWrap(Sim, Seed, ResolveWrapTargetMesh());
	Phase = ERopePhase::Wrapped;
	OnRopeWrapped.Broadcast(WrapController.State.BoneName);
	return true;
}

void URopeComponent::ReleaseWrap()
{
	const FName Bone = WrapController.State.BoneName;
	WrapController.Release(ERopeReleaseReason::Manual);
	Phase = ERopePhase::Releasing;
	OnRopeReleased.Broadcast(Bone, ERopeReleaseReason::Manual);
}

FPrimitiveSceneProxy* URopeComponent::CreateSceneProxy()
{
	return new FRopeSceneProxy(this);
}

int32 URopeComponent::GetNumMaterials() const
{
	return 1;
}

UMaterialInterface* URopeComponent::GetMaterial(int32 /*ElementIndex*/) const
{
	return RopeMaterial;
}

void URopeComponent::SetMaterial(int32 /*ElementIndex*/, UMaterialInterface* Material)
{
	RopeMaterial = Material;
	MarkRenderStateDirty();
}

FBoxSphereBounds URopeComponent::CalcBounds(const FTransform& LocalToWorld) const
{
	// Anchor the bounds to the component (the pinned start) with a radius that always contains the
	// rope no matter how it deforms: the chain is inextensible, so no particle is ever farther than
	// RopeLength from the pin (+ tube radius). Deriving bounds from the per-frame sim points instead
	// makes them lag the render thread by a frame; during fast character motion the rope outruns that
	// tight box and gets culled from the shadow/main pass -> the shadow vanishes while moving and the
	// VSM cache keeps a stale afterimage. Anchoring to the component transform moves the bounds with
	// the character via the engine's tracked transform, so there is no lag and no spurious culling.
	const float Reach = RopeLength + Radius + 1.0f;
	return FBoxSphereBounds(LocalToWorld.GetLocation(), FVector(Reach), Reach);
}
