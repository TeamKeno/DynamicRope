// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeComponent.h"
#include "Collision/RopeCollider.h"
#include "Collision/RopeColliderProvider.h"

URopeComponent::URopeComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
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

	// Pin the start to the component (hand/socket).
	Sim.InvMass[0] = 0.0f;
}

void URopeComponent::GatherFrameColliders(TArray<IRopeCollider*>& OutColliders) const
{
	OutColliders.Reset();
	FBox RopeBounds(ForceInit);
	for (const FVector& P : Sim.Positions)
	{
		RopeBounds += P;
	}
	for (const TScriptInterface<IRopeColliderProvider>& Provider : ColliderProviders)
	{
		if (IRopeColliderProvider* Raw = Provider.GetInterface())
		{
			Raw->GatherColliders(RopeBounds, OutColliders);
		}
	}
}

void URopeComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (Sim.Num() == 0)
	{
		InitRope();
	}

	// Keep the pinned start following the component.
	if (Sim.Num() > 0 && Sim.InvMass[0] <= 0.0f)
	{
		Sim.Positions[0] = GetComponentLocation();
		Sim.PrevPositions[0] = Sim.Positions[0];
	}

	switch (Phase)
	{
	case ERopePhase::Flight:
	case ERopePhase::Contacting:
	{
		TArray<IRopeCollider*> Colliders;
		GatherFrameColliders(Colliders);
		Solver.Step(Sim, SolverConfig, Colliders, DeltaTime);
		// TODO(M2): run contact decision → transition to Wrapped + WrapController.BeginWrap.
		break;
	}
	case ERopePhase::Wrapped:
		WrapController.Hold(Sim, nullptr); // TODO(M3): pass the target skeletal mesh.
		break;
	case ERopePhase::Releasing:
		// TODO(M3): ease constraints out, then -> Free.
		Phase = ERopePhase::Free;
		break;
	default:
		break;
	}

	MarkRenderStateDirty(); // refresh bounds; real geometry update lands with the scene proxy.
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

void URopeComponent::ReleaseWrap()
{
	const FName Bone = WrapController.State.BoneName;
	WrapController.Release(ERopeReleaseReason::Manual);
	Phase = ERopePhase::Releasing;
	OnRopeReleased.Broadcast(Bone, ERopeReleaseReason::Manual);
}

FPrimitiveSceneProxy* URopeComponent::CreateSceneProxy()
{
	// TODO(M1): FRopeSceneProxy — tube mesh from the centerline (parallel-transport frames).
	return nullptr;
}

int32 URopeComponent::GetNumMaterials() const
{
	return 1;
}

FBoxSphereBounds URopeComponent::CalcBounds(const FTransform& LocalToWorld) const
{
	if (Sim.Num() == 0)
	{
		return FBoxSphereBounds(LocalToWorld.GetLocation(), FVector(RopeLength), RopeLength);
	}

	FBox Box(ForceInit);
	for (const FVector& P : Sim.Positions)
	{
		Box += P;
	}
	return FBoxSphereBounds(Box.ExpandBy(RopeLength * 0.05f));
}
