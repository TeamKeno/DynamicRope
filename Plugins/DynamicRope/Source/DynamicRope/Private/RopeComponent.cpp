// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeComponent.h"
#include "Collision/RopeCollider.h"
#include "Collision/RopeColliderProvider.h"
#include "Render/RopeSceneProxy.h"
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

	// Advance the pinned-start target; the solver sweeps Prev->Target across substeps so a fast
	// character move doesn't yank (and explode) the chain.
	if (Sim.bStartPinned)
	{
		Sim.StartPinPrev = Sim.StartPinTarget;
		Sim.StartPinTarget = GetComponentLocation();
	}

	switch (Phase)
	{
	case ERopePhase::Free:        // dangles from the hand and follows the character
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
	// Geometry is component-local; build a local box from the world sim points, then transform.
	if (Sim.Num() == 0)
	{
		return FBoxSphereBounds(LocalToWorld.GetLocation(), FVector(RopeLength), RopeLength);
	}

	FBox LocalBox(ForceInit);
	for (const FVector& P : Sim.Positions)
	{
		LocalBox += LocalToWorld.InverseTransformPosition(P);
	}
	return FBoxSphereBounds(LocalBox.ExpandBy(Radius + 1.0f)).TransformBy(LocalToWorld);
}
