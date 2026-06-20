// Copyright Epic Games, Inc. All Rights Reserved.
//
// PoC — experimental, not shipping. See Docs/PoC/01_PostWrapModel.md.

#include "PoC/RopePoCActor.h"
#include "PoC/RopePoCCapsuleActor.h"

#include "Components/SplineMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInterface.h"
#include "UObject/ConstructorHelpers.h"
#include "DrawDebugHelpers.h"
#include "EngineUtils.h"
#include "Engine/Engine.h"
#include "HAL/PlatformTime.h"

namespace
{
	// Base cross-section radius (cm) of /Engine/BasicShapes/Cylinder.
	constexpr float EngineCylinderBaseRadius = 50.0f;
	// Clamp the simulated timestep so large frame hitches can't explode the solver.
	constexpr float MaxSimDeltaSeconds = 1.0f / 30.0f;
}

ARopePoCActor::ARopePoCActor()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true;

	RopeRoot = CreateDefaultSubobject<USceneComponent>(TEXT("RopeRoot"));
	SetRootComponent(RopeRoot);
}

void ARopePoCActor::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	// Lay out a fresh straight rope whenever the actor is placed/moved/edited.
	bInitialized = false;
}

void ARopePoCActor::BeginPlay()
{
	Super::BeginPlay();
	bInitialized = false;
}

#if WITH_EDITOR
void ARopePoCActor::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	// Any property tweak rebuilds the rope on the next tick.
	bInitialized = false;
}
#endif

FVector ARopePoCActor::GetStartWorld() const
{
	return GetActorLocation();
}

FVector ARopePoCActor::GetEndWorld() const
{
	if (bPinEnd && EndAnchorActor)
	{
		return EndAnchorActor->GetActorLocation();
	}
	// Free end: lay the rope out along the actor's forward axis.
	return GetActorLocation() + GetActorForwardVector() * RopeLength;
}

void ARopePoCActor::InitializeRope()
{
	NumParticles = FMath::Max(2, NumParticles);
	SegmentLength = RopeLength / static_cast<float>(NumParticles - 1);

	const FVector StartW = GetStartWorld();
	const FVector EndW = GetEndWorld();

	Positions.SetNum(NumParticles);
	OldPositions.SetNum(NumParticles);
	InvMasses.SetNum(NumParticles);

	for (int32 i = 0; i < NumParticles; ++i)
	{
		const float Alpha = static_cast<float>(i) / static_cast<float>(NumParticles - 1);
		Positions[i] = FMath::Lerp(StartW, EndW, Alpha);
		OldPositions[i] = Positions[i];
		InvMasses[i] = 1.0f;
	}

	RebuildSegmentMeshes();
	GatherColliders();
	ApplyPinning();

	bInitialized = true;
}

void ARopePoCActor::GatherColliders()
{
	ActiveColliders.Reset();

	for (ARopePoCCapsuleActor* C : Colliders)
	{
		if (C)
		{
			ActiveColliders.AddUnique(C);
		}
	}

	if (bAutoFindColliders)
	{
		if (UWorld* World = GetWorld())
		{
			for (TActorIterator<ARopePoCCapsuleActor> It(World); It; ++It)
			{
				ActiveColliders.AddUnique(*It);
			}
		}
	}
}

void ARopePoCActor::RebuildSegmentMeshes()
{
	// Fall back to engine assets so the rope is visible with zero setup.
	if (!RopeMesh)
	{
		RopeMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	}
	if (!RopeMaterial)
	{
		RopeMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	}

	const int32 DesiredSegments = NumParticles - 1;
	const float RadiusScale = RopeRadius / EngineCylinderBaseRadius;

	// Reuse existing components when the segment count is unchanged (e.g. the actor is
	// just being dragged): only refresh the per-segment visual settings.
	bool bCountMatches = (SegmentMeshes.Num() == DesiredSegments);
	for (USplineMeshComponent* SM : SegmentMeshes)
	{
		bCountMatches &= (SM != nullptr);
	}
	if (bCountMatches)
	{
		for (USplineMeshComponent* SM : SegmentMeshes)
		{
			if (RopeMesh) { SM->SetStaticMesh(RopeMesh); }
			if (RopeMaterial) { SM->SetMaterial(0, RopeMaterial); }
			SM->SetStartScale(FVector2D(RadiusScale, RadiusScale), false);
			SM->SetEndScale(FVector2D(RadiusScale, RadiusScale), false);
		}
		return;
	}

	// Segment count changed: drop stale components and rebuild from scratch.
	for (USplineMeshComponent* SM : SegmentMeshes)
	{
		if (SM)
		{
			SM->DestroyComponent();
		}
	}
	SegmentMeshes.Reset();
	SegmentMeshes.SetNum(DesiredSegments);

	for (int32 i = 0; i < DesiredSegments; ++i)
	{
		USplineMeshComponent* SM = NewObject<USplineMeshComponent>(this, NAME_None, RF_Transient);
		SM->SetMobility(EComponentMobility::Movable);
		SM->SetupAttachment(RopeRoot);
		SM->RegisterComponent();

		SM->SetForwardAxis(ESplineMeshAxis::Z, /*bUpdateMesh=*/false);
		if (RopeMesh)
		{
			SM->SetStaticMesh(RopeMesh);
		}
		if (RopeMaterial)
		{
			SM->SetMaterial(0, RopeMaterial);
		}
		SM->SetStartScale(FVector2D(RadiusScale, RadiusScale), false);
		SM->SetEndScale(FVector2D(RadiusScale, RadiusScale), false);
		SM->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		SM->SetCastShadow(true);

		SegmentMeshes[i] = SM;
	}
}

void ARopePoCActor::ApplyPinning()
{
	if (Positions.Num() == 0)
	{
		return;
	}

	if (bPinStart)
	{
		Positions[0] = GetStartWorld();
		OldPositions[0] = Positions[0];
		InvMasses[0] = 0.0f;
	}
	else
	{
		InvMasses[0] = 1.0f;
	}

	const int32 Last = Positions.Num() - 1;
	if (bPinEnd && EndAnchorActor)
	{
		Positions[Last] = EndAnchorActor->GetActorLocation();
		OldPositions[Last] = Positions[Last];
		InvMasses[Last] = 0.0f;
	}
	else
	{
		InvMasses[Last] = 1.0f;
	}
}

void ARopePoCActor::SimulateStep(float DeltaSeconds)
{
	const float Dt = FMath::Min(DeltaSeconds, MaxSimDeltaSeconds);
	const float Dt2 = Dt * Dt;
	const float DampFactor = 1.0f - FMath::Clamp(Damping, 0.0f, 1.0f);

	// Verlet integration for free particles.
	for (int32 i = 0; i < Positions.Num(); ++i)
	{
		if (InvMasses[i] <= 0.0f)
		{
			continue;
		}
		const FVector Velocity = (Positions[i] - OldPositions[i]) * DampFactor;
		const FVector NewPos = Positions[i] + Velocity + Gravity * Dt2;
		OldPositions[i] = Positions[i];
		Positions[i] = NewPos;
	}

	// Re-pin endpoints, then satisfy distance + collision constraints.
	ApplyPinning();
	for (int32 Iter = 0; Iter < SolverIterations; ++Iter)
	{
		SolveConstraints();
		SolveCollisions();
	}
}

void ARopePoCActor::SolveConstraints()
{
	for (int32 i = 0; i < Positions.Num() - 1; ++i)
	{
		FVector& A = Positions[i];
		FVector& B = Positions[i + 1];
		const float WA = InvMasses[i];
		const float WB = InvMasses[i + 1];
		const float WSum = WA + WB;
		if (WSum <= 0.0f)
		{
			continue; // both endpoints pinned
		}

		const FVector Delta = B - A;
		const float Dist = Delta.Size();
		if (Dist <= KINDA_SMALL_NUMBER)
		{
			continue;
		}

		const float Diff = (Dist - SegmentLength) / Dist;
		const FVector Correction = Delta * Diff;
		A += Correction * (WA / WSum);
		B -= Correction * (WB / WSum);
	}
}

void ARopePoCActor::SolveCollisions()
{
	if (ActiveColliders.Num() == 0)
	{
		return;
	}

	for (const TWeakObjectPtr<ARopePoCCapsuleActor>& WeakC : ActiveColliders)
	{
		const ARopePoCCapsuleActor* C = WeakC.Get();
		if (!C)
		{
			continue;
		}

		FVector SegA, SegB;
		float CapsuleRadius;
		C->GetCapsuleSegment(SegA, SegB, CapsuleRadius);
		const float MinDist = CapsuleRadius + RopeCollisionRadius;

		for (int32 i = 0; i < Positions.Num(); ++i)
		{
			if (InvMasses[i] <= 0.0f)
			{
				continue; // don't push pinned particles
			}

			const FVector Closest = FMath::ClosestPointOnSegment(Positions[i], SegA, SegB);
			FVector ToParticle = Positions[i] - Closest;
			const float Dist = ToParticle.Size();
			if (Dist >= MinDist)
			{
				continue;
			}

			// Project the particle out to the capsule surface.
			FVector Normal;
			if (Dist > KINDA_SMALL_NUMBER)
			{
				Normal = ToParticle / Dist;
			}
			else
			{
				// Degenerate: particle on the axis — pick an arbitrary perpendicular.
				const FVector Axis = (SegB - SegA).GetSafeNormal(1e-4f, FVector::UpVector);
				Normal = FVector::CrossProduct(Axis, FVector::ForwardVector).GetSafeNormal(1e-4f, FVector::RightVector);
			}
			Positions[i] = Closest + Normal * MinDist;
		}
	}
}

void ARopePoCActor::UpdateSegmentMeshes()
{
	if (SegmentMeshes.Num() != Positions.Num() - 1)
	{
		return;
	}

	const FTransform ActorXf = GetActorTransform();

	// Per-particle tangents (Catmull-style), in local space, for smooth segments.
	const int32 N = Positions.Num();
	TArray<FVector, TInlineAllocator<64>> Local;
	Local.SetNum(N);
	for (int32 i = 0; i < N; ++i)
	{
		Local[i] = ActorXf.InverseTransformPosition(Positions[i]);
	}

	for (int32 i = 0; i < SegmentMeshes.Num(); ++i)
	{
		USplineMeshComponent* SM = SegmentMeshes[i];
		if (!SM)
		{
			continue;
		}

		const FVector StartPos = Local[i];
		const FVector EndPos = Local[i + 1];

		const FVector PrevA = Local[FMath::Max(i - 1, 0)];
		const FVector NextA = Local[FMath::Min(i + 1, N - 1)];
		const FVector PrevB = Local[FMath::Max(i, 0)];
		const FVector NextB = Local[FMath::Min(i + 2, N - 1)];

		FVector StartTangent = (NextA - PrevA) * 0.5f;
		FVector EndTangent = (NextB - PrevB) * 0.5f;
		if (StartTangent.IsNearlyZero()) { StartTangent = EndPos - StartPos; }
		if (EndTangent.IsNearlyZero()) { EndTangent = EndPos - StartPos; }

		SM->SetStartAndEnd(StartPos, StartTangent, EndPos, EndTangent, /*bUpdateMesh=*/true);
	}
}

void ARopePoCActor::DrawDebugRope() const
{
	const UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	for (int32 i = 0; i < Positions.Num(); ++i)
	{
		const FColor PointColor = (InvMasses[i] <= 0.0f) ? FColor::Red : FColor::Yellow;
		DrawDebugPoint(World, Positions[i], 6.0f, PointColor, false, -1.0f, SDPG_World);
		if (i < Positions.Num() - 1)
		{
			DrawDebugLine(World, Positions[i], Positions[i + 1], FColor::Cyan, false, -1.0f, SDPG_World, 0.5f);
		}
	}
}

void ARopePoCActor::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (!bInitialized)
	{
		InitializeRope();
	}

	const double SolveStart = FPlatformTime::Seconds();
	SimulateStep(DeltaSeconds);
	const double SolveEnd = FPlatformTime::Seconds();

	LastSolveMs = static_cast<float>((SolveEnd - SolveStart) * 1000.0);
	AvgSolveMs = (AvgSolveMs <= 0.0f) ? LastSolveMs : FMath::Lerp(AvgSolveMs, LastSolveMs, 0.05f);

	UpdateSegmentMeshes();

	if (bDrawDebug)
	{
		DrawDebugRope();

		if (GEngine)
		{
			const FColor BudgetColor = (AvgSolveMs < 0.3f) ? FColor::Green : FColor::Orange;
			GEngine->AddOnScreenDebugMessage(
				reinterpret_cast<uint64>(this), 0.0f, BudgetColor,
				FString::Printf(TEXT("[Rope] solve %.3f ms (avg) | particles %d | iters %d | colliders %d"),
					AvgSolveMs, NumParticles, SolverIterations, ActiveColliders.Num()));
		}
	}
}
