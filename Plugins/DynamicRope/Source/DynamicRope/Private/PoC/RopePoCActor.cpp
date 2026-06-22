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

	// Wrap-latch state starts empty (no node latched yet).
	LatchCapsule.Init(-1, NumParticles);
	LatchAlong.Init(0.0f, NumParticles);
	LatchRadialDir.Init(FVector::UpVector, NumParticles);
	LatchRadialDist.Init(0.0f, NumParticles);
	LatchAxis.Init(FVector::UpVector, NumParticles);
	ContactDwell.Init(0.0f, NumParticles);
	bHasPrevPins = false;

	RebuildSegmentMeshes();
	GatherProviders();
	ApplyPinning();

	bInitialized = true;
}

void ARopePoCActor::GatherProviders()
{
	CapsuleProviders.Reset();

	// Explicit providers (test capsule actors).
	for (ARopePoCCapsuleActor* C : Colliders)
	{
		if (C)
		{
			CapsuleProviders.AddUnique(C);
		}
	}

	if (!bAutoFindColliders)
	{
		return;
	}

	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	// Any actor or component in the level implementing IRopeCapsuleProvider.
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		AActor* Actor = *It;
		if (!Actor)
		{
			continue;
		}

		if (Actor->Implements<URopeCapsuleProvider>())
		{
			CapsuleProviders.AddUnique(Actor);
		}

		TArray<UActorComponent*> Components;
		Actor->GetComponents(Components);
		for (UActorComponent* Comp : Components)
		{
			if (Comp && Comp->Implements<URopeCapsuleProvider>())
			{
				CapsuleProviders.AddUnique(Comp);
			}
		}
	}
}

void ARopePoCActor::BuildFrameCapsules()
{
	FrameCapsules.Reset();
	FrameCapsuleOwner.Reset();

	for (int32 ProviderIdx = 0; ProviderIdx < CapsuleProviders.Num(); ++ProviderIdx)
	{
		UObject* Obj = CapsuleProviders[ProviderIdx].Get();
		if (!Obj)
		{
			continue;
		}
		if (const IRopeCapsuleProvider* Provider = Cast<IRopeCapsuleProvider>(Obj))
		{
			const int32 Before = FrameCapsules.Num();
			Provider->GatherRopeCapsules(FrameCapsules);
			// Tag every capsule this provider just appended with its provider index.
			for (int32 c = Before; c < FrameCapsules.Num(); ++c)
			{
				FrameCapsuleOwner.Add(ProviderIdx);
			}
		}
	}

	// Reset this frame's pull-reaction accumulators to match the capsule set.
	CapsuleReaction.Init(FVector::ZeroVector, FrameCapsules.Num());
	CapsuleReactionPoint.Init(FVector::ZeroVector, FrameCapsules.Num());
	CapsuleReactionWeight.Init(0.0f, FrameCapsules.Num());
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
	SetPinnedTargets(GetStartWorld(), GetEndWorld());
}

void ARopePoCActor::SetPinnedTargets(const FVector& StartW, const FVector& EndW)
{
	if (Positions.Num() == 0)
	{
		return;
	}

	if (bPinStart)
	{
		Positions[0] = StartW;
		OldPositions[0] = StartW;
		InvMasses[0] = 0.0f;
	}
	else
	{
		InvMasses[0] = 1.0f;
	}

	const int32 Last = Positions.Num() - 1;
	if (bPinEnd && EndAnchorActor)
	{
		Positions[Last] = EndW;
		OldPositions[Last] = EndW;
		InvMasses[Last] = 0.0f;
	}
	else
	{
		InvMasses[Last] = 1.0f;
	}
}

void ARopePoCActor::SimulateStep(float DeltaSeconds)
{
	const float FrameDt = FMath::Min(DeltaSeconds, MaxSimDeltaSeconds);
	const int32 Sub = FMath::Clamp(SimSubsteps, 1, 16);
	const float SubDt = FrameDt / static_cast<float>(Sub);
	const float SubDt2 = SubDt * SubDt;
	const float DampFactor = 1.0f - FMath::Clamp(Damping, 0.0f, 1.0f);
	const int32 ItersPerSub = FMath::Max(1, SolverIterations / Sub);

	// This frame's capsules (current pose) + reset the per-frame reaction accumulators.
	BuildFrameCapsules();

	// Pinned-end targets: sweep from last frame's pose (Prev) to this frame's (Target).
	const FVector StartTarget = GetStartWorld();
	const FVector EndTarget = GetEndWorld();
	if (!bHasPrevPins)
	{
		PrevStartWorld = StartTarget;
		PrevEndWorld = EndTarget;
		bHasPrevPins = true;
	}

	const bool bCanInterpCapsules = (PrevFrameCapsules.Num() == FrameCapsules.Num());

	for (int32 s = 1; s <= Sub; ++s)
	{
		const float Alpha = static_cast<float>(s) / static_cast<float>(Sub);
		const float AlphaPrev = static_cast<float>(s - 1) / static_cast<float>(Sub);

		// Capsule poses for this substep, and the substep before (friction surface velocity).
		ActiveCapsules = FrameCapsules;
		PrevActiveCapsules = FrameCapsules;
		if (bCanInterpCapsules)
		{
			for (int32 c = 0; c < FrameCapsules.Num(); ++c)
			{
				const FRopeCapsule& P = PrevFrameCapsules[c];
				const FRopeCapsule& F = FrameCapsules[c];
				ActiveCapsules[c].A = FMath::Lerp(P.A, F.A, Alpha);
				ActiveCapsules[c].B = FMath::Lerp(P.B, F.B, Alpha);
				PrevActiveCapsules[c].A = FMath::Lerp(P.A, F.A, AlphaPrev);
				PrevActiveCapsules[c].B = FMath::Lerp(P.B, F.B, AlphaPrev);
			}
		}

		// Verlet integrate free particles by the (smaller) substep timestep.
		for (int32 i = 0; i < Positions.Num(); ++i)
		{
			if (InvMasses[i] <= 0.0f)
			{
				continue;
			}
			const FVector Velocity = (Positions[i] - OldPositions[i]) * DampFactor;
			const FVector NewPos = Positions[i] + Velocity + Gravity * SubDt2;
			OldPositions[i] = Positions[i];
			Positions[i] = NewPos;
		}

		// Sweep the pinned ends to their interpolated targets.
		SetPinnedTargets(FMath::Lerp(PrevStartWorld, StartTarget, Alpha),
		                 FMath::Lerp(PrevEndWorld, EndTarget, Alpha));

		// Hold: keep latched wrap nodes glued to the (moving) capsule surface this substep.
		UpdateLatchedPositions();

		for (int32 Iter = 0; Iter < ItersPerSub; ++Iter)
		{
			SolveConstraints();
			SolveCollisions();
		}

		// Friction is a per-substep velocity adjustment, after contacts are resolved.
		ApplyFriction();
	}

	// Latch newly-established wraps / break yanked-off ones, using the resolved frame state.
	ManageWrapLatch(FrameDt);
	// Latched nodes don't get push-out reaction (they're pinned) — feed their tension instead.
	AccumulateLatchReaction();

	// S4: reaction was accumulated across all substeps — hand it to the providers once.
	ApplyPullReaction();

	// Remember this frame's pin targets and capsule poses for next frame's sweep.
	PrevStartWorld = StartTarget;
	PrevEndWorld = EndTarget;
	PrevFrameCapsules = FrameCapsules;
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
	for (int32 c = 0; c < ActiveCapsules.Num(); ++c)
	{
		const FRopeCapsule& Cap = ActiveCapsules[c];
		const float MinDist = Cap.Radius + RopeCollisionRadius;

		for (int32 i = 0; i < Positions.Num(); ++i)
		{
			if (InvMasses[i] <= 0.0f)
			{
				continue; // don't push pinned particles
			}

			const FVector Closest = FMath::ClosestPointOnSegment(Positions[i], Cap.A, Cap.B);
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
				const FVector Axis = (Cap.B - Cap.A).GetSafeNormal(1e-4f, FVector::UpVector);
				Normal = FVector::CrossProduct(Axis, FVector::ForwardVector).GetSafeNormal(1e-4f, FVector::RightVector);
			}

			const FVector Target = Closest + Normal * MinDist;
			const FVector PushOut = Target - Positions[i]; // displacement applied to the particle
			Positions[i] = Target;

			// S4: the body pushed the particle out by +PushOut, so the rope pushes the
			// body by -PushOut (Newton's 3rd law). Accumulate it for ApplyPullReaction.
			if (bEnableTwoWayPull && CapsuleReaction.IsValidIndex(c))
			{
				CapsuleReaction[c] += -PushOut;
				CapsuleReactionPoint[c] += Closest;
				CapsuleReactionWeight[c] += 1.0f;
			}
		}
	}
}

void ARopePoCActor::ApplyFriction()
{
	if (WrapFriction <= 0.0f)
	{
		return;
	}

	// Need same-order previous-substep capsules to estimate how the surface moved.
	if (PrevActiveCapsules.Num() != ActiveCapsules.Num())
	{
		return;
	}

	for (int32 c = 0; c < ActiveCapsules.Num(); ++c)
	{
		const FRopeCapsule& Cur = ActiveCapsules[c];
		const FRopeCapsule& Prev = PrevActiveCapsules[c];
		const float ContactDist = Cur.Radius + RopeCollisionRadius + FrictionContactBand;

		const FVector Seg = Cur.B - Cur.A;
		const float SegLenSq = Seg.SizeSquared();

		for (int32 i = 0; i < Positions.Num(); ++i)
		{
			if (InvMasses[i] <= 0.0f)
			{
				continue;
			}

			const FVector Closest = FMath::ClosestPointOnSegment(Positions[i], Cur.A, Cur.B);
			const FVector ToParticle = Positions[i] - Closest;
			const float Dist = ToParticle.Size();
			if (Dist > ContactDist)
			{
				continue; // not in contact — no friction
			}

			const FVector Normal = (Dist > KINDA_SMALL_NUMBER) ? (ToParticle / Dist) : FVector::UpVector;

			// Sample the surface velocity at the contact point (same parameter on prev/cur segment).
			const float T = (SegLenSq > KINDA_SMALL_NUMBER)
				? FMath::Clamp(FVector::DotProduct(Positions[i] - Cur.A, Seg) / SegLenSq, 0.0f, 1.0f)
				: 0.0f;
			const FVector SurfaceVel = FMath::Lerp(Cur.A, Cur.B, T) - FMath::Lerp(Prev.A, Prev.B, T);

			// Relative tangential motion between the rope particle and the moving surface.
			const FVector ParticleVel = Positions[i] - OldPositions[i];
			const FVector RelVel = ParticleVel - SurfaceVel;
			const FVector RelTangent = RelVel - FVector::DotProduct(RelVel, Normal) * Normal;

			// Cancel a fraction of it. In Verlet, shifting OldPosition toward Position lowers velocity.
			OldPositions[i] += RelTangent * WrapFriction;
		}
	}
}

void ARopePoCActor::ApplyPullReaction()
{
	if (!bEnableTwoWayPull || PullReactionGain <= 0.0f)
	{
		return;
	}

	for (int32 c = 0; c < FrameCapsules.Num(); ++c)
	{
		const float Weight = CapsuleReactionWeight.IsValidIndex(c) ? CapsuleReactionWeight[c] : 0.0f;
		if (Weight <= 0.0f)
		{
			continue; // no contact this frame
		}

		// Resolve the provider that owns this capsule.
		if (!FrameCapsuleOwner.IsValidIndex(c))
		{
			continue;
		}
		const int32 ProviderIdx = FrameCapsuleOwner[c];
		if (!CapsuleProviders.IsValidIndex(ProviderIdx))
		{
			continue;
		}
		UObject* Obj = CapsuleProviders[ProviderIdx].Get();
		IRopeCapsuleProvider* Provider = Obj ? Cast<IRopeCapsuleProvider>(Obj) : nullptr;
		if (!Provider)
		{
			continue;
		}

		const FVector Impulse = CapsuleReaction[c] * PullReactionGain;
		const FVector AppPoint = CapsuleReactionPoint[c] / Weight; // averaged contact point
		Provider->ApplyRopeReaction(Impulse, AppPoint);

		if (bDrawDebug)
		{
			if (const UWorld* World = GetWorld())
			{
				DrawDebugDirectionalArrow(World, AppPoint, AppPoint + Impulse * 20.0f,
					12.0f, FColor::Magenta, false, -1.0f, SDPG_World, 1.5f);
			}
		}
	}
}

void ARopePoCActor::UpdateLatchedPositions()
{
	if (!bEnableWrapLatch)
	{
		return;
	}

	for (int32 i = 0; i < Positions.Num(); ++i)
	{
		const int32 c = LatchCapsule[i];
		if (c < 0)
		{
			continue;
		}
		if (!ActiveCapsules.IsValidIndex(c))
		{
			// Capsule set changed under us — drop the latch (interior nodes only get latched).
			LatchCapsule[i] = -1;
			InvMasses[i] = 1.0f;
			continue;
		}

		const FRopeCapsule& Cap = ActiveCapsules[c];
		const FVector AxisVec = Cap.B - Cap.A;
		const float AxisLen = AxisVec.Size();
		const FVector CurAxis = (AxisLen > KINDA_SMALL_NUMBER) ? (AxisVec / AxisLen) : LatchAxis[i];

		// Rotate the stored radial offset by however the capsule axis turned since last step,
		// so the wrap follows the limb as it swings (capsule is symmetric about its axis).
		const FQuat Turn = FQuat::FindBetweenNormals(LatchAxis[i], CurAxis);
		const FVector NewRadial = Turn.RotateVector(LatchRadialDir[i]).GetSafeNormal(1e-4f, CurAxis);
		LatchRadialDir[i] = NewRadial;
		LatchAxis[i] = CurAxis;

		// Surface point = point along the axis + radial offset.
		const float Along = FMath::Clamp(LatchAlong[i], 0.0f, AxisLen);
		const FVector Surface = Cap.A + CurAxis * Along + NewRadial * LatchRadialDist[i];

		Positions[i] = Surface;
		OldPositions[i] = Surface; // pinned: don't carry velocity while held
		InvMasses[i] = 0.0f;
	}
}

void ARopePoCActor::ManageWrapLatch(float FrameDt)
{
	if (!bEnableWrapLatch)
	{
		// Disabled: release every latch so the rope is fully dynamic again.
		for (int32 i = 0; i < Positions.Num(); ++i)
		{
			if (LatchCapsule[i] >= 0)
			{
				LatchCapsule[i] = -1;
				InvMasses[i] = 1.0f;
			}
			ContactDwell[i] = 0.0f;
		}
		return;
	}

	const float MaxLen = SegmentLength * LatchReleaseStrain;

	// Never latch the (user-controlled) endpoints.
	for (int32 i = 1; i < Positions.Num() - 1; ++i)
	{
		// Already latched → break it if a neighbouring segment is stretched past the limit.
		if (LatchCapsule[i] >= 0)
		{
			const bool bYanked =
				(Positions[i] - Positions[i - 1]).Size() > MaxLen ||
				(Positions[i + 1] - Positions[i]).Size() > MaxLen;
			if (bYanked)
			{
				LatchCapsule[i] = -1;
				InvMasses[i] = 1.0f;
				ContactDwell[i] = 0.0f;
			}
			continue;
		}

		// Not latched → find the closest capsule currently in contact.
		int32 BestCap = -1;
		float BestDist = TNumericLimits<float>::Max();
		FVector BestClosest = FVector::ZeroVector;
		FVector BestAxis = FVector::UpVector;
		float BestAxisLen = 0.0f;

		for (int32 c = 0; c < ActiveCapsules.Num(); ++c)
		{
			const FRopeCapsule& Cap = ActiveCapsules[c];
			const float ContactDist = Cap.Radius + RopeCollisionRadius + LatchContactBand;
			const FVector Closest = FMath::ClosestPointOnSegment(Positions[i], Cap.A, Cap.B);
			const float Dist = (Positions[i] - Closest).Size();
			if (Dist <= ContactDist && Dist < BestDist)
			{
				const FVector AxisVec = Cap.B - Cap.A;
				const float AxisLen = AxisVec.Size();
				BestDist = Dist;
				BestCap = c;
				BestClosest = Closest;
				BestAxisLen = AxisLen;
				BestAxis = (AxisLen > KINDA_SMALL_NUMBER) ? (AxisVec / AxisLen) : FVector::UpVector;
			}
		}

		if (BestCap < 0)
		{
			ContactDwell[i] = 0.0f; // not touching anything — reset dwell
			continue;
		}

		// Dwell long enough → establish the latch, storing the contact relative to the capsule.
		ContactDwell[i] += FrameDt;
		if (ContactDwell[i] >= LatchContactTime)
		{
			const FVector Radial = Positions[i] - BestClosest;
			const float RadialDist = Radial.Size();

			LatchCapsule[i] = BestCap;
			// Along = distance of the closest point from the capsule's A end, projected on the axis.
			LatchAlong[i] = FMath::Clamp(FVector::DotProduct(BestClosest - ActiveCapsules[BestCap].A, BestAxis), 0.0f, BestAxisLen);
			LatchRadialDir[i] = (RadialDist > KINDA_SMALL_NUMBER)
				? (Radial / RadialDist)
				: FVector::CrossProduct(BestAxis, FVector::ForwardVector).GetSafeNormal(1e-4f, FVector::RightVector);
			LatchRadialDist[i] = FMath::Max(RadialDist, 1e-3f);
			LatchAxis[i] = BestAxis;
			InvMasses[i] = 0.0f;
		}
	}
}

void ARopePoCActor::AccumulateLatchReaction()
{
	if (!bEnableTwoWayPull || !bEnableWrapLatch)
	{
		return;
	}

	for (int32 i = 1; i < Positions.Num() - 1; ++i)
	{
		const int32 c = LatchCapsule[i];
		if (c < 0 || !CapsuleReaction.IsValidIndex(c))
		{
			continue;
		}

		// A latched node is pinned to the limb, so it gets no push-out reaction. Instead the
		// rope tension at this node (its segments stretched toward the neighbours) is the force
		// the rope exerts on the limb — pull the capsule that way.
		FVector Pull = FVector::ZeroVector;
		const int32 Neighbours[2] = { i - 1, i + 1 };
		for (int32 j : Neighbours)
		{
			const FVector D = Positions[j] - Positions[i];
			const float Len = D.Size();
			const float Stretch = Len - SegmentLength;
			if (Stretch > 0.0f && Len > KINDA_SMALL_NUMBER)
			{
				Pull += (D / Len) * Stretch;
			}
		}

		CapsuleReaction[c] += Pull;
		CapsuleReactionPoint[c] += Positions[i];
		CapsuleReactionWeight[c] += 1.0f;
	}
}

void ARopePoCActor::ReleaseAllWraps()
{
	for (int32 i = 0; i < LatchCapsule.Num(); ++i)
	{
		if (LatchCapsule[i] >= 0)
		{
			LatchCapsule[i] = -1;
			InvMasses[i] = 1.0f;
		}
		if (ContactDwell.IsValidIndex(i))
		{
			ContactDwell[i] = 0.0f;
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
		// Orange = latched wrap node, Red = pinned endpoint, Yellow = free particle.
		const bool bLatched = LatchCapsule.IsValidIndex(i) && LatchCapsule[i] >= 0;
		const FColor PointColor = bLatched ? FColor(255, 128, 0)
			: (InvMasses[i] <= 0.0f) ? FColor::Red : FColor::Yellow;
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
			int32 LatchedCount = 0;
			for (int32 LC : LatchCapsule) { if (LC >= 0) { ++LatchedCount; } }
			GEngine->AddOnScreenDebugMessage(
				reinterpret_cast<uint64>(this), 0.0f, BudgetColor,
				FString::Printf(TEXT("[Rope] solve %.3f ms (avg) | particles %d | iters %d | sub %d | capsules %d | latched %d"),
					AvgSolveMs, NumParticles, SolverIterations, FMath::Clamp(SimSubsteps, 1, 16), FrameCapsules.Num(), LatchedCount));
		}
	}
}
