// Copyright Epic Games, Inc. All Rights Reserved.

#include "Demo/RopeDemoLever.h"
#include "Collision/RopeWrapTargetComponent.h"
#include "DynamicRopeLog.h"
#include "RopeComponent.h"
#include "Subsystem/RopeSimSubsystem.h"

#include "Components/StaticMeshComponent.h"
#include "DrawDebugHelpers.h"

ARopeDemoLever::ARopeDemoLever()
{
	// Only the handle swing and the target rotation need a tick; the wrap tracking itself is event
	// driven.
	PrimaryActorTick.bCanEverTick = true;

	// Bare components with no meshes and no transforms: the geometry is authored on the Blueprint
	// subclass or the placed instance, since the project supplies its own assets.
	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);

	Frame = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Frame"));
	Frame->SetupAttachment(Root);

	Pivot = CreateDefaultSubobject<USceneComponent>(TEXT("Pivot"));
	Pivot->SetupAttachment(Root);
	Pivot->SetMobility(EComponentMobility::Movable);

	Handle = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Handle"));
	Handle->SetupAttachment(Pivot);
	// It actually swings, so it has to be movable; a static mesh component defaults to static.
	Handle->SetMobility(EComponentMobility::Movable);

	// The wrap opt-in: serves the handle's simple collision as wrappable colliders each frame.
	WrapTarget = CreateDefaultSubobject<URopeWrapTargetComponent>(TEXT("WrapTarget"));
	WrapTarget->TargetComponent = Handle;
}

void ARopeDemoLever::BeginPlay()
{
	Super::BeginPlay();

	CurrentAngleDeg = RestAngleDeg;
	if (Pivot)
	{
		Pivot->SetRelativeRotation(FRotator(CurrentAngleDeg, 0.0f, 0.0f));
	}

	// The target's rotation now is the Off end of its motion; the On end is this composed with
	// TargetRotationOffset.
	if (IsValid(TargetActor))
	{
		TargetInitialQuat = TargetActor->GetActorQuat();
	}

	// Receive a notification whenever any rope in the world wraps or releases, which allows tracking
	// the handle's engagements without knowing in advance which rope will wrap it.
	if (URopeSimSubsystem* Sim = URopeSimSubsystem::Get(GetWorld()))
	{
		WrappedHandle = Sim->OnAnyRopeWrapped.AddUObject(this, &ARopeDemoLever::HandleAnyRopeWrapped);
		ReleasedHandle = Sim->OnAnyRopeReleased.AddUObject(this, &ARopeDemoLever::HandleAnyRopeReleased);
	}
}

void ARopeDemoLever::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (URopeSimSubsystem* Sim = URopeSimSubsystem::Get(GetWorld()))
	{
		Sim->OnAnyRopeWrapped.Remove(WrappedHandle);
		Sim->OnAnyRopeReleased.Remove(ReleasedHandle);
	}
	WrappingRopes.Reset();

	Super::EndPlay(EndPlayReason);
}

void ARopeDemoLever::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (!Pivot)
	{
		return;
	}

	// A rope can be destroyed while still wrapped without a release event reaching us, so expired
	// entries are dropped here rather than trusted forever.
	for (auto It = WrappingRopes.CreateIterator(); It; ++It)
	{
		if (!It->IsValid())
		{
			It.RemoveCurrent();
		}
	}

	const float PullRate = ComputePullRate();
	const float PreviousAngle = CurrentAngleDeg;
	if (PullRate > 0.0f)
	{
		CurrentAngleDeg = FMath::Min(CurrentAngleDeg + PullRate * DeltaSeconds, PulledAngleDeg);
	}
	else if (!(bHoldAtEnd && bOn))
	{
		// Not being pulled: the handle springs back to rest on its own. With bHoldAtEnd, a completed
		// pull (bOn) pins it at the pulled end instead, until SetOn(false) releases the hold.
		CurrentAngleDeg = FMath::FInterpConstantTo(CurrentAngleDeg, RestAngleDeg, DeltaSeconds, ReturnAngularSpeed);
	}

	if (!FMath::IsNearlyEqual(CurrentAngleDeg, PreviousAngle))
	{
		Pivot->SetRelativeRotation(FRotator(CurrentAngleDeg, 0.0f, 0.0f));
	}

	// The latch: reaching the pulled end fires once, then stays disarmed until the handle has
	// returned near rest, so holding it pinned at the end cannot repeat-fire the toggle.
	const float Range = FMath::Max(PulledAngleDeg - RestAngleDeg, 1.0f);
	if (bArmed && CurrentAngleDeg >= PulledAngleDeg - KINDA_SMALL_NUMBER)
	{
		ToggleFromPull();
		bArmed = false;
	}
	else if (!bArmed && CurrentAngleDeg <= RestAngleDeg + Range * 0.25f)
	{
		bArmed = true;
	}

	UpdateTargetRotation();
}

float ARopeDemoLever::GetPullProgress() const
{
	const float Range = PulledAngleDeg - RestAngleDeg;
	return Range > KINDA_SMALL_NUMBER ? FMath::Clamp((CurrentAngleDeg - RestAngleDeg) / Range, 0.0f, 1.0f) : 0.0f;
}

void ARopeDemoLever::SetOn(bool bNewOn)
{
	if (bOn == bNewOn)
	{
		return;
	}

	bOn = bNewOn;

	UE_LOG(LogDynamicRope, Log, TEXT("[%s] demo lever forced %s"), *GetName(), bOn ? TEXT("On") : TEXT("Off"));

	OnLeverStateChanged.Broadcast(this, bOn);
}

void ARopeDemoLever::HandleAnyRopeWrapped(const FRopeWrappedEventInfo& Info)
{
	if (!Handle || Info.Mesh.Get() != Handle)
	{
		// A different component was wrapped, so this is ignored.
		return;
	}

	if (Info.Rope.IsValid())
	{
		WrappingRopes.Add(Info.Rope);
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] demo lever wrapped by %s (%d rope(s) attached)"),
			*GetName(), *GetNameSafe(Info.Rope.Get()), WrappingRopes.Num());
	}
}

void ARopeDemoLever::HandleAnyRopeReleased(const URopeComponent* Rope, const USceneComponent* WrappedMesh,
	FName /*Bone*/, ERopeReleaseReason /*Reason*/)
{
	if (!Handle || WrappedMesh != Handle)
	{
		return;
	}

	for (auto It = WrappingRopes.CreateIterator(); It; ++It)
	{
		if (!It->IsValid() || It->Get() == Rope)
		{
			It.RemoveCurrent();
		}
	}
}

float ARopeDemoLever::ComputePullRate() const
{
	const FVector Tip = ComputeTipWorld(CurrentAngleDeg);
	// The swing arc tangent, taken numerically from two nearby angles. Deriving it from the hinge
	// axis by hand would have to reproduce the rotator's pitch sign convention; this cannot disagree
	// with the transform that actually moves the handle.
	const FVector Tangent = (ComputeTipWorld(CurrentAngleDeg + 1.0f) - Tip).GetSafeNormal();

#if ENABLE_DRAW_DEBUG
	// Drawn before the early-outs, so an empty engagement set is visible too: a yellow tip with no
	// arrows and "ropes=0" means the wrap never bound to the handle in the first place.
	if (bDebugDrawPull)
	{
		DrawDebugSphere(GetWorld(), Tip, 6.0f, 8, FColor::Yellow, false, -1.0f, SDPG_Foreground);
		DrawDebugDirectionalArrow(GetWorld(), Tip, Tip + Tangent * 80.0f, 20.0f, FColor::Yellow,
			false, -1.0f, SDPG_Foreground, 2.0f);
		DrawDebugString(GetWorld(), Tip + FVector(0.0f, 0.0f, 24.0f),
			FString::Printf(TEXT("ropes=%d"), WrappingRopes.Num()), nullptr, FColor::Yellow, 0.0f, true);
	}
#endif

	if (WrappingRopes.Num() == 0 || Tangent.IsNearlyZero())
	{
		return 0.0f;
	}

	// Several ropes can wrap one handle; the strongest plausible pull wins rather than stacking,
	// which keeps the swing speed bounded.
	float BestRate = 0.0f;
	int32 RopeIndex = 0;
	for (const TWeakObjectPtr<URopeComponent>& WeakRope : WrappingRopes)
	{
		const URopeComponent* Rope = WeakRope.Get();
		if (!Rope)
		{
			continue;
		}

		// Gate 1: a slack rope draped over the handle is not a pull.
		const bool bTautOk = !bRequireTautPull || Rope->IsPullTaut();

		// Gate 2: the authoritative constraint tension carries the load, so a threshold here means
		// "yank hard enough", not "the visual rope looks stretched".
		const float Tension = Rope->GetConstraintTension();
		const bool bTensionOk = Tension >= PullTensionThreshold;

		// Gate 3: the pull must actually point along the tip's swing arc. Node 0 is the hand end,
		// so the direction the rope hauls the handle is tip towards hand.
		const FVector PullDir = (Rope->GetNodePosition(0) - Tip).GetSafeNormal();
		const float Alignment = PullDir.IsNearlyZero() ? -1.0f : static_cast<float>(PullDir | Tangent);
		const bool bAlignOk = Alignment >= MinPullAlignment;

		const bool bPass = bTautOk && bTensionOk && bAlignOk;

#if ENABLE_DRAW_DEBUG
		if (bDebugDrawPull)
		{
			const FColor Color = bPass ? FColor::Green : FColor::Red;
			DrawDebugDirectionalArrow(GetWorld(), Tip, Tip + PullDir * 120.0f, 20.0f, Color,
				false, -1.0f, SDPG_Foreground, 2.0f);
			DrawDebugString(GetWorld(), Tip + FVector(0.0f, 0.0f, 44.0f + 18.0f * RopeIndex),
				FString::Printf(TEXT("%s: taut=%s tension=%.0f/%.0f align=%.2f/%.2f"),
					*GetNameSafe(Rope), bTautOk ? TEXT("ok") : TEXT("NO"),
					Tension, PullTensionThreshold, Alignment, MinPullAlignment),
				nullptr, Color, 0.0f, true);
		}
#endif
		++RopeIndex;

		if (!bPass)
		{
			continue;
		}

		// Scaling by the alignment makes the handle follow the pull: square along the arc swings at
		// full speed, a grazing pull crawls.
		BestRate = FMath::Max(BestRate, PullAngularSpeed * Alignment);
	}
	return BestRate;
}

FVector ARopeDemoLever::ComputeTipWorld(float AngleDeg) const
{
	const USceneComponent* Parent = Pivot ? Pivot->GetAttachParent() : nullptr;
	const FTransform ParentTM = Parent ? Parent->GetComponentTransform() : GetActorTransform();
	const FVector PivotRelative = Pivot ? Pivot->GetRelativeLocation() : FVector::ZeroVector;
	const FVector TipLocal = FRotator(AngleDeg, 0.0f, 0.0f).RotateVector(FVector(0.0f, 0.0f, HandleLength));
	return ParentTM.TransformPosition(PivotRelative + TipLocal);
}

void ARopeDemoLever::ToggleFromPull()
{
	bOn = !bOn;

	UE_LOG(LogDynamicRope, Log, TEXT("[%s] demo lever pulled %s (%d rope(s) attached)"),
		*GetName(), bOn ? TEXT("On") : TEXT("Off"), WrappingRopes.Num());

	OnLeverStateChanged.Broadcast(this, bOn);
}

void ARopeDemoLever::UpdateTargetRotation()
{
	if (!IsValid(TargetActor))
	{
		return;
	}

	// The target mirrors the handle in real time — the lever is a crank, not a button — so the
	// travel itself is the motion parameter and an idle handle costs nothing.
	const float Progress = GetPullProgress();
	if (FMath::IsNearlyEqual(Progress, LastAppliedTargetProgress))
	{
		return;
	}
	LastAppliedTargetProgress = Progress;

	// The ease reshapes the mapping, not the timing: the prop moves least near the ends of the
	// travel, which reads as weight while staying directly coupled to the handle.
	const float Eased = FMath::InterpEaseInOut(0.0f, 1.0f, Progress, TargetEaseExponent);

	// In local space the offset is about the target's own axes, so a yawed prop still tips over its
	// own edge; in world space it is about the world axes regardless of the prop's facing.
	const FQuat Offset = TargetRotationOffset.Quaternion();
	const FQuat PulledQuat = bOffsetInLocalSpace ? TargetInitialQuat * Offset : Offset * TargetInitialQuat;
	TargetActor->SetActorRotation(FQuat::Slerp(TargetInitialQuat, PulledQuat, Eased));
}
