// Copyright Epic Games, Inc. All Rights Reserved.

#include "Debug/GameplayDebuggerCategory_Rope.h"

#if WITH_GAMEPLAY_DEBUGGER

#include "RopeComponent.h"
#include "Gameplay/RopeWielderComponent.h"
#include "Debug/RopeDebugSnapshot.h"
#include "Subsystem/RopeDebugSubsystem.h"
// RopeFlightDebug::SelectCandidateBoxes, the candidate box selection, a pure function with unit tests.
#include "Logic/RopeFlightDebugSelection.h"
#include "GameFramework/Actor.h"
// RopeGPU::TubeRingBucket and MaxTubeRings, for diagnosing the GPU tube path and bucket.
#include "RopeTubeBuilder.h"
// RopeGPU::IsRuntimeSupported, the same runtime test the proxy's GPU tube gate uses, meaning an RHI plus SM5.
#include "RopeGPUSolver.h"
// DrawDebug* with SDPG_Foreground, for the collider overlay, drawn on top like an editor selection.
#include "DrawDebugHelpers.h"

namespace
{
	// Reproduce the GPU tube eligibility on the game thread with the same expression the proxy uses,
	// rather than reading the proxy's own flag across threads. The proxy freezes its decision when it is
	// created, so the two can diverge after a runtime property change, which is why the label reads
	// "tube-eligible". The node count comes from the same source the proxy uses, the configured particle
	// count. With advanced display off, only the path and the reason for a CPU fallback are reported: the
	// reason is actionable, while the bucket and ring counts are internal dispatch numbers and belong in
	// the detail view.
	FString TubeDiagString(int32 NumNodes, int32 WantedSubdiv, bool bAdvanced)
	{
		if (!RopeGPU::IsRuntimeSupported())
		{
			// There is no renderable RHI, or it is below SM5, so the proxy uses the CPU tube build
			// regardless of the ring count.
			return FString(TEXT("{red}cpu{grey}(no gpu runtime)"));
		}
		const int32 Nodes = FMath::Max(2, NumNodes);
		// Reduce the subdivision to fit the ring limit with the same helper the proxy uses, so the bucket
		// and ring counts shown are the ones actually in use.
		const int32 Subdiv = RopeGPU::ComputeTubeSubdiv(Nodes, WantedSubdiv);
		const int32 NumRings = (Nodes - 1) * Subdiv + 1;
		const int32 Bucket = RopeGPU::TubeRingBucket(NumRings);
		if (Bucket > 0)
		{
			return bAdvanced
				? FString::Printf(TEXT("{green}gpu{grey}(bucket %d, rings %d)"), Bucket, NumRings)
				: FString(TEXT("{green}gpu"));
		}
		return FString::Printf(TEXT("{red}cpu{grey}(rings %d > %d)"), NumRings, RopeGPU::MaxTubeRings());
	}

	// The solve path this rope took this frame. The subsystem puts solve frames and override-only frames
	// onto the GPU alike, so the stepped flag alone cannot decide it. The order of the tests puts what
	// actually happened first: the sleeping flag is settled at the end of the frame, so checking it first
	// would let SLEEP mask a SOLVE that genuinely happened this frame.
	const TCHAR* SolvePathToken(bool bSleeping, bool bSolved, bool bGpuStepped, bool bLogicOverride)
	{
		if (bGpuStepped)
		{
			return bSolved ? TEXT("{green}GPU_SOLVE") : TEXT("{green}GPU_OVERRIDE");
		}
		if (bSolved)
		{
			// A frame solved on the CPU because the rope was not eligible for GPU residency, whether from
			// exceeding the node count, having no RHI, or similar.
			return TEXT("{red}CPU_SOLVE");
		}
		if (bLogicOverride)
		{
			// A frame with no solve, where logic updated the positions.
			return TEXT("{yellow}CPU_OVERRIDE");
		}
		// Nothing happened, which distinguishes being asleep from simply having nothing to do.
		return bSleeping ? TEXT("{cyan}SLEEP") : TEXT("{grey}IDLE");
	}

	const TCHAR* DebugPhaseName(ERopePhase Phase)
	{
		switch (Phase)
		{
		case ERopePhase::Free:       return TEXT("Free");
		case ERopePhase::Flight:     return TEXT("Flight");
		case ERopePhase::Contacting: return TEXT("Contacting");
		case ERopePhase::Wrapping:   return TEXT("Wrapping");
		case ERopePhase::Wrapped:    return TEXT("Wrapped");
		case ERopePhase::Releasing:  return TEXT("Releasing");
		case ERopePhase::GuidedThrow: return TEXT("GuidedThrow");
		case ERopePhase::Loaded:       return TEXT("Loaded");
		default:                     return TEXT("?");
		}
	}

	// The resolve mode's display name, reported alongside the automatic release text. The enumerator name
	// is printed as-is, so a value read off the screen leads straight back to the code.
	const TCHAR* DebugResolveModeName(ERopeWrapResolveMode Mode)
	{
		switch (Mode)
		{
		case ERopeWrapResolveMode::FullSimulation: return TEXT("FullSimulation");
		case ERopeWrapResolveMode::AssistedJudged: return TEXT("AssistedJudged");
		case ERopeWrapResolveMode::GuaranteedWrap: return TEXT("GuaranteedWrap");
		default:                                   return TEXT("?");
		}
	}

	// Compresses ascending node indices into contiguous ranges, such as "12-16,23-24". It shows, on one
	// line, the "from where to where, and is it one piece" that a count alone cannot. Input that is not
	// ascending simply produces more, smaller ranges.
	FString NodeRangeString(const TArray<int32>& Ascending)
	{
		FString Out;
		for (int32 i = 0; i < Ascending.Num(); )
		{
			const int32 RunStart = Ascending[i];
			int32 RunEnd = RunStart;
			while (i + 1 < Ascending.Num() && Ascending[i + 1] == RunEnd + 1)
			{
				++i;
				RunEnd = Ascending[i];
			}
			++i;
			if (!Out.IsEmpty())
			{
				Out += TEXT(",");
			}
			Out += (RunStart == RunEnd)
				? FString::Printf(TEXT("%d"), RunStart)
				: FString::Printf(TEXT("%d-%d"), RunStart, RunEnd);
		}
		return Out;
	}

	// Proximity nodes are already ascending, because the capture loop fills them in node order, so only
	// the indices are extracted and handed to the range compression.
	FString ProximityRangeString(const TArray<FRopeNodeProximityDebug>& Proximity)
	{
		TArray<int32> Idx;
		Idx.Reserve(Proximity.Num());
		for (const FRopeNodeProximityDebug& P : Proximity)
		{
			Idx.Add(P.NodeIndex);
		}
		return NodeRangeString(Idx);
	}

	FColor CandidateSourceColor(ERopeContactCandidateSource Source)
	{
		switch (Source)
		{
		case ERopeContactCandidateSource::Actual: return FColor::Cyan;
		case ERopeContactCandidateSource::PredictiveFree: return FColor::Green;
		case ERopeContactCandidateSource::PredictiveGuided: return FColor(255, 80, 255);
		default: return FColor::White;
		}
	}

	// Display names for candidate sources, used by the summary and legend text, paired with the colours.
	const TCHAR* CandidateSourceName(ERopeContactCandidateSource Source)
	{
		switch (Source)
		{
		case ERopeContactCandidateSource::Actual: return TEXT("Actual");
		case ERopeContactCandidateSource::PredictiveFree: return TEXT("PredictiveFree");
		case ERopeContactCandidateSource::PredictiveGuided: return TEXT("PredictiveGuided");
		default: return TEXT("?");
		}
	}
}

FGameplayDebuggerCategory_Rope::FGameplayDebuggerCategory_Rope()
{
	bShowOnlyWithDebugActor = false;

	// Sub-view toggle keys, received while the category is active, replacing console variables.
	// The keys are given as FName literals, which avoids a link dependency on the InputCore module
	// symbols that EKeys and FKey would bring.
	const FGameplayDebuggerInputHandlerConfig NodesCfg(TEXT("ToggleNodes"), TEXT("P"));
	const FGameplayDebuggerInputHandlerConfig FlightCfg(TEXT("ToggleFlight"), TEXT("U"));
	const FGameplayDebuggerInputHandlerConfig WrapCfg(TEXT("ToggleWrap"), TEXT("I"));
	const FGameplayDebuggerInputHandlerConfig CollidersCfg(TEXT("ToggleColliders"), TEXT("O"));
	const FGameplayDebuggerInputHandlerConfig AimCfg(TEXT("ToggleAim"), TEXT("J"));
	const FGameplayDebuggerInputHandlerConfig AdvancedCfg(TEXT("ToggleAdvanced"), TEXT("K"));
	BindKeyPress(NodesCfg, this, &FGameplayDebuggerCategory_Rope::OnToggleNodes);
	BindKeyPress(FlightCfg, this, &FGameplayDebuggerCategory_Rope::OnToggleFlight);
	BindKeyPress(WrapCfg, this, &FGameplayDebuggerCategory_Rope::OnToggleWrap);
	BindKeyPress(CollidersCfg, this, &FGameplayDebuggerCategory_Rope::OnToggleColliders);
	BindKeyPress(AimCfg, this, &FGameplayDebuggerCategory_Rope::OnToggleAim);
	BindKeyPress(AdvancedCfg, this, &FGameplayDebuggerCategory_Rope::OnToggleAdvanced);

	// Point display is replicated through its own data pack rather than as shapes; the reason is given in
	// the FRepData comment in the header.
	// It resets on tick by default, so it is emptied on every collection tick and CollectData does not
	// reset it separately.
	SetDataPackReplication<FRepData>(&DataPack);
}

void FGameplayDebuggerCategory_Rope::FRepData::Serialize(FArchive& Ar)
{
	int32 NumPoints = Points.Num();
	Ar << NumPoints;
	if (Ar.IsLoading())
	{
		Points.SetNum(NumPoints);
	}
	for (FPoint& Point : Points)
	{
		Ar << Point.Location;
		Ar << Point.Color;
		Ar << Point.Size;
	}
}

void FGameplayDebuggerCategory_Rope::AddPoint(const FVector& Location, float PixelSize, const FColor& Color)
{
	FRepData::FPoint Point;
	Point.Location = Location;
	Point.Color = Color;
	Point.Size = PixelSize;
	DataPack.Points.Add(Point);
}

void FGameplayDebuggerCategory_Rope::DrawData(APlayerController* OwnerPC, FGameplayDebuggerCanvasContext& CanvasContext)
{
	FGameplayDebuggerCategory::DrawData(OwnerPC, CanvasContext);

	// Points are drawn in the foreground, because many nodes sit inside or against character and wall
	// meshes and would be buried at world depth priority.
	// DrawData runs every frame on the viewing side, so the points do not flicker even when the
	// collection interval is longer than a frame.
	if (UWorld* World = CanvasContext.World.Get())
	{
		for (const FRepData::FPoint& Point : DataPack.Points)
		{
			DrawDebugPoint(World, Point.Location, Point.Size, Point.Color, false, -1.0f, SDPG_Foreground);
		}
	}
}

TSharedRef<FGameplayDebuggerCategory> FGameplayDebuggerCategory_Rope::MakeInstance()
{
	return MakeShareable(new FGameplayDebuggerCategory_Rope());
}

void FGameplayDebuggerCategory_Rope::OnToggleNodes()     { ViewMask ^= static_cast<uint8>(EView::Nodes); }
void FGameplayDebuggerCategory_Rope::OnToggleFlight()    { ViewMask ^= static_cast<uint8>(EView::Flight); }
void FGameplayDebuggerCategory_Rope::OnToggleWrap()      { ViewMask ^= static_cast<uint8>(EView::Wrap); }
void FGameplayDebuggerCategory_Rope::OnToggleColliders() { ViewMask ^= static_cast<uint8>(EView::Colliders); }
void FGameplayDebuggerCategory_Rope::OnToggleAim()       { ViewMask ^= static_cast<uint8>(EView::Aim); }
void FGameplayDebuggerCategory_Rope::OnToggleAdvanced()  { ViewMask ^= static_cast<uint8>(EView::Advanced); }

ERopeDebugCapture FGameplayDebuggerCategory_Rope::BuildCaptureMask() const
{
	ERopeDebugCapture Mask = ERopeDebugCapture::None;
	if (HasView(EView::Nodes))     { Mask |= ERopeDebugCapture::Nodes; }
	if (HasView(EView::Flight))    { Mask |= ERopeDebugCapture::Flight; }
	if (HasView(EView::Wrap))      { Mask |= ERopeDebugCapture::Wrap; }
	if (HasView(EView::Colliders)) { Mask |= ERopeDebugCapture::Colliders; }
	// Aim and Advanced collect nothing: the former reads the wielder live, and the latter only changes the
	// display detail of what has already been collected.
	return Mask;
}

void FGameplayDebuggerCategory_Rope::CollectData(APlayerController* OwnerPC, AActor* DebugActor)
{
	if (!DebugActor)
	{
		AddTextLine(TEXT("{grey}no debug actor"));
		return;
	}

	// Register the target actor and the capture scope, so the next frame's sim tick captures that actor's
	// ropes, and only the views that are enabled.
	// Toggling a view therefore takes effect one frame later, which is inherent to registering a target.
	URopeDebugSubsystem* Dbg = URopeDebugSubsystem::Get(DebugActor->GetWorld());
	if (Dbg)
	{
		Dbg->SetTarget(DebugActor, BuildCaptureMask());
	}

	auto OnOff = [](bool b) { return b ? TEXT("{green}on") : TEXT("{grey}off"); };
	AddTextLine(FString::Printf(
		TEXT("{white}views  [P]nodes=%s{white} [U]flight=%s{white} [I]wrap=%s{white} [O]colliders=%s{white} [J]aim=%s{white} [K]advanced=%s"),
		OnOff(HasView(EView::Nodes)), OnOff(HasView(EView::Flight)), OnOff(HasView(EView::Wrap)),
		OnOff(HasView(EView::Colliders)), OnOff(HasView(EView::Aim)), OnOff(HasView(EView::Advanced))));

	// Aiming belongs to the wielder rather than the rope, so it is found once on the actor and drawn
	// separately from the rope loop.
	if (const URopeWielderComponent* Wielder = DebugActor->FindComponentByClass<URopeWielderComponent>())
	{
		DrawAim(*Wielder);
	}

	int32 Count = 0;
	for (UActorComponent* Comp : DebugActor->GetComponents())
	{
		const URopeComponent* Rope = Cast<URopeComponent>(Comp);
		if (!Rope)
		{
			continue;
		}
		++Count;

		// Always-present information such as the phase and the centreline comes from the live component,
		// while the diagnostic overlays come from the snapshot where one exists.
		const FRopeDebugSnapshot* Snap = Dbg ? Dbg->GetSnapshot(Rope) : nullptr;
		// The Flight overlay source: the live component while it is in Flight, and otherwise the last held
		// Flight snapshot within a half-second window, so it stays visible briefly even after the capture
		// decision moves the rope on to Wrapping.
		float HeldFlightAge = 0.0f;
		const FRopeDebugSnapshot* HeldFlight = (Snap && Snap->bHasFlight) ? nullptr
			: (Dbg ? Dbg->GetHeldFlightSnapshot(Rope, HeldFlightAge) : nullptr);
		DrawRope(*Rope, Snap, HeldFlight, HeldFlightAge);
	}

	if (Count == 0)
	{
		AddTextLine(TEXT("{grey}no URopeComponent on debug actor"));
	}
}

void FGameplayDebuggerCategory_Rope::DrawAim(const URopeWielderComponent& Wielder)
{
	if (!HasView(EView::Aim))
	{
		return;
	}

	// Nothing is queried here: the wielder sweeps every tick while aiming is valid and this only reads the
	// sample it left behind. With aiming off the sample is empty, its ray length is 0, and there is no ray
	// to draw.

	// In a mode that never uses an aim ray at all, such as FullSimulation, the section is omitted
	// entirely. It would otherwise read the same forever unless the mode changed, which has no diagnostic
	// value and merely occupies the screen even in the detail view.
	if (!Wielder.UsesAimRay())
	{
		return;
	}
	// The mode does use aiming but the rope cannot be thrown in this phase; GuaranteedWrap only aims from
	// Loaded. It is a temporary state that resolves on entering that phase, which makes it the answer to
	// "why is aiming not engaging". It is the very reason the aim view can be on with no ray visible, so
	// it is reported regardless of the detail setting.
	if (!Wielder.IsAimActive())
	{
		AddTextLine(TEXT("  {white}aim: {grey}inactive — GuaranteedWrap aims from Loaded only"));
		return;
	}

	const FRopeAimHudSample& Aim = Wielder.GetAimHudSample();
	const UWorld* World = Wielder.GetWorld();
	if (!World || Aim.RayLength <= KINDA_SMALL_NUMBER || Aim.RayDirection.IsNearlyZero())
	{
		AddTextLine(TEXT("  {white}aim: {grey}no ray"));
		return;
	}

	// The colour convention matches the aiming HUD: green for wrappable, red for hit but not wrappable,
	// and cyan for a miss.
	const bool bAnyHit = Aim.bHasTarget || Aim.bBlocked;
	const FColor MainColor = Aim.bHasTarget ? FColor::Green : (Aim.bBlocked ? FColor::Red : FColor::Cyan);
	const FVector RayDir = Aim.RayDirection.GetSafeNormal();
	const FVector RayStart = Aim.RayOrigin;
	const FVector RayEnd = RayStart + RayDir * Aim.RayLength;
	const FVector RayStop = bAnyHit ? Aim.HitWorldPos : RayEnd;

	// For the same reason as the colliders, this uses DrawDebug* in the foreground rather than AddShape:
	// AddShape hardcodes its depth priority to world, which buries it behind geometry. It is not a
	// limitation of shape expressiveness, since MakeCapsule does take a rotation.
	// The lifetime is kept short so it survives only until the next collection.
	constexpr float LifeTime = 0.05f;
	constexpr uint8 FG = SDPG_Foreground;
	// The capsule dimensions are exactly the length and radius handed to QuerySwept, so the volume aiming
	// tests against can be seen directly.
	if (Aim.QueryRadius > KINDA_SMALL_NUMBER)
	{
		const FQuat CapsuleRotation = FRotationMatrix::MakeFromZ(RayDir).ToQuat();
		DrawDebugCapsule(World, (RayStart + RayEnd) * 0.5f,
			Aim.RayLength * 0.5f + Aim.QueryRadius, Aim.QueryRadius,
			CapsuleRotation, MainColor, false, LifeTime, FG, 1.0f);
	}
	DrawDebugLine(World, RayStart, RayStop, MainColor, false, LifeTime, FG, 2.0f);
	if (bAnyHit)
	{
		// The stretch beyond the hit, showing how far aiming could have reached.
		DrawDebugLine(World, RayStop, RayEnd, FColor(96, 0, 0), false, LifeTime, FG, 1.0f);
		// The bone name is not drawn as a 3D label, because the aim text line below reports the same name
		// permanently and it would duplicate it.
		DrawDebugSphere(World, Aim.HitWorldPos, 8.0f, 12, FColor::Yellow, false, LifeTime, FG, 2.0f);
	}

	// Hold the temporary from ToString() in a local: taking it as a const TCHAR* would already dangle on
	// the next line.
	const FString BoneText = Aim.Bone.IsNone() ? FString(TEXT("-")) : Aim.Bone.ToString();
	if (Aim.bHasTarget)
	{
		AddTextLine(FString::Printf(TEXT("  {white}aim: {green}%s{white} dist=%.0f radius=%.1f"),
			*BoneText, Aim.Distance, Aim.QueryRadius));
	}
	else if (Aim.bBlocked)
	{
	// The ray hit something that cannot be wrapped: no bone, no source mesh, or refused by CanWrapTarget.
		AddTextLine(FString::Printf(TEXT("  {white}aim: {red}blocked{white} %s dist=%.0f {grey}(not wrappable)"),
			*BoneText, Aim.Distance));
	}
	else
	{
		AddTextLine(FString::Printf(TEXT("  {white}aim: {grey}no target{white} radius=%.1f"), Aim.QueryRadius));
	}
}

void FGameplayDebuggerCategory_Rope::DrawRope(const URopeComponent& Rope, const FRopeDebugSnapshot* Snap,
	const FRopeDebugSnapshot* HeldFlight, float HeldFlightAgeSeconds)
{
	// One screen uses a single point in time: where a snapshot exists, the header, the centreline and the
	// overlays all read from it. Leaving the header live would draw the same node at two different moments
	// and read as simulation jitter or an unstable latch.
	// Only on the first frame, before any snapshot exists, is the header taken live and labelled as such.
	const ERopePhase PhaseEnd = Snap ? Snap->Phase : Rope.GetPhase();
	const ERopePhase PhaseStart = Snap ? Snap->PhaseAtFrameStart : PhaseEnd;
	const TArray<FVector>& Points = Snap ? Snap->Positions : Rope.GetCenterlinePositions();
	// The node count in the header. In the default aim-only state the positions are not copied, so the
	// point array is empty; the NodeCount scalar, which the snapshot always carries, is used instead. On
	// the first live frame the point count is used.
	const int32 NodeCount = Snap ? Snap->NodeCount : Points.Num();
	const FName Bone = Snap ? Snap->WrapBoneName : Rope.GetWrappedBoneName();
	const bool bSleeping = Snap ? Snap->bSleeping : Rope.IsSleeping();
	const float LODScale = Snap ? Snap->LodScale : Rope.GetSolverLODScale();

	// When the rope changed phase within the frame, both the start and the end are reported. That
	// combination is the clue to what caused the transition; on a Flight to Contacting frame, for example,
	// the Flight overlay holds the observations that caused it.
	const FString PhaseText = (PhaseStart != PhaseEnd)
		? FString::Printf(TEXT("%s{grey}→{white}%s"), DebugPhaseName(PhaseStart), DebugPhaseName(PhaseEnd))
		: FString(DebugPhaseName(PhaseEnd));

	// How many frames behind the current one the snapshot is. It is omitted entirely at 0, meaning it was
	// captured this frame, so seeing this token at all indicates a delay, and a growing value means the
	// simulation is not keeping up with capture, as when the target is released or the game is paused. It
	// is spelled out in parentheses to pair with the opposite branch's live label.
	const uint64 AgeFrames = (Snap && GFrameCounter > Snap->FrameStamp) ? (GFrameCounter - Snap->FrameStamp) : 0;
	const FString AgeText = Snap
		? ((AgeFrames > 0)
			? FString::Printf(TEXT("  {grey}(%llu frame%s behind)"),
				static_cast<unsigned long long>(AgeFrames), (AgeFrames == 1) ? TEXT("") : TEXT("s"))
			: FString())
		: FString(TEXT("  {grey}(live — diag pending)"));

	// Identity: the component name plus the owning actor. An index reflects the collection order and can
	// change between frames, so it cannot identify a rope. On the first frame, with no snapshot, the same
	// values are read from the live component.
	const FString NameText = Snap ? Snap->ComponentName : Rope.GetName();
	const AActor* LiveOwner = Rope.GetOwner();
	const FString OwnerText = Snap ? Snap->OwnerActorName : (LiveOwner ? LiveOwner->GetName() : TEXT("None"));

	// Scaling state: whether the rope is asleep, meaning the solve is skipped, and the distance LOD
	// iteration scale, which is below 1 while it is being reduced.
	// The mode is always shown, because each mode has an entirely different contract for establishing a
	// wrap and a different set of meaningful settings, which is the premise for reading every other line.
	AddTextLine(FString::Printf(
		TEXT("{yellow}%s{grey}@%s{white} phase=%s mode=%s nodes=%d wrapBone=%s%s%s%s"),
		*NameText, *OwnerText, *PhaseText,
		DebugResolveModeName(Snap ? Snap->ResolveMode : Rope.ResolveMode), NodeCount,
		Bone.IsNone() ? TEXT("-") : *Bone.ToString(),
		bSleeping ? TEXT("  {cyan}asleep") : TEXT(""),
		LODScale < 0.999f ? *FString::Printf(TEXT("  {cyan}lod=x%.2f"), LODScale) : TEXT(""),
		*AgeText));

	// The solve path, as one of six tokens, plus the tube eligibility, which is a recomputed estimate
	// rather than the proxy's actual state; see the comment on TubeDiagString.
	AddTextLine(FString::Printf(
		TEXT("  {grey}solve=%s{grey} tube-eligible=%s"),
		Snap ? SolvePathToken(Snap->bSleeping, Snap->bSolveThisFrame, Snap->bGpuStepped, Snap->bLogicOverride)
			 : SolvePathToken(Rope.IsSleeping(), Rope.WasSolvedThisFrame(),
					Rope.IsGpuSteppedThisFrame(), Rope.HadLogicOverrideThisFrame()),
		*TubeDiagString(Snap ? Snap->NumParticles : Rope.NumParticles,
			Snap ? Snap->TubeSmoothingSubdiv : Rope.TubeSmoothingSubdiv, HasView(EView::Advanced))));

	//~ nodes ------------------------------------------------------------
	if (HasView(EView::Nodes))
	{
		// Only the node points are drawn. The connecting segments would duplicate what the tube mesh
		// already shows, and the phase colour is reported by the phase text on the header line above.
		// Per-node state is the job of the Flight and wrap overlays.
		for (const FVector& Point : Points)
		{
			AddPoint(Point, 6.0f, FColor::Yellow);
		}
		// Highlight the latch nodes, whose indices and positions come from the same snapshot. The size
		// matches an ordinary node and only the colour differs: drawn afterwards it covers the yellow point
		// exactly, so it reads as one node turning red.
		// Enlarging it would exceed the node spacing and become a blob that hides its neighbours.
		if (Snap)
		{
			for (int32 NodeIdx : Snap->LatchedNodes)
			{
				if (Points.IsValidIndex(NodeIdx))
				{
					AddPoint(Points[NodeIdx], 6.0f, FColor::Red);
				}
			}
		}
	}

	// Everything below is transient capture data, drawn only where a snapshot exists.
	if (!Snap)
	{
		return;
	}
	const FRopeDebugSnapshot& S = *Snap;

	//~ Nodes: the proximity re-query ---------------------------------------
	// Which face a node is against and with what normal. It is node state rather than collider shape, so
	// it belongs to the nodes view.
	// It is called proximity because it is not a real solver contact: the query radius is widened to the
	// collision radius plus 4 cm and re-queried, so nearby nodes that never touched are caught too. The
	// GPU does not read contacts back, so this is re-queried on the CPU.
	if (HasView(EView::Nodes) && S.NodeProximity.Num() > 0)
	{
		if (UWorld* World = Rope.GetWorld())
		{
			constexpr uint8 FG = SDPG_Foreground;
			// Which nodes touched, and over what range, is reported as a range string. A 3D label per node
			// would overlap unreadably, since the spacing is one segment length.
			AddTextLine(FString::Printf(
				TEXT("  {grey}proximity n=%d nodes=%s {grey}(requery r+%.0fcm, not solver contacts)"),
				S.NodeProximity.Num(), *ProximityRangeString(S.NodeProximity), S.ProximityQueryMargin));
			for (int32 i = 0; i < S.NodeProximity.Num(); ++i)
			{
				const FRopeNodeProximityDebug& NP = S.NodeProximity[i];
				// Magenta for static world geometry and orange for everything else, whether a skeletal bone
				// or a wrap target.
				const FColor NColor = NP.bWorldStatic ? FColor(255, 0, 255) : FColor(255, 128, 0);
				const FVector Tip = NP.Position + NP.Normal * 15.0f;
				// The arrow points along the outward contact normal exactly, which is a real direction
				// rather than an axis-aligned one. There is one per node, so it is drawn thin; thick arrows
				// merge between neighbouring nodes and the direction becomes unreadable.
				DrawDebugDirectionalArrow(World, NP.Position, Tip, 5.0f, NColor, false, -1.0f, FG, 1.5f);
				// Only the first range's starting node gets a 3D label, as the anchor point tying the node
				// text above to the 3D view; the remaining numbers are already in the range string. The
				// array is in node index order, so index 0 is merely the lowest number and does not mean the
				// capture candidate, the dominant target or the start of a wrap: it may be a static world
				// contact, shown in magenta, or a phase with no wrapping at all. With several ranges, only
				// the first is labelled.
				if (i == 0)
				{
					DrawDebugString(World, Tip, FString::Printf(TEXT("n%d"), NP.NodeIndex),
						nullptr, NColor, 0.0f, true, 1.0f);
				}
			}
		}
	}

	//~ flight -----------------------------------------------------------
	// The Flight overlay's source: the live component while it is in Flight, and otherwise the last held
	// Flight snapshot within a half-second window.
	// While held, the values from the moment of the decision are drawn as they were, and every one of them
	// is a value or a key, so freezing them is safe. That keeps the overlay visible, frozen, for a moment
	// after the capture moves the rope on to Wrapping. Everything else, including the header, the wrap view
	// and the nodes, stays live.
	const FRopeDebugSnapshot* FlightSnap = (Snap && Snap->bHasFlight) ? Snap : HeldFlight;
	if (HasView(EView::Flight) && FlightSnap && FlightSnap->bHasFlight)
	{
		const FRopeDebugSnapshot& FS = *FlightSnap;
		const bool bHeld = (FlightSnap != Snap);
		// The elapsed label appended to the summary while it is held.
		const FString HeldText = bHeld
			? FString::Printf(TEXT("  {yellow}(held %.1fs)"), HeldFlightAgeSeconds) : FString();

		// Whether this node's contact became a valid candidate. The same node can touch several targets in
		// one frame, so the node index alone cannot decide it; all three of the node index, the bone and
		// the mesh are compared, following the target identity contract.
		// The candidate count is small enough that a linear search suffices.
		auto HasValidCandidateFor = [&FS](const FRopeFlightNodeDebug& Node)
		{
			for (int32 i = 0; i < FS.Candidates.Num(); ++i)
			{
				const FRopeContactCandidate& Candidate = FS.Candidates[i];
				if (!Candidate.bValid || Candidate.NodeIndex != Node.NodeIndex || Candidate.Bone != Node.Contact.Bone)
				{
					continue;
				}
				// The key array is one-to-one with the candidates. Without it, as in an older snapshot, only
				// the bone is matched.
				if (!FS.CandidateMeshKeys.IsValidIndex(i) || FS.CandidateMeshKeys[i] == Node.ContactMeshKey)
				{
					return true;
				}
			}
			return false;
		};

		// The basic Flight view reports only what the rope is trying to catch and whether contact
		// succeeded, while the advanced one adds the raw observations behind candidate selection and the
		// whip.
		// This single gate separates the basic and detailed Flight overlays.
		const bool bAdvanced = HasView(EView::Advanced);

		for (const FRopeFlightNodeDebug& Node : FS.NodeDebug)
		{
			if (bAdvanced)
			{
				// The node's movement line, from its previous to its current position: where it passed
				// through in between. There is one per node, so it is for the detail view only.
				AddShape(FGameplayDebuggerShape::MakeSegment(Node.PrevPosition, Node.Position, 1.0f, FColor::White));
				// Near-body markers are drawn only for nodes with no contact: overlaying a yellow point on a
				// node whose contact point, green or red, already reports its state would simply hide it.
				if (Node.bNearBody && !Node.Contact.bHit)
				{
					AddPoint(Node.Position, 8.0f, FColor::Yellow);
				}
			}

			if (Node.Contact.bHit)
			{
				// Green when the contact became a valid candidate and red when it was rejected. This
				// judgement is the core of the basic Flight view, so it is always shown.
				const FColor HitColor = HasValidCandidateFor(Node) ? FColor::Green : FColor::Red;
				AddPoint(Node.Contact.SurfacePoint, 10.0f, HitColor);
				if (bAdvanced)
				{
					// The contact normal, a raw observation, is for the detail view only. Its length is
					// proportional to the node scale rather than a fixed 22 cm.
					const float NormalLen = FMath::Clamp(FS.NodeCollisionRadius * 3.0f, 12.0f, 40.0f);
					AddShape(FGameplayDebuggerShape::MakeSegment(Node.Contact.SurfacePoint,
						Node.Contact.SurfacePoint + Node.Contact.Normal.GetSafeNormal() * NormalLen, 1.0f, FColor::Blue));
				}
			}
		}

		// Candidate box selection. The basic Flight view shows only the single capture target, matching the
		// tracker's mesh and bone, while the advanced one shows that target plus the general candidates
		// with the greatest penetration, up to five. The capture target is always included even when it
		// falls outside the top N, because a predictive target has low penetration and tends to sort last,
		// yet it is exactly what the rope is about to catch on and therefore the most useful thing to see.
		// The selection rule is a pure function pinned by unit tests, in RopeFlightDebugSelectionTests. The
		// advanced flag is defined above.
		const RopeFlightDebug::FCandidateSelection Sel = RopeFlightDebug::SelectCandidateBoxes(
			FS.Candidates, FS.CandidateMeshKeys, FS.TrackerBone, FS.TrackerMeshKey,
			bAdvanced ? 5 : 1, /*bFillWithGeneral=*/bAdvanced);

		for (const int32 i : Sel.BoxIndices)
		{
			const FRopeContactCandidate& Candidate = FS.Candidates[i];
			const FColor SourceColor = CandidateSourceColor(Candidate.Source);
			if (i == Sel.CaptureTargetIndex)
			{
				// The capture target is drawn as two boxes, a white outline around a source-coloured inner
				// box, so its size and outline mark it as the target.
				AddShape(FGameplayDebuggerShape::MakeBox(Candidate.WorldPoint, FVector(4.5f), FColor::White));
				AddShape(FGameplayDebuggerShape::MakeBox(Candidate.WorldPoint, FVector(2.8f), SourceColor));
			}
			else
			{
				AddShape(FGameplayDebuggerShape::MakeBox(Candidate.WorldPoint, FVector(3.5f), SourceColor));
			}
		}

		// A one-line summary: what the rope is trying to catch, as the capture target's mesh and bone, its
		// source, and the total, shown and hidden candidate counts, plus the held elapsed time while held.
		// The mesh name is resolved safely through the key rather than a raw pointer that may dangle, and
		// reads as unknown when it has died.
		if (Sel.CaptureTargetIndex != INDEX_NONE)
		{
			const FRopeContactCandidate& Cap = FS.Candidates[Sel.CaptureTargetIndex];
			FString MeshName(TEXT("?"));
			if (FS.CandidateMeshKeys.IsValidIndex(Sel.CaptureTargetIndex))
			{
				if (const UObject* M = FS.CandidateMeshKeys[Sel.CaptureTargetIndex].ResolveObjectPtr())
				{
					MeshName = M->GetName();
				}
			}
			AddTextLine(FString::Printf(
				TEXT("  {grey}flight capture-target=%s:%s src=%s candidates=%d shown=%d hidden=%d%s"),
				*MeshName, *Cap.Bone.ToString(), CandidateSourceName(Cap.Source),
				Sel.TotalValid, Sel.Shown, Sel.Hidden, *HeldText));
		}
		else
		{
			AddTextLine(FString::Printf(
				TEXT("  {grey}flight capture-target=none candidates=%d shown=%d hidden=%d%s"),
				Sel.TotalValid, Sel.Shown, Sel.Hidden, *HeldText));
		}

		// The colour legend for candidate box sources, for the detail view only, since the source can only
		// be told apart by colour; it is reported as one line there.
		if (bAdvanced)
		{
			AddTextLine(TEXT("  {grey}box src: {cyan}Actual {green}PredictiveFree {magenta}PredictiveGuided"));
		}

		// The whip guide. Its colours are kept to one family, cyan, so they do not mix with the green and
		// red of contact success and failure.
		if (FS.bWhipActive && FS.Positions.Num() >= 2)
		{
			const int32 LastNode = FS.Positions.Num() - 1;
			// The guide curve, meaning the target points and the lines joining them, is shown by default,
			// since where the swing is heading is the point of it.
			for (int32 i = 0; i < FS.WhipGuideTargets.Num(); ++i)
			{
				AddPoint(FS.WhipGuideTargets[i], 10.0f, FColor::Cyan);
				if (i + 1 < FS.WhipGuideTargets.Num())
				{
					AddShape(FGameplayDebuggerShape::MakeSegment(FS.WhipGuideTargets[i], FS.WhipGuideTargets[i + 1], 1.0f, FColor::Cyan));
				}
			}
			// The basic Flight view marks only the boundary between guided and free nodes, because how far
			// the guide reaches is the point; per-node display moves to the detail view. The boundary node
			// is the last one whose fraction is at or below the guided end.
			const int32 BoundaryNode = FMath::Clamp(FMath::FloorToInt(FS.WhipGuidedEnd * LastNode), 0, LastNode);
			AddShape(FGameplayDebuggerShape::MakeBox(FS.Positions[BoundaryNode], FVector(5.0f), FColor::Cyan));

			if (bAdvanced)
			{
			// Per-node guided and free display, as a raw observation, kept to one colour family: cyan boxes
			// for guided nodes and pale cyan points for free ones.
				for (int32 i = 1; i <= LastNode; ++i)
				{
					const float Frac = static_cast<float>(i) / static_cast<float>(LastNode);
					if (Frac <= FS.WhipGuidedEnd)
					{
						AddShape(FGameplayDebuggerShape::MakeBox(FS.Positions[i], FVector(3.5f), FColor::Cyan));
					}
					else
					{
						AddPoint(FS.Positions[i], 8.0f, FColor(90, 170, 170));
					}
				}
				// The correction line from a node to its guide target, as a raw observation: which node is
				// being pulled to which target and by how much. Drawn in teal, within the cyan family.
				for (int32 i = 0; i < FS.WhipGuideTargets.Num(); ++i)
				{
					if (FS.WhipGuideNodeIndices.IsValidIndex(i) && FS.Positions.IsValidIndex(FS.WhipGuideNodeIndices[i]))
					{
						AddShape(FGameplayDebuggerShape::MakeSegment(FS.Positions[FS.WhipGuideNodeIndices[i]],
							FS.WhipGuideTargets[i], 1.0f, FColor(0, 180, 200)));
					}
				}
			}
		}
	}

	//~ Wrap: the wrapping path axis ----------------------------------------
	// The path axis ResolveWrappingAxis chose during Wrapping, drawn as a yellow line through the target
	// with a direction arrow. It diagnoses the wrapping path rather than collision shapes, so it belongs to
	// the wrap view. Throwing at the same pillar from several angles and checking that this line always
	// follows the long axis is what it is for. It appears during the Wrapping phase only.
	if (HasView(EView::Wrap) && S.bHasWrapAxis)
	{
		if (UWorld* World = Rope.GetWorld())
		{
			const FVector AxisDir = S.WrapAxisDirection.GetSafeNormal();
			const FVector AxisO = S.WrapAxisOrigin;
			// The axis length is proportional to the rope's scale, as the larger of 80 and six segment
			// lengths. A degenerate axis, meaning a zero vector, reports text only.
			const float AxisLen = FMath::Max(80.0f, S.WrapAxisSegmentLength * 6.0f);
			if (!AxisDir.IsNearlyZero())
			{
				constexpr uint8 FG = SDPG_Foreground;
				DrawDebugLine(World, AxisO - AxisDir * AxisLen, AxisO + AxisDir * AxisLen, FColor::Yellow, false, -1.0f, FG, 3.0f);
				DrawDebugDirectionalArrow(World, AxisO, AxisO + AxisDir * AxisLen, 16.0f, FColor::Yellow, false, -1.0f, FG, 3.0f);
			}
			// The direction of a valid axis is already shown by the yellow line and arrow above and is not
			// repeated as text. A degenerate axis, meaning a zero vector, cannot be drawn at all, so only
			// that fact is reported.
			if (AxisDir.IsNearlyZero())
			{
				AddTextLine(TEXT("  {red}wrapAxis degenerate{grey} (zero direction)"));
			}
		}
	}

	//~ Wrap: the binding result --------------------------------------------
	if (HasView(EView::Wrap) && S.bHasWrapped)
	{
		// Only the latch nodes are reported. The unique information in a wrap result is which nodes are
		// pinned to a bone; the positions of the others are already drawn in place by the tube mesh, and
		// per-node state is the job of the nodes view.
		// Drawing every node meant 23 of the default 24 duplicated the tube.
		// The box size is derived from the node collision radius. A fixed 4.5 cm meant nothing and made a
		// node read as though it had that volume; in reality a node is a point mass and the radius is how
		// far it is held off a surface.
		// It is the AABB of that radius, so the corners lie outside the corresponding sphere: it indicates
		// the scale rather than being an exact volume.
		const FVector LatchExtent(FMath::Max(S.NodeCollisionRadius, KINDA_SMALL_NUMBER));
		for (const int32 NodeIdx : S.LatchedNodes)
		{
			if (S.Positions.IsValidIndex(NodeIdx))
			{
				AddShape(FGameplayDebuggerShape::MakeBox(S.Positions[NodeIdx], LatchExtent, FColor::Yellow));
			}
		}

		AddTextLine(FString::Printf(TEXT("  {green}wrapped{white} bone=%s mesh=%s latched=%d"),
			*S.WrapBoneName.ToString(), *S.MeshName, S.Latched.Num()));
		// The tension, as the relative force from lambda. The threshold and the warning colour are reported
		// only in modes where the automatic release actually runs: GuaranteedWrap never looks at the
		// threshold, since only an explicit release is valid, so warning against it would be meaningless.
		if (!S.bAutoReleaseEnabled)
		{
			AddTextLine(FString::Printf(
				TEXT("    tension=%.0f  {grey}auto-release=disabled (%s — explicit release only)"),
				S.WrapTension, DebugResolveModeName(S.ResolveMode)));
		}
		else if (S.TensionReleaseForce > 0.0f)
		{
			const TCHAR* Color = (S.WrapTension > S.TensionReleaseForce) ? TEXT("{red}")
				: (S.WrapTension > S.TensionReleaseForce * 0.8f) ? TEXT("{yellow}") : TEXT("{white}");
			// The time spent above the threshold is reported too, because exceeding it has to persist for
			// the configured time before the rope lets go, which ignores spikes.
			AddTextLine(FString::Printf(TEXT("    tension=%s%.0f{white} / release=%.0f  {grey}(%.2f/%.2fs)"),
				Color, S.WrapTension, S.TensionReleaseForce, S.TensionOverTime, S.TensionReleaseTime));
		}
		else
		{
			AddTextLine(FString::Printf(TEXT("    tension=%.0f (release off)"), S.WrapTension));
		}

		// The pull direction, shown whenever the sample is valid, regardless of tension.
		// The teal line and point are the fractional aim leg, from the anchor to the smoothed aim position.
		// That is the input to the direction average and not the applied direction; the applied direction is
		// the green arrow below. The two only coincide in a settled state.
		// Foreground DrawDebug* is used because the anchor is inside the wrapped bone and AddShape's world
		// depth priority would bury it in the mesh.
		if (S.bPullValid)
		{
			if (UWorld* World = Rope.GetWorld())
			{
				constexpr uint8 FG = SDPG_Foreground;
				DrawDebugLine(World, S.PullPoint, S.PullAimPoint, FColor::Cyan, false, -1.0f, FG, 3.0f);
				DrawDebugPoint(World, S.PullAimPoint, 12.0f, FColor::Cyan, false, -1.0f, FG);
			}

			// Detail: the green arrow is the smoothed pull direction, which is what is actually applied this
			// frame. How far it diverges from the teal input is the lag of the direction smoothing. The
			// angle shown compares the raw integer aim against the smoothed one, which is a different pair
			// from teal against green, so use it only to tune the smoothing coefficient. An aim node number
			// that jumps between frames signals a direction jump.
			if (HasView(EView::Advanced))
			{
				if (UWorld* World = Rope.GetWorld())
				{
					// It shares an origin with the teal leg and nearly coincides with it in a settled state,
					// so it is drawn after it, since later is on top in the foreground, and at twice the
					// thickness: whichever is hidden when they overlap must not be the direction actually
					// applied.
					constexpr float DiagLen = 40.0f;
					DrawDebugDirectionalArrow(World, S.PullPoint, S.PullPoint + S.PullDirection * DiagLen,
						16.0f, FColor::Green, false, -1.0f, SDPG_Foreground, 6.0f);
				}
				const float JitterDeg = FMath::RadiansToDegrees(FMath::Acos(
					FMath::Clamp(static_cast<float>(FVector::DotProduct(S.PullDirRaw, S.PullDirection)), -1.0f, 1.0f)));
				AddTextLine(FString::Printf(TEXT("    {grey}pull-dir aim=node%d rawAim<->smooth=%.1f deg"),
					S.PullAimNode, JitterDeg));
			}
		}

		// Pull state; the sample is always produced. The basic line reports conclusions only: whether it is
		// pulling, through the tension; whether it is taut, which is the condition for applying active
		// pull; how far it has overshot and how close it is to letting go, through the tether and release
		// values; and whether there is input, through the active value.
		// The observations behind those conclusions, meaning the chain geometry, the direction vectors and
		// the internal lambda values, move to the detail line below.
		if (S.bPullValid)
		{
			// The distance release judges on the overshoot alone, independently of tension, so it can
			// approach or exceed the limit even while slack with zero tension. Both branches therefore
			// report the same values for the release state and the active pull input: reporting only one of
			// them would hide an imminent release while the rope is slack.
			const bool bDistanceReleaseLive = S.bAutoReleaseEnabled && S.DistanceReleaseSlack > 0.0f;
			// Warn by colour as the overshoot approaches or passes the limit: yellow from 80 percent and red
			// beyond it.
			const TCHAR* OvershootColor = TEXT("{white}");
			if (bDistanceReleaseLive)
			{
				OvershootColor = (S.TetherOvershoot > S.DistanceReleaseSlack) ? TEXT("{red}")
					: (S.TetherOvershoot > S.DistanceReleaseSlack * 0.8f) ? TEXT("{yellow}") : TEXT("{white}");
			}
			// With a threshold of 0, or under GuaranteedWrap, it reads as off rather than as a number, since
			// a release value of 0 would read as a threshold of zero.
			const FString ReleaseText = bDistanceReleaseLive
				? FString::Printf(TEXT("release=%.0f"), S.DistanceReleaseSlack)
				: FString(TEXT("release=off"));
			// The active value is the request SetActivePull stored. It is only applied after passing the taut
			// gate, and the two bypass layers, so whether it passed is reported alongside: the number alone
			// would make a blocked frame and an applied one look identical.
			const FString ActiveText = (S.ActivePullForce > 0.0f)
				? FString::Printf(TEXT(" active=%.0f%s"), S.ActivePullForce,
					S.bActivePullApplied ? TEXT("") : TEXT("{yellow} blocked"))
				: FString();

			if (S.PullTension > KINDA_SMALL_NUMBER)
			{
				AddTextLine(FString::Printf(
					TEXT("    {orange}pull{white} tension=%.0f taut=%s{white} tether=%s%.0fcm{white}(%s)%s"),
					S.PullTension, S.bPullTaut ? TEXT("{green}Y") : TEXT("{grey}N"),
					OvershootColor, S.TetherOvershoot, *ReleaseText, *ActiveText));
			}
			else
			{
				// Zero tension means slack. If that is unexpected, the chain geometry on the detail line
				// explains why.
				AddTextLine(FString::Printf(
					TEXT("    {grey}pull slack (tension 0) taut=%s{grey} tether=%s%.0fcm{grey}(%s)%s"),
					S.bPullTaut ? TEXT("{green}Y") : TEXT("{grey}N"),
					OvershootColor, S.TetherOvershoot, *ReleaseText, *ActiveText));
			}

			// Detail: the observations behind the conclusions. The chain values are the precondition gate for
			// traction, comparing the sum of the corner-to-corner leg chords against the rest length; a
			// minimum tension of 0 means the tension is not reaching the hand; and the sag is the largest sag
			// on any leg.
			if (HasView(EView::Advanced))
			{
				// The tether tension is coloured against its limit: yellow from 80 percent and red on
				// reaching it, which means lambda is being clamped. Sitting at the limit is designed
				// behaviour, so it is shown by colour alone with no separate warning text.
				const TCHAR* TensionColor = TEXT("{cyan}");
				if (S.MaxTetherTension > 0.0f)
				{
					TensionColor = (S.TetherTension >= S.MaxTetherTension * 0.999f) ? TEXT("{red}")
						: (S.TetherTension > S.MaxTetherTension * 0.8f) ? TEXT("{yellow}") : TEXT("{cyan}");
				}
				AddTextLine(FString::Printf(
					TEXT("      {grey}chain=%s{grey}(%.0f/%.0fcm, minT=%.0f, sag=%.0f) tether %sT=%.0f{grey}/%.0f"),
					S.bChainTaut ? TEXT("{green}Y") : TEXT("{grey}N"),
					S.TautChordLen, S.FreeRestLen, S.MinFreeTension, S.MaxLegSag,
					TensionColor, S.TetherTension, S.MaxTetherTension));
				AddTextLine(FString::Printf(
					TEXT("      {grey}length-constraint=%s attempted=%.0fcm/s rejected=%.2fcm"),
					*S.ConstraintBackend, S.AttemptedOutwardSpeed, S.AttemptedViolation));
			}
		}
		else
		{
			AddTextLine(TEXT("    {grey}pull n/a (no hand-side anchor)"));
		}

		// The latches are folded per bone. Latch nodes are mostly contiguous, so a row per node would
		// merely repeat the same bone name, and a wide wrap would produce tens or hundreds of rows and be
		// truncated in a panel that does not scroll. One line per bone, giving the count and the node
		// ranges, fits because few bones are ever wrapped, and it reveals how many latches landed on which
		// bone and over what range. The positions are drawn by the yellow 3D boxes and the total is
		// reported by the wrapped line's latch count. With a single latch this duplicates that line
		// entirely, so it is reported only when there are several, and then only in the detail view.
		if (HasView(EView::Advanced) && S.Latched.Num() >= 2)
		{
			// The order of first appearance, by first latch node, is preserved so it reads in node index
			// order.
			TArray<FName> BoneOrder;
			TMap<FName, TArray<int32>> ByBone;
			for (const FRopeLatchNode& Latch : S.Latched)
			{
				TArray<int32>& Idx = ByBone.FindOrAdd(Latch.Bone);
				if (Idx.Num() == 0)
				{
					BoneOrder.Add(Latch.Bone);
				}
				Idx.Add(Latch.NodeIndex);
			}
			for (const FName& LatchBone : BoneOrder)
			{
				TArray<int32>& Idx = ByBone[LatchBone];
			// If one bone has more than one wrapped island the indices can be out of order, so they are
			// sorted defensively.
				Idx.Sort();
				AddTextLine(FString::Printf(TEXT("      {grey}bone=%s count=%d nodes=%s"),
					*LatchBone.ToString(), Idx.Num(), *NodeRangeString(Idx)));
			}
		}
	}

	//~ colliders --------------------------------------------------------
	// Colliders are drawn with foreground DrawDebug* rather than AddShape, because AddShape hardcodes its
	// depth priority to world in FGameplayDebuggerShape::Draw and is buried behind meshes. It is not a
	// limitation of shape expressiveness, since MakeCapsule and MakeBox also take a rotation. The trade-off
	// is that DrawDebug* is not replicated to a remote client, which is an acceptable choice while the
	// supported scope is standalone and local play, and one to revisit if multiplayer is supported.
	if (HasView(EView::Colliders))
	{
		if (UWorld* World = Rope.GetWorld())
		{
			constexpr uint8 FG = SDPG_Foreground;
			constexpr float LineThick = 1.5f;

			// The colour follows three categories of "can this actually be wrapped", in ColorPass below.
			// The static wrap target count is not every wrappable target but the number of static opt-in
			// targets served by URopeWrapTargetComponent.
			int32 StaticWrapTargetCount = 0;
			for (const FRopeDebugCollider& C : S.Colliders)
			{
				if (C.bWrapTarget) { ++StaticWrapTargetCount; }
			}
			AddTextLine(FString::Printf(
				TEXT("  {grey}colliders n=%d [{green}wrappable{grey}/{red}rejected{grey}/{cyan}collision-only{grey}]  staticWrapTargets=%d"),
				S.Colliders.Num(),
				StaticWrapTargetCount));

			// Drawn in three passes so green, meaning wrappable, always ends up on top: a wrap target
			// serving its own push-out shape at the same location, as a physics-body prop does, would
			// otherwise hide it, so cyan is laid down first, then red, then green, since later is on top in
			// the foreground.
			for (int32 DrawPass = 0; DrawPass < 3; ++DrawPass)
			for (const FRopeDebugCollider& C : S.Colliders)
			{
				// Cyan is static world geometry, push-out only and excluded from detection; red takes part in
				// detection but is refused by the gate; and green has valid attribution and passed
				// CanWrapTarget. A static prop served by URopeWrapTargetComponent reports IsWorldStatic() as
				// false, so it is green or red rather than cyan.
				const int32 ColorPass = C.bWorldStatic ? 0 : (C.bWrapAllowed ? 2 : 1);
				if (ColorPass != DrawPass) { continue; }
				const FColor Color = (ColorPass == 2) ? FColor::Green
					: (ColorPass == 1) ? FColor::Red : FColor::Cyan;
				switch (C.Shape)
				{
				case ERopeDebugColliderShape::Capsule:
				{
					// The real collision volume, as a sphyl. Its half height includes the hemispherical caps,
					// so it is half the segment plus the radius.
					// A degenerate one, where the endpoints coincide and it is a sphere, has no direction and
					// is drawn as a sphere.
					const FVector Axis = C.B - C.A;
					const float SegLen = static_cast<float>(Axis.Size());
					if (SegLen > KINDA_SMALL_NUMBER)
					{
						const FVector Center = (C.A + C.B) * 0.5f;
						const FQuat Rot = FRotationMatrix::MakeFromZ(Axis).ToQuat();
						DrawDebugCapsule(World, Center, SegLen * 0.5f + C.Radius, C.Radius, Rot, Color, false, -1.0f, FG, LineThick);
					}
					else
					{
						DrawDebugSphere(World, C.A, C.Radius, 12, Color, false, -1.0f, FG, LineThick);
					}
					break;
				}
				case ERopeDebugColliderShape::Box:
					// An oriented box, whose half extents are the box extent.
					DrawDebugBox(World, C.Center, C.HalfExtents, C.Rot, Color, false, -1.0f, FG, LineThick);
					break;
				case ERopeDebugColliderShape::Convex:
					// The hull wireframe: one line per consecutive pair of edge endpoints.
					for (int32 e = 0; e + 1 < C.ConvexEdges.Num(); e += 2)
					{
						DrawDebugLine(World, C.ConvexEdges[e], C.ConvexEdges[e + 1], Color, false, -1.0f, FG, LineThick);
					}
					break;
				case ERopeDebugColliderShape::Bounds:
				default:
					if (C.Bounds.IsValid)
					{
						DrawDebugBox(World, C.Bounds.GetCenter(), C.Bounds.GetExtent(), Color, false, -1.0f, FG, LineThick);
					}
					break;
				}
			}

		}
	}
}

#endif // WITH_GAMEPLAY_DEBUGGER
