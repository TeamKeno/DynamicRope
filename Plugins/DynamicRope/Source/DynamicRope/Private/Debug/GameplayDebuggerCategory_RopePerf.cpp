// Copyright 2026 TeamKeno. All Rights Reserved.

#include "Debug/GameplayDebuggerCategory_RopePerf.h"

#if WITH_GAMEPLAY_DEBUGGER

#include "RopeComponent.h"
#include "Core/RopeLifecycleTypes.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/Pawn.h"
#include "Camera/PlayerCameraManager.h"
#include "DrawDebugHelpers.h"
// The registered rope list, which is the single source for this screen.
#include "Subsystem/RopeSimSubsystem.h"

namespace
{
	const TCHAR* PerfPhaseName(ERopePhase Phase)
	{
		switch (Phase)
		{
		case ERopePhase::Free:        return TEXT("Free");
		case ERopePhase::Flight:      return TEXT("Flight");
		case ERopePhase::Contacting:  return TEXT("Contacting");
		case ERopePhase::Wrapping:    return TEXT("Wrapping");
		case ERopePhase::Wrapped:     return TEXT("Wrapped");
		case ERopePhase::GuidedThrow: return TEXT("GuidedThrow");
		case ERopePhase::Releasing:   return TEXT("Releasing");
		case ERopePhase::Loaded:        return TEXT("Loaded");
		default:                      return TEXT("?");
		}
	}

	// The live snapshot each per-rope line needs, gathered up front so the lines can be sorted.
	struct FPerfRow
	{
		ERopePhase Phase = ERopePhase::Free;
		int32 Nodes = 0;
		bool bGpuStepped = false;   // Stepped this frame with resident GPU state.
		bool bCpuSolved = false;    // Solved, but not on the GPU, meaning the CPU fallback.
		bool bSleeping = false;     // Asleep because it is Free and at rest.
		bool bGdf = false;          // Configured to use the world global distance field.
		float LodScale = 1.0f;
		float Distance = -1.0f;     // From the camera to the anchor, in centimetres, or minus one with no camera.
		FVector Anchor = FVector::ZeroVector;
		// With no centreline yet, meaning before seeding or with zero nodes, the anchor is a meaningless zero vector.
		// Drawing the marker anyway stamps a numbered label at the world origin for a rope that is not there, and
		// several of them stack up, sending the reader to the wrong place.
		bool bHasAnchor = false;
	};

	// The point size is in screen pixels. Near markers stay large enough to see, while over the range from 5 m to
	// 100 m the size drops from six to three pixels so that the dots do not cover the labels when several ropes
	// converge at a distance. The number label is always shown, regardless of distance.
	float MarkerPixelSize(float Distance)
	{
		if (Distance < 0.0f)
		{
			return 6.0f;
		}

		return FMath::GetMappedRangeValueClamped(
			FVector2D(500.0f, 10000.0f), FVector2D(6.0f, 3.0f), Distance);
	}
}

FGameplayDebuggerCategory_RopePerf::FGameplayDebuggerCategory_RopePerf()
{
	// The whole world is drawn with no debug actor required.
	bShowOnlyWithDebugActor = false;

	// Resetting on tick, the default, clears it every collection tick, so CollectData does not reset separately.
	SetDataPackReplication<FRepData>(&DataPack);
}

void FGameplayDebuggerCategory_RopePerf::FRepData::Serialize(FArchive& Ar)
{
	int32 NumMarkers = Markers.Num();
	Ar << NumMarkers;
	if (Ar.IsLoading())
	{
		Markers.SetNum(NumMarkers);
	}
	for (FMarker& Marker : Markers)
	{
		Ar << Marker.Location;
		Ar << Marker.Color;
		Ar << Marker.PixelSize;
		Ar << Marker.DisplayIndex;
	}
}

TSharedRef<FGameplayDebuggerCategory> FGameplayDebuggerCategory_RopePerf::MakeInstance()
{
	return MakeShareable(new FGameplayDebuggerCategory_RopePerf());
}

void FGameplayDebuggerCategory_RopePerf::DrawData(
	APlayerController* OwnerPC, FGameplayDebuggerCanvasContext& CanvasContext)
{
	FGameplayDebuggerCategory::DrawData(OwnerPC, CanvasContext);

	UWorld* World = CanvasContext.World.Get();
	if (!World)
	{
		return;
	}

	for (const FRepData::FMarker& Marker : DataPack.Markers)
	{
		DrawDebugPoint(World, Marker.Location, Marker.PixelSize, Marker.Color,
			false, -1.0f, SDPG_Foreground);
	// The numbers correlating the list with the world are all shown, whether or not they overlap, so they can be identified at a distance.
		DrawDebugString(World, Marker.Location + FVector(0.0, 0.0, 10.0),
			FString::Printf(TEXT("#%d"), Marker.DisplayIndex), nullptr, Marker.Color,
			0.0f, true, 1.0f);
	}
}

void FGameplayDebuggerCategory_RopePerf::CollectData(APlayerController* OwnerPC, AActor* DebugActor)
{
	const UWorld* World = OwnerPC ? OwnerPC->GetWorld() : (DebugActor ? DebugActor->GetWorld() : nullptr);
	if (!World)
	{
		AddTextLine(TEXT("{grey}no world"));
		return;
	}

	// The camera position, used to compute the distance: the camera manager first, then the pawn. With neither, the distance is omitted.
	bool bHasCam = false;
	FVector CamLoc = FVector::ZeroVector;
	if (OwnerPC)
	{
		if (const APlayerCameraManager* CM = OwnerPC->PlayerCameraManager)
		{
			CamLoc = CM->GetCameraLocation();
			bHasCam = true;
		}
		else if (const APawn* Pawn = OwnerPC->GetPawn())
		{
			CamLoc = Pawn->GetActorLocation();
			bHasCam = true;
		}
	}

	// Walks the subsystem's registered rope list to collect live values; see GetRegisteredRopes below for why.
	TArray<FPerfRow> Rows;
	int32 NumGpu = 0, NumCpu = 0, NumSleeping = 0, NumGdf = 0;
	int64 TotalParticles = 0;
	// It reads the subsystem's registered list because what this screen counts is not the ropes in the world but the
	// ropes this subsystem actually drives: an unregistered rope is never ticked and costs nothing, which makes it
	// the wrong denominator for a performance screen.
	// A full UObject scan would be expensive regardless of the rope count, and a screen opened to measure performance
	// would weigh down the frame itself and distort what it is measuring.
	const URopeSimSubsystem* SimSub = URopeSimSubsystem::Get(World);
	if (!SimSub)
	{
		AddTextLine(TEXT("{grey}no rope sim subsystem"));
		return;
	}
	for (const TObjectPtr<URopeComponent>& RopePtr : SimSub->GetRegisteredRopes())
	{
		// Entries destroyed and awaiting collection can be present, since invalid ones are cleaned up only at a tick frame boundary.
		const URopeComponent* Rope = RopePtr.Get();
		if (!IsValid(Rope))
		{
			continue;
		}

		FPerfRow Row;
		Row.Phase = Rope->GetPhase();
		Row.Nodes = Rope->GetNodeCount();
		Row.bGpuStepped = Rope->IsGpuSteppedThisFrame();
		Row.bCpuSolved = !Row.bGpuStepped && Rope->WasSolvedThisFrame();
		Row.bSleeping = Rope->IsSleeping();
		Row.bGdf = Rope->bUseWorldGDF;
		Row.LodScale = Rope->GetSolverLODScale();

		const TArray<FVector>& Points = Rope->GetCenterlinePositions();
		if (Points.Num() > 0)
		{
			Row.Anchor = Points[0];
			Row.bHasAnchor = true;
			if (bHasCam)
			{
				Row.Distance = static_cast<float>(FVector::Dist(CamLoc, Row.Anchor));
			}
		}

		TotalParticles += Row.Nodes;
		if (Row.bGpuStepped) { ++NumGpu; }
		else if (Row.bCpuSolved) { ++NumCpu; }
		if (Row.bSleeping) { ++NumSleeping; }
		if (Row.bGdf) { ++NumGdf; }
		Rows.Add(Row);
	}

	const int32 NumRopes = Rows.Num();
	const int32 NumIdle = FMath::Max(0, NumRopes - NumGpu - NumCpu);

	// The summary at the top. The registered count is the number of ropes registered with the subsystem and driven
	// this frame rather than the number placed in the world: a rope before registration, meaning just spawned and
	// before BeginPlay, or one whose registration failed is never ticked and does not appear here.
	// Its breakdown into GPU, CPU and idle uses the same definitions as Active in 'stat DynamicRope'.
	AddTextLine(FString::Printf(
		TEXT("{white}Rope Perf (world){grey}  registered=%d  {green}gpu=%d {red}cpu=%d {grey}idle=%d  {cyan}sleeping=%d"),
		NumRopes, NumGpu, NumCpu, NumIdle, NumSleeping));
	AddTextLine(FString::Printf(
		TEXT("{grey}particles=%lld  gdf-enabled=%d%s"),
		static_cast<long long>(TotalParticles), NumGdf,
		bHasCam ? TEXT("") : TEXT("  (no camera — distances hidden)")));

	if (NumRopes == 0)
	{
		AddTextLine(TEXT("{grey}no registered ropes in world"));
		return;
	}

	// Sorting puts the ropes being solved, which is where the cost is, at the top, and within those orders by node count descending, so that the expensive ropes survive being cut off at the limit.
	Rows.Sort([](const FPerfRow& A, const FPerfRow& B)
	{
		const bool bActiveA = A.bGpuStepped || A.bCpuSolved;
		const bool bActiveB = B.bGpuStepped || B.bCpuSolved;
		if (bActiveA != bActiveB)
		{
			return bActiveA;
		}
		return A.Nodes > B.Nodes;
	});

	// One line per rope, plus a phase-coloured dot at the anchor with a '#i' label, so a rope can be located in the world and correlated with the list.
	const int32 MaxRows = FMath::Min(40, Rows.Num());
	for (int32 i = 0; i < MaxRows; ++i)
	{
		const FPerfRow& R = Rows[i];

		// The solve path token states what was actually done first, on the same basis as the GPU and CPU counters
		// above, so that the summary and the rows cannot disagree. SLEEP is used as the path only on a frame where
		// nothing at all happened: a rope can go to sleep in Finalize after solving, meaning it both solved and
		// slept, and that case is marked separately as asleep below.
		const TCHAR* Solve = R.bGpuStepped ? TEXT("{green}gpu")
			: (R.bCpuSolved ? TEXT("{red}cpu")
			: (R.bSleeping ? TEXT("{cyan}SLEEP") : TEXT("{grey}idle")));
		// A rope that solved this frame but then went to sleep. It is counted by the sleeping counter as well, so the
		// state is appended to keep a row counted as GPU or CPU from appearing on screen as SLEEP alone.
		const FString Asleep = (R.bSleeping && (R.bGpuStepped || R.bCpuSolved))
			? FString(TEXT(" {cyan}asleep")) : FString();

		const FString Lod = (R.LodScale < 0.999f)
			? FString::Printf(TEXT(" {cyan}lod=x%.2f"), R.LodScale) : FString();
		const FString Dist = (R.Distance >= 0.0f)
			? FString::Printf(TEXT(" {grey}d=%.0f"), R.Distance) : FString();

		AddTextLine(FString::Printf(
			TEXT("{yellow}#%d {white}%s{white} n=%d %s%s{white}%s %s%s"),
			i + 1, PerfPhaseName(R.Phase), R.Nodes, Solve, *Asleep, *Lod,
			R.bGdf ? TEXT("{green}gdf") : TEXT("{grey}gdf-off"), *Dist));

		// The anchor marker, correlating the world with the list: cyan for asleep, white for solving and grey for idle.
		// A rope with no centreline has nowhere to draw: plotting the zero vector would leave a ghost marker at the world origin.
		if (R.bHasAnchor)
		{
			const FColor MarkerColor = R.bSleeping ? FColor::Cyan
				: ((R.bGpuStepped || R.bCpuSolved) ? FColor::White : FColor(140, 140, 140));
			FRepData::FMarker& Marker = DataPack.Markers.AddDefaulted_GetRef();
			Marker.Location = R.Anchor;
			Marker.Color = MarkerColor;
			Marker.PixelSize = MarkerPixelSize(R.Distance);
			Marker.DisplayIndex = i + 1;
		}
	}
	if (Rows.Num() > MaxRows)
	{
		AddTextLine(FString::Printf(TEXT("{grey}... %d more"), Rows.Num() - MaxRows));
	}
}

#endif // WITH_GAMEPLAY_DEBUGGER
