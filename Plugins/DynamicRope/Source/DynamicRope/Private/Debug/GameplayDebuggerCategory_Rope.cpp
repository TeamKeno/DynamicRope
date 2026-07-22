// Copyright Epic Games, Inc. All Rights Reserved.

#include "Debug/GameplayDebuggerCategory_Rope.h"

#if WITH_GAMEPLAY_DEBUGGER

#include "RopeComponent.h"
#include "Gameplay/RopeWielderComponent.h"
#include "Debug/RopeDebugSnapshot.h"
#include "Subsystem/RopeDebugSubsystem.h"
// RopeFlightDebug::SelectCandidateBoxes — 후보 박스 선택(순수 함수, 단위 테스트 있음)
#include "Logic/RopeFlightDebugSelection.h"
#include "GameFramework/Actor.h"
// RopeGPU::TubeRingBucket / MaxTubeRings — GPU 튜브 경로/버킷 진단
#include "RopeTubeBuilder.h"
// RopeGPU::IsRuntimeSupported — 프록시의 GPU 튜브 게이트와 같은 런타임 판정(RHI + SM5)
#include "RopeGPUSolver.h"
// DrawDebug*(SDPG_Foreground) — 콜라이더 전경 오버레이(에디터 셀렉션처럼 위에 그림)
#include "DrawDebugHelpers.h"

namespace
{
	// GPU 튜브 적격성을 프록시와 같은 수식(IsRuntimeSupported() && NumRings<=MaxTubeRings)으로 게임 스레드에서
	// 재현한다 — 프록시의 bUseGpuTube를 크로스스레드로 읽지 않는다. 프록시는 생성 시점에 판정을 굳히므로 런타임
	// 속성 변경 후에는 갈릴 수 있고, 그래서 라벨이 tube-eligible이다. NumNodes는 프록시와 같은 소스인
	// NumParticles(설정값)를 받는다. bAdvanced가 꺼지면 경로와 CPU 폴백 사유만 낸다(사유는 조치 가능하고,
	// 버킷·링 수는 디스패치 내부 수치라 상세에서만).
	FString TubeDiagString(int32 NumNodes, int32 WantedSubdiv, bool bAdvanced)
	{
		if (!RopeGPU::IsRuntimeSupported())
		{
			// 렌더 가능 RHI가 없거나 SM5 미만 — 링 수와 무관하게 프록시가 CPU BuildTube로 간다.
			return FString(TEXT("{red}cpu{grey}(no gpu runtime)"));
		}
		const int32 Nodes = FMath::Max(2, NumNodes);
		// 프록시와 같은 헬퍼로 Subdiv를 링 상한에 맞춰 하향 → 실제 사용 버킷/링을 표시.
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

	// 이번 프레임 이 로프가 밟은 솔브 경로. 서브시스템이 솔브 프레임과 override-only 프레임을 똑같이 GPU에
	// 실으므로 bGpuStepped 하나로는 판정할 수 없다. 판정 순서는 "실제로 한 일" 먼저 — bSleeping은 프레임
	// 끝에 확정되므로 먼저 보면 이 프레임 실제로 밟은 SOLVE를 SLEEP이 덮는다.
	const TCHAR* SolvePathToken(bool bSleeping, bool bSolved, bool bGpuStepped, bool bLogicOverride)
	{
		if (bGpuStepped)
		{
			return bSolved ? TEXT("{green}GPU_SOLVE") : TEXT("{green}GPU_OVERRIDE");
		}
		if (bSolved)
		{
			// GPU 상주 대상이 아니어서 CPU로 푼 프레임(노드 수 초과·RHI 없음 등).
			return TEXT("{red}CPU_SOLVE");
		}
		if (bLogicOverride)
		{
			// 솔브는 없지만 로직이 위치를 갱신한 프레임.
			return TEXT("{yellow}CPU_OVERRIDE");
		}
		// 아무 일도 없었다 — 잠들어서인지(SLEEP) 그냥 할 일이 없어서인지(IDLE)를 가른다.
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

	// 도달 모드 표시명(자동 해제 문구에 함께 낸다). 열거자 이름 그대로 낸다 — 화면에서 읽은 값으로
	// 코드를 바로 찾을 수 있어야 한다.
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

	// 오름차순 노드 인덱스를 연속 구간으로 압축한 문자열("12-16,23-24"). 개수만으로는 알 수 없는 "어디부터
	// 어디까지 / 한 덩어리인가"를 한 줄로 읽게 한다. 입력이 오름차순이 아니면 구간이 잘게 쪼개진다.
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

	// 근접 노드는 캡처 루프가 노드 순서대로 채워 이미 오름차순 — 인덱스만 뽑아 범위 압축에 넘긴다.
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

	// 후보 출처 표시명(요약/범례 텍스트용) — 색과 짝을 이룬다.
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

	// 하위 보기 토글 키. 카테고리가 활성일 때 입력된다. cvar(r.DynamicRope.Debug.*) 대체.
	// 키는 FName 리터럴로 지정한다 — EKeys/FKey는 InputCore 모듈 심볼이라 링크 의존을 피한다.
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

	// 점 표시는 셰이프가 아니라 자체 데이터 팩으로 복제한다(사유는 헤더 FRepData 주석).
	// ResetOnTick(기본) — 수집 틱마다 비워지므로 CollectData에서 따로 Reset하지 않는다.
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

	// 점은 전경으로 — 노드 상당수가 캐릭터/벽 메시 안이나 표면에 붙어 있어 SDPG_World면 묻힌다.
	// DrawData는 보는 쪽에서 매 프레임 돌므로, 수집 간격이 프레임보다 길어도 점이 깜빡이지 않는다.
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
	// Aim/Advanced는 수집이 없다 — 전자는 Wielder 라이브 읽기, 후자는 이미 모은 것의 표시 상세도다.
	return Mask;
}

void FGameplayDebuggerCategory_Rope::CollectData(APlayerController* OwnerPC, AActor* DebugActor)
{
	if (!DebugActor)
	{
		AddTextLine(TEXT("{grey}no debug actor"));
		return;
	}

	// 대상 액터 + 캡처 범위를 등록 → sim tick(GT)이 다음 프레임 이 액터의 로프를, 켜진 보기만 캡처한다.
	// 보기를 토글하면 반영이 한 프레임 늦는다(대상 등록 자체가 원래 그렇다).
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

	// 조준은 로프가 아니라 Wielder 소유 — 로프 순회와 별개로 액터에서 한 번 찾아 그린다.
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

		// phase/centerline 등 상시 정보는 라이브에서, 진단 오버레이는 스냅샷(있으면)에서.
		const FRopeDebugSnapshot* Snap = Dbg ? Dbg->GetSnapshot(Rope) : nullptr;
		// flight 오버레이 소스: 라이브가 Flight면 그걸 쓰고, 아니면 hold된 마지막 flight(0.5초 창)를 잔류
		// 소스로 넘긴다(캡처 결정 직후 Wrapping으로 넘어가도 잠시 보이게).
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

	// 질의는 하지 않는다 — Wielder가 조준이 성립하는 동안 매 틱 스윕해 남긴 샘플을 읽기만 한다.
	// 조준이 꺼져 있으면 샘플이 비어 있고(RayLength=0), 그릴 ray 자체가 없다.

	// FullSimulation처럼 조준 ray를 아예 쓰지 않는 모드에서는 이 섹션 자체를 내지 않는다. 모드가 바뀌지
	// 않는 한 영영 같은 문구라 진단 가치가 없고, 상세 보기에 둬도 화면만 차지한다.
	if (!Wielder.UsesAimRay())
	{
		return;
	}
	// 조준 모드는 맞지만 지금 던질 수 없는 phase — GuaranteedWrap은 Loaded(장전)에서만 조준이 성립한다.
	// 진입하면 해소되는 일시 상태라 "왜 조준이 안 잡히나"의 답이 된다. [J]를 켰는데 ray가 안 보이는
	// 이유 그 자체이므로 상세 보기와 무관하게 낸다.
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

	// 색 규약은 조준 HUD와 동일하다: 감김 가능=green / 걸렸지만 감김 불가=red / 미스=cyan.
	const bool bAnyHit = Aim.bHasTarget || Aim.bBlocked;
	const FColor MainColor = Aim.bHasTarget ? FColor::Green : (Aim.bBlocked ? FColor::Red : FColor::Cyan);
	const FVector RayDir = Aim.RayDirection.GetSafeNormal();
	const FVector RayStart = Aim.RayOrigin;
	const FVector RayEnd = RayStart + RayDir * Aim.RayLength;
	const FVector RayStop = bAnyHit ? Aim.HitWorldPos : RayEnd;

	// collider와 같은 이유로 AddShape 대신 DrawDebug*(전경): AddShape는 depth priority가 SDPG_World로
	// 하드코딩돼 지오메트리에 가린다(형상 표현의 문제가 아니다 — MakeCapsule은 회전 인자를 받는다).
	// 수명은 다음 수집까지만 남게 짧게 준다.
	constexpr float LifeTime = 0.05f;
	constexpr uint8 FG = SDPG_Foreground;
	// 캡슐 치수는 실제 QuerySwept에 넘어간 길이·반경 그대로 — 조준이 검사하는 부피를 눈으로 확인한다.
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
		// hit 너머 남은 구간 — 조준이 어디까지 뻗을 수 있었는지.
		DrawDebugLine(World, RayStop, RayEnd, FColor(96, 0, 0), false, LifeTime, FG, 1.0f);
		// 본 이름은 3D 라벨로 띄우지 않는다 — 아래 aim 텍스트 줄이 같은 이름을 상시 내보내므로 중복.
		DrawDebugSphere(World, Aim.HitWorldPos, 8.0f, 12, FColor::Yellow, false, LifeTime, FG, 2.0f);
	}

	// ToString()의 임시를 로컬에 잡아둔다 — const TCHAR*로 받으면 다음 줄에서 이미 dangling이다.
	const FString BoneText = Aim.Bone.IsNone() ? FString(TEXT("-")) : Aim.Bone.ToString();
	if (Aim.bHasTarget)
	{
		AddTextLine(FString::Printf(TEXT("  {white}aim: {green}%s{white} dist=%.0f radius=%.1f"),
			*BoneText, Aim.Distance, Aim.QueryRadius));
	}
	else if (Aim.bBlocked)
	{
		// ray는 맞았지만 감을 수 없다 — 본 없음/SourceMesh 없음/CanWrapTarget 거부.
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
	// 화면 한 장은 하나의 시간 기준만 쓴다 — 스냅샷이 있으면 헤더·centerline·오버레이가 모두 그 스냅샷을
	// 읽는다. 헤더만 라이브로 두면 같은 노드가 두 시점에 겹쳐 그려져 시뮬 떨림/latch 불안정처럼 보인다.
	// 스냅샷이 아직 없는 첫 프레임에만 라이브로 헤더를 내고 (live) 라벨을 붙인다.
	const ERopePhase PhaseEnd = Snap ? Snap->Phase : Rope.GetPhase();
	const ERopePhase PhaseStart = Snap ? Snap->PhaseAtFrameStart : PhaseEnd;
	const TArray<FVector>& Points = Snap ? Snap->Positions : Rope.GetCenterlinePositions();
	// 헤더 nodes= 개수. 기본(aim만) 상태에선 Positions를 복사하지 않으므로 Points.Num()이 0이다 —
	// 스냅샷에는 항상 담기는 NodeCount 스칼라를 쓴다(라이브 첫 프레임에는 Points.Num()).
	const int32 NodeCount = Snap ? Snap->NodeCount : Points.Num();
	const FName Bone = Snap ? Snap->WrapBoneName : Rope.GetWrappedBoneName();
	const bool bSleeping = Snap ? Snap->bSleeping : Rope.IsSleeping();
	const float LODScale = Snap ? Snap->LodScale : Rope.GetSolverLODScale();

	// 프레임 안에서 전이했으면 시작→종료를 함께 낸다. 이 조합이 곧 "무엇 때문에 넘어갔나"의 단서다
	// (예: Flight→Contacting 프레임의 flight 오버레이 = 전이를 일으킨 관측).
	const FString PhaseText = (PhaseStart != PhaseEnd)
		? FString::Printf(TEXT("%s{grey}→{white}%s"), DebugPhaseName(PhaseStart), DebugPhaseName(PhaseEnd))
		: FString(DebugPhaseName(PhaseEnd));

	// 스냅샷이 이번 프레임보다 몇 프레임 뒤처졌나. 0이면(이번 프레임 캡처) 아예 내지 않으므로, 이 토큰이
	// 보인다는 것 자체가 지연 상태다 — 커질수록 sim이 캡처를 못 따라오는 것(대상 해제/일시정지). 반대
	// 분기 (live — diag pending)과 짝을 이루도록 풀어쓴 괄호체로 낸다.
	const uint64 AgeFrames = (Snap && GFrameCounter > Snap->FrameStamp) ? (GFrameCounter - Snap->FrameStamp) : 0;
	const FString AgeText = Snap
		? ((AgeFrames > 0)
			? FString::Printf(TEXT("  {grey}(%llu frame%s behind)"),
				static_cast<unsigned long long>(AgeFrames), (AgeFrames == 1) ? TEXT("") : TEXT("s"))
			: FString())
		: FString(TEXT("  {grey}(live — diag pending)"));

	// 정체성: 컴포넌트 이름 + 소유 액터. 인덱스는 수집 순서라 프레임마다 바뀔 수 있어 로프를 특정하지
	// 못한다. 스냅샷이 없는 첫 프레임에는 라이브 컴포넌트에서 같은 값을 읽는다.
	const FString NameText = Snap ? Snap->ComponentName : Rope.GetName();
	const AActor* LiveOwner = Rope.GetOwner();
	const FString OwnerText = Snap ? Snap->OwnerActorName : (LiveOwner ? LiveOwner->GetName() : TEXT("None"));

	// 스케일링 상태: 슬립(솔브 스킵) 여부 + 거리 LOD iteration 배율(1 미만이면 감쇠 중).
	// 모드는 상시 표기다 — 모드마다 성립 계약과 유효한 설정이 통째로 달라 나머지 줄의 해석 전제가 된다.
	AddTextLine(FString::Printf(
		TEXT("{yellow}%s{grey}@%s{white} phase=%s mode=%s nodes=%d wrapBone=%s%s%s%s"),
		*NameText, *OwnerText, *PhaseText,
		DebugResolveModeName(Snap ? Snap->ResolveMode : Rope.ResolveMode), NodeCount,
		Bone.IsNone() ? TEXT("-") : *Bone.ToString(),
		bSleeping ? TEXT("  {cyan}asleep") : TEXT(""),
		LODScale < 0.999f ? *FString::Printf(TEXT("  {cyan}lod=x%.2f"), LODScale) : TEXT(""),
		*AgeText));

	// 솔브 경로(6종 토큰) + 튜브 적격성(프록시 실제 상태가 아닌 재계산 추정치 — TubeDiagString 주석 참조).
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
		// 노드 점만 찍는다 — 연결 세그먼트는 튜브 메시가 이미 보여주므로 중복이고, phase 색은 위 헤더
		// 줄의 phase=... 텍스트가 낸다. 노드 단위 상태 구분은 [U]flight / [I]wrap 오버레이 담당.
		for (const FVector& Point : Points)
		{
			AddPoint(Point, 6.0f, FColor::Yellow);
		}
		// latch 노드 강조(인덱스와 위치가 같은 스냅샷에서 온다). 크기는 일반 노드와 같게 두고 색으로만
		// 구분한다 — 뒤에 그려 노란 점을 정확히 덮으므로, 노드 하나가 빨갛게 바뀐 것으로 읽힌다.
		// 키우면 노드 간격보다 커져 이웃 노드까지 가리는 덩어리가 된다.
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

	// 이하 진단 오버레이는 transient 캡처 데이터 — 스냅샷이 있을 때만.
	if (!Snap)
	{
		return;
	}
	const FRopeDebugSnapshot& S = *Snap;

	//~ nodes: 근접 재질의 ------------------------------------------------
	// 노드가 어느 면에 어느 법선으로 붙었나. collider 형상이 아니라 **노드 상태**라 [P]nodes 소속이다.
	// 이름이 proximity인 이유: 실제 solver 접촉이 아니라 질의 반경을 CollisionRadius+4cm로 넓혀 다시
	// 질의한 결과라, 닿지 않은 근처 노드도 잡힌다(GPU는 접촉을 리드백하지 않아 CPU로 재질의한다).
	if (HasView(EView::Nodes) && S.NodeProximity.Num() > 0)
	{
		if (UWorld* World = Rope.GetWorld())
		{
			constexpr uint8 FG = SDPG_Foreground;
			// 닿은 노드가 몇 번부터 어디까지인지는 구간 문자열로 낸다 — 노드마다 3D 라벨을 띄우면 간격이
			// SegmentLength라 서로 겹쳐 읽을 수 없다.
			AddTextLine(FString::Printf(
				TEXT("  {grey}proximity n=%d nodes=%s {grey}(requery r+%.0fcm, not solver contacts)"),
				S.NodeProximity.Num(), *ProximityRangeString(S.NodeProximity), S.ProximityQueryMargin));
			for (int32 i = 0; i < S.NodeProximity.Num(); ++i)
			{
				const FRopeNodeProximityDebug& NP = S.NodeProximity[i];
				// 정적 월드=마젠타, 그 외(스켈레탈 본/랩 대상)=주황.
				const FColor NColor = NP.bWorldStatic ? FColor(255, 0, 255) : FColor(255, 128, 0);
				const FVector Tip = NP.Position + NP.Normal * 15.0f;
				// 화살표는 접촉 바깥 법선 그대로 — 축 정렬이 아니라 실제 방향이다. 노드 수만큼 나오는
				// 표시라 굵기는 가늘게 둔다 — 굵으면 인접 노드끼리 뭉쳐 방향을 읽을 수 없다.
				DrawDebugDirectionalArrow(World, NP.Position, Tip, 5.0f, NColor, false, -1.0f, FG, 1.5f);
				// 3D 라벨은 **첫 구간의 시작 노드** 하나만 — 위 nodes=... 텍스트와 3D를 잇는 기준점 역할이다
				// (나머지 번호는 그 구간 문자열이 이미 낸다). 배열이 노드 인덱스 순이라 i==0은 "가장 작은
				// 번호"일 뿐이고, 캡처 후보나 dominant 대상, 감김 시작점이라는 뜻이 아니다 — 정적 월드
				// 접촉(마젠타)일 수도, 감김이 없는 페이즈일 수도 있다. 구간이 여럿이면 첫 구간만 라벨된다.
				if (i == 0)
				{
					DrawDebugString(World, Tip, FString::Printf(TEXT("n%d"), NP.NodeIndex),
						nullptr, NColor, 0.0f, true, 1.0f);
				}
			}
		}
	}

	//~ flight -----------------------------------------------------------
	// flight 오버레이의 소스: 라이브가 Flight면 라이브, 아니면 hold된 마지막 flight(0.5초 창)를 쓴다.
	// hold면 결정 순간의 값(WorldPoint/SurfacePoint/Positions/MeshKeys — 전부 값·키라 frozen 안전)을 그대로
	// 그려, 캡처 직후 Wrapping으로 넘어가도 잠시 정지한 채 보이게 한다. 헤더/wrap/노드 등 나머지는 라이브 S.
	const FRopeDebugSnapshot* FlightSnap = (Snap && Snap->bHasFlight) ? Snap : HeldFlight;
	if (HasView(EView::Flight) && FlightSnap && FlightSnap->bHasFlight)
	{
		const FRopeDebugSnapshot& FS = *FlightSnap;
		const bool bHeld = (FlightSnap != Snap);
		// hold 중이면 요약에 붙일 경과 라벨.
		const FString HeldText = bHeld
			? FString::Printf(TEXT("  {yellow}(held %.1fs)"), HeldFlightAgeSeconds) : FString();

		// 이 노드의 접촉이 유효 후보로 이어졌는가. 같은 노드가 한 프레임에 여러 대상에 닿을 수 있으므로
		// 노드 인덱스만으로는 판정할 수 없고, 대상 식별 계약대로 (NodeIndex, Bone, Mesh) 3자를 모두 본다.
		// 후보 수가 작아 선형 탐색으로 충분하다.
		auto HasValidCandidateFor = [&FS](const FRopeFlightNodeDebug& Node)
		{
			for (int32 i = 0; i < FS.Candidates.Num(); ++i)
			{
				const FRopeContactCandidate& Candidate = FS.Candidates[i];
				if (!Candidate.bValid || Candidate.NodeIndex != Node.NodeIndex || Candidate.Bone != Node.Contact.Bone)
				{
					continue;
				}
				// 키 배열은 Candidates와 1:1. 없으면(구 스냅샷) 본까지만 맞은 것으로 본다.
				if (!FS.CandidateMeshKeys.IsValidIndex(i) || FS.CandidateMeshKeys[i] == Node.ContactMeshKey)
				{
					return true;
				}
			}
			return false;
		};

		// 기본 [U]은 "무엇을 포착하려는가 + 접촉 성공/실패"만, [U]+[K]은 후보 선정·Whip의 원시 관측치까지.
		// 이 게이트 하나로 flight 오버레이의 기본/상세를 가른다.
		const bool bAdvanced = HasView(EView::Advanced);

		for (const FRopeFlightNodeDebug& Node : FS.NodeDebug)
		{
			if (bAdvanced)
			{
				// 노드 이동선(prev→pos): "그 사이 어디를 지났나". 노드 수만큼 늘어 상세 보기 전용.
				AddShape(FGameplayDebuggerShape::MakeSegment(Node.PrevPosition, Node.Position, 1.0f, FColor::White));
				// near-body: 접촉이 없는 노드만 — 접촉점(초록/빨강)이 이미 상태를 낸 노드에 노란 점을 겹치면
				// 덮여서 안 보인다.
				if (Node.bNearBody && !Node.Contact.bHit)
				{
					AddPoint(Node.Position, 8.0f, FColor::Yellow);
				}
			}

			if (Node.Contact.bHit)
			{
				// 접촉 성공(유효 후보로 이어짐)=초록 / 탈락=빨강. 이 판정이 기본 [U]의 핵심이라 상시 표시.
				const FColor HitColor = HasValidCandidateFor(Node) ? FColor::Green : FColor::Red;
				AddPoint(Node.Contact.SurfacePoint, 10.0f, HitColor);
				if (bAdvanced)
				{
					// 접촉 법선(원시 관측치)은 상세 보기 전용. 길이는 노드 스케일에 비례(고정 22cm 대신).
					const float NormalLen = FMath::Clamp(FS.NodeCollisionRadius * 3.0f, 12.0f, 40.0f);
					AddShape(FGameplayDebuggerShape::MakeSegment(Node.Contact.SurfacePoint,
						Node.Contact.SurfacePoint + Node.Contact.Normal.GetSafeNormal() * NormalLen, 1.0f, FColor::Blue));
				}
			}
		}

		// 후보 박스 선택. 기본 [U]은 포착 대상(tracker (Mesh,Bone)) 대표 1개만, [U]+[K]은 대표 + penetration
		// 상위 일반 후보로 최대 5개까지. 대표는 top-N 밖이어도 항상 포함한다(predictive 대표는 penetration이
		// 낮아 정렬 꼴찌이기 쉬운데, 그게 "곧 무엇에 걸리려 하나"라 가장 보고 싶은 값이다). 선택 규칙은
		// 순수 함수라 단위 테스트로 고정한다(RopeFlightDebugSelectionTests). bAdvanced는 위에서 정의.
		const RopeFlightDebug::FCandidateSelection Sel = RopeFlightDebug::SelectCandidateBoxes(
			FS.Candidates, FS.CandidateMeshKeys, FS.TrackerBone, FS.TrackerMeshKey,
			bAdvanced ? 5 : 1, /*bFillWithGeneral=*/bAdvanced);

		for (const int32 i : Sel.BoxIndices)
		{
			const FRopeContactCandidate& Candidate = FS.Candidates[i];
			const FColor SourceColor = CandidateSourceColor(Candidate.Source);
			if (i == Sel.CaptureTargetIndex)
			{
				// 포착 대상: 흰 외곽 박스 + 출처색 내부 박스 두 겹으로 "이게 대표"임을 크기/외곽으로 드러낸다.
				AddShape(FGameplayDebuggerShape::MakeBox(Candidate.WorldPoint, FVector(4.5f), FColor::White));
				AddShape(FGameplayDebuggerShape::MakeBox(Candidate.WorldPoint, FVector(2.8f), SourceColor));
			}
			else
			{
				AddShape(FGameplayDebuggerShape::MakeBox(Candidate.WorldPoint, FVector(3.5f), SourceColor));
			}
		}

		// 요약 한 줄: 무엇을 포착하려는가(capture-target=Mesh:Bone) + 출처 + 후보 총수/표시/숨김. hold면 (held Xs).
		// mesh 이름은 dangling 가능한 raw 포인터 대신 키로 안전 해석(죽었으면 ?).
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

		// 후보 박스 출처 색 범례(상세 보기 전용) — 색만으로 출처를 구분해야 하므로 [K]에서 한 줄로 낸다.
		if (bAdvanced)
		{
			AddTextLine(TEXT("  {grey}box src: {cyan}Actual {green}PredictiveFree {magenta}PredictiveGuided"));
		}

		// whip 가이드. 색은 한 계열(cyan)로 통일한다 — 접촉 성공/실패의 초록/빨강과 섞이지 않게.
		if (FS.bWhipActive && FS.Positions.Num() >= 2)
		{
			const int32 LastNode = FS.Positions.Num() - 1;
			// 가이드 커브(타깃 점 + 이음선)는 기본으로 — 스윙이 어디로 향하는지가 요지다.
			for (int32 i = 0; i < FS.WhipGuideTargets.Num(); ++i)
			{
				AddPoint(FS.WhipGuideTargets[i], 10.0f, FColor::Cyan);
				if (i + 1 < FS.WhipGuideTargets.Num())
				{
					AddShape(FGameplayDebuggerShape::MakeSegment(FS.WhipGuideTargets[i], FS.WhipGuideTargets[i + 1], 1.0f, FColor::Cyan));
				}
			}
			// 기본 [U]은 guided/free 경계 하나만 마커로 — "어디까지 가이드가 끄는가"가 요지고, 노드별 표시는
			// 상세 보기로 뺀다. 경계 노드 = Frac이 WhipGuidedEnd 이하인 마지막 노드.
			const int32 BoundaryNode = FMath::Clamp(FMath::FloorToInt(FS.WhipGuidedEnd * LastNode), 0, LastNode);
			AddShape(FGameplayDebuggerShape::MakeBox(FS.Positions[BoundaryNode], FVector(5.0f), FColor::Cyan));

			if (bAdvanced)
			{
				// guided/free 노드별 표시(원시) — 한 색 계열: guided=cyan 박스, free=옅은 cyan 점.
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
				// 노드→가이드 타깃 보정선(원시): 어느 노드를 어느 타깃으로 얼마나 끄는가. cyan 계열(teal)로.
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

	//~ wrap: 감김 경로 축 -------------------------------------------------
	// Wrapping에서 ResolveWrappingAxis가 정한 경로 축(대상을 관통하는 노란 선 + 방향 화살표). 충돌 형상이
	// 아니라 **감김 경로 진단**이라 [I]wrap 소속이다. 같은 기둥을 여러 각도로 던져 이 선이 늘 장축을
	// 따르는지 눈으로 확인한다. Wrapping 페이즈에서만 뜬다.
	if (HasView(EView::Wrap) && S.bHasWrapAxis)
	{
		if (UWorld* World = Rope.GetWorld())
		{
			const FVector AxisDir = S.WrapAxisDirection.GetSafeNormal();
			const FVector AxisO = S.WrapAxisOrigin;
			// 축 길이는 로프 스케일에 비례(max(80, SegmentLength×6)). 축이 퇴화(0벡터)면 텍스트만 낸다.
			const float AxisLen = FMath::Max(80.0f, S.WrapAxisSegmentLength * 6.0f);
			if (!AxisDir.IsNearlyZero())
			{
				constexpr uint8 FG = SDPG_Foreground;
				DrawDebugLine(World, AxisO - AxisDir * AxisLen, AxisO + AxisDir * AxisLen, FColor::Yellow, false, -1.0f, FG, 3.0f);
				DrawDebugDirectionalArrow(World, AxisO, AxisO + AxisDir * AxisLen, 16.0f, FColor::Yellow, false, -1.0f, FG, 3.0f);
			}
			// 정상 축의 방향은 위 노란 선/화살표가 이미 보여주므로 문자열로 반복하지 않는다. 축이
			// 퇴화(0벡터)하면 선 자체를 그릴 수 없으니 그 사실만 낸다.
			if (AxisDir.IsNearlyZero())
			{
				AddTextLine(TEXT("  {red}wrapAxis degenerate{grey} (zero direction)"));
			}
		}
	}

	//~ wrap: 결착 결과 ----------------------------------------------------
	if (HasView(EView::Wrap) && S.bHasWrapped)
	{
		// latch 노드만 낸다. 감김 결과에서 고유한 정보는 "어느 노드가 본에 고정됐나"뿐이고, 나머지 노드의
		// 위치는 튜브 메시가 이미 그 자리에 그린다(노드 단위 상태가 필요하면 [P] nodes 담당).
		// 전부 찍던 종전 루프는 기본 24노드에서 23개가 튜브와 겹치는 중복이었다.
		// 박스 크기는 노드 충돌 반지름에서 유도한다 — 종전 고정 4.5cm는 아무것도 뜻하지 않아 노드가
		// 그만한 볼륨을 갖는 것처럼 읽혔다. 실제로는 노드가 질점이고 반지름은 표면에서 띄우는 거리다.
		// 반지름의 AABB이므로 모서리는 그 구보다 밖에 있다(정확한 볼륨이 아니라 그 크기의 지표).
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
		// 장력(λ/h² 상대 힘). 임계치와 경고색은 자동 해제가 실제로 도는 모드에서만 낸다 — GuaranteedWrap은
		// 임계치를 보지 않으므로(명시 해제만 유효) 임계 대비 경고가 의미를 갖지 않는다.
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
			// 임계 초과 지속 시간도 함께 — 임계를 넘어도 TensionReleaseTime 동안 지속돼야 풀린다(스파이크 무시).
			AddTextLine(FString::Printf(TEXT("    tension=%s%.0f{white} / release=%.0f  {grey}(%.2f/%.2fs)"),
				Color, S.WrapTension, S.TensionReleaseForce, S.TensionOverTime, S.TensionReleaseTime));
		}
		else
		{
			AddTextLine(FString::Printf(TEXT("    tension=%.0f (release off)"), S.WrapTension));
		}

		// Pull 방향(장력 유무와 무관하게 bPullValid면 항상).
		// 청록 선/점 = fractional aim leg — 앵커 → 스무딩된 조준 위치(AimPos). 방향 EMA의 **입력**이지 인가
		// 방향이 아니다(인가되는 SmoothedPullDir은 아래 초록 화살표). 정상 상태에서만 둘이 겹친다.
		// 전경 DrawDebug*를 쓰는 이유: 앵커가 감긴 본 안이라 AddShape의 SDPG_World로는 메시에 묻힌다.
		if (S.bPullValid)
		{
			if (UWorld* World = Rope.GetWorld())
			{
				constexpr uint8 FG = SDPG_Foreground;
				DrawDebugLine(World, S.PullPoint, S.PullAimPoint, FColor::Cyan, false, -1.0f, FG, 3.0f);
				DrawDebugPoint(World, S.PullAimPoint, 12.0f, FColor::Cyan, false, -1.0f, FG);
			}

			// 상세: 초록 화살표 = SmoothedPullDir = 이번 프레임 실제 인가 방향. 청록(EMA 입력)과 벌어지는
			// 정도가 방향 스무딩 지연이다. 표시되는 각도차는 raw(정수 조준) ↔ smoothed 비교로 청록↔초록과는
			// 다른 쌍이니 EMA 계수 조정에만 쓴다. 조준 노드 번호는 프레임마다 튀면 방향 점프의 신호다.
			if (HasView(EView::Advanced))
			{
				if (UWorld* World = Rope.GetWorld())
				{
					// 청록 다리와 원점을 공유하고 정상 상태에서 거의 겹치므로 청록보다 뒤에(전경은 나중이
					// 위) 두 배 굵기로 그린다 — 겹칠 때 가려지는 쪽이 실제 인가 방향이면 안 된다.
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

		// Pull 상태(샘플은 항상 산출). 기본 줄은 **결론**만 낸다 — 당기고 있는가(tension), 팽팽한가(taut,
		// 능동 Pull 인가 조건), 얼마나 초과했고 놓칠 만한가(tether/release), 입력이 걸렸는가(active).
		// 그 결론을 만든 관측치(chain 기하·방향 벡터·λ 내부값)는 아래 상세 줄로 내린다.
		if (S.bPullValid)
		{
			// 거리 release는 **장력과 무관하게** 초과분(TetherOvershoot)만 보고 판정하므로, 슬랙(tension 0)
			// 상태에서도 한계에 근접하거나 넘을 수 있다. 그래서 release 상태와 능동 Pull 입력은 두 분기가
			// 같은 값을 낸다 — 한쪽만 내면 슬랙일 때 "곧 놓친다"를 놓친다.
			const bool bDistanceReleaseLive = S.bAutoReleaseEnabled && S.DistanceReleaseSlack > 0.0f;
			// 초과분이 한계에 근접/초과할 때 색으로 경고(노랑 80%+, 빨강 초과).
			const TCHAR* OvershootColor = TEXT("{white}");
			if (bDistanceReleaseLive)
			{
				OvershootColor = (S.TetherOvershoot > S.DistanceReleaseSlack) ? TEXT("{red}")
					: (S.TetherOvershoot > S.DistanceReleaseSlack * 0.8f) ? TEXT("{yellow}") : TEXT("{white}");
			}
			// 임계가 0이거나 GuaranteedWrap이면 숫자가 아니라 off — release=0은 "임계가 0"으로 읽힌다.
			const FString ReleaseText = bDistanceReleaseLive
				? FString::Printf(TEXT("release=%.0f"), S.DistanceReleaseSlack)
				: FString(TEXT("release=off"));
			// active는 SetActivePull이 저장한 **요청값**이다. 팽팽 게이트(+우회 2층)를 통과해야 실제로 인가되므로
			// 통과 여부를 함께 낸다 — 숫자만 내면 막힌 프레임과 인가된 프레임이 같아 보인다.
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
				// 장력 0 = 슬랙. 이 상태가 의외라면 chain 기하(상세 줄)가 이유를 말해준다.
				AddTextLine(FString::Printf(
					TEXT("    {grey}pull slack (tension 0) taut=%s{grey} tether=%s%.0fcm{grey}(%s)%s"),
					S.bPullTaut ? TEXT("{green}Y") : TEXT("{grey}N"),
					OvershootColor, S.TetherOvershoot, *ReleaseText, *ActiveText));
			}

			// 상세: 결론을 만든 관측치. chain은 견인의 선행 게이트(코너-다리 chord 합 vs rest 길이),
			// minT=0이면 장력이 손까지 전달되지 않는다는 뜻, sag는 다리별 최대 처짐이다.
			if (HasView(EView::Advanced))
			{
				// 테더 장력 T: 상한 대비 색(80%+ 노랑, 도달 빨강 = λ 클램프 중). 상한에 붙는 것 자체는 설계된
				// 동작이라 별도 경고 문구 없이 색으로만 드러낸다.
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
			}
		}
		else
		{
			AddTextLine(TEXT("    {grey}pull n/a (no hand-side anchor)"));
		}

		// latch를 bone 단위로 접어 낸다 — latch 노드는 대부분 연속이라 per-node 행은 같은 본 이름을
		// 반복할 뿐이고, 넓게 감기면 수십~수백 행이 되어 스크롤 없는 패널에서 잘린다. 본당 한 줄로
		// 개수 + 노드 구간을 내면(감는 본 수는 적어 잘림이 없다) "어느 본에 몇 개가 어느 구간에 걸렸나"가
		// 드러난다. 위치는 3D 노란 박스가, 총개수는 wrapped 줄의 latched=N이 낸다. latch가 하나면 그
		// 줄과 완전 중복이라 여럿일 때만, 그것도 상세 보기에서만 낸다.
		if (HasView(EView::Advanced) && S.Latched.Num() >= 2)
		{
			// 등장 순서(첫 latch 노드 순)를 보존해 노드 인덱스 흐름대로 읽히게 한다.
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
				// 같은 본에 감김 섬이 둘 이상이면 인덱스가 뒤섞일 수 있어 방어적으로 정렬한다.
				Idx.Sort();
				AddTextLine(FString::Printf(TEXT("      {grey}bone=%s count=%d nodes=%s"),
					*LatchBone.ToString(), Idx.Num(), *NodeRangeString(Idx)));
			}
		}
	}

	//~ colliders --------------------------------------------------------
	// 콜라이더는 AddShape 대신 DrawDebug*(SDPG_Foreground)로 그린다 — AddShape는 depth priority가 SDPG_World로
	// 하드코딩돼(FGameplayDebuggerShape::Draw) 메시에 가린다. 형상 표현력 문제는 아니다(MakeCapsule/MakeBox도
	// 회전 인자를 받는다). 대신 DrawDebug*는 원격 클라이언트로 복제되지 않는다 — 지원 범위가 Standalone/로컬이라
	// 성립하는 선택이며 멀티플레이 지원 시 재검토 대상.
	if (HasView(EView::Colliders))
	{
		if (UWorld* World = Rope.GetWorld())
		{
			constexpr uint8 FG = SDPG_Foreground;
			constexpr float LineThick = 1.5f;

			// 색은 "실제로 감길 수 있는가" 3범주(아래 ColorPass). staticWrapTargets는 감김 가능 대상 전체가
			// 아니라 URopeWrapTargetComponent가 서빙하는 정적 opt-in 대상 수다.
			int32 StaticWrapTargetCount = 0;
			for (const FRopeDebugCollider& C : S.Colliders)
			{
				if (C.bWrapTarget) { ++StaticWrapTargetCount; }
			}
			AddTextLine(FString::Printf(
				TEXT("  {grey}colliders n=%d [{green}wrappable{grey}/{red}rejected{grey}/{cyan}collision-only{grey}]  staticWrapTargets=%d"),
				S.Colliders.Num(),
				StaticWrapTargetCount));

			// 3패스로 그려 초록(감김 가능)이 항상 위에 오게 한다: 랩 대상이 자기 push-out 셰이프를 같은 위치에
			// 서빙하면(PhysicsBody 프롭) 겹쳐서 가려지므로, cyan → red → green 순으로 깔고 덮는다(전경은 나중이 위).
			for (int32 DrawPass = 0; DrawPass < 3; ++DrawPass)
			for (const FRopeDebugCollider& C : S.Colliders)
			{
				// cyan = 정적 월드(push-out 전용, 감지 비참여) / red = 감지에는 들어오지만 게이트가 거부 /
				// green = 귀속 유효 + CanWrapTarget 통과. URopeWrapTargetComponent의 정적 프롭은
				// IsWorldStatic()=false라 cyan이 아니라 green/red로 갈린다.
				const int32 ColorPass = C.bWorldStatic ? 0 : (C.bWrapAllowed ? 2 : 1);
				if (ColorPass != DrawPass) { continue; }
				const FColor Color = (ColorPass == 2) ? FColor::Green
					: (ColorPass == 1) ? FColor::Red : FColor::Cyan;
				switch (C.Shape)
				{
				case ERopeDebugColliderShape::Capsule:
				{
					// 실제 충돌 볼륨(sphyl) 그대로. HalfHeight는 반구 포함 전체 절반이라 세그먼트 절반 + Radius.
					// 축퇴(A==B, 구 셰이프)는 방향이 없으므로 구.
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
					// 회전 OBB. HalfExtents가 Box extent.
					DrawDebugBox(World, C.Center, C.HalfExtents, C.Rot, Color, false, -1.0f, FG, LineThick);
					break;
				case ERopeDebugColliderShape::Convex:
					// 헐 와이어프레임: 엣지 끝점 쌍(연속 2개)마다 라인.
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
