// Copyright Epic Games, Inc. All Rights Reserved.

#include "Debug/GameplayDebuggerCategory_Rope.h"

#if WITH_GAMEPLAY_DEBUGGER

#include "RopeComponent.h"
#include "Gameplay/RopeWielderComponent.h"
#include "Debug/RopeDebugSnapshot.h"
#include "Subsystem/RopeDebugSubsystem.h"
#include "GameFramework/Actor.h"
// RopeGPU::TubeRingBucket / MaxTubeRings — GPU 튜브 경로/버킷 진단
#include "RopeTubeBuilder.h"
// RopeGPU::IsRuntimeSupported — 프록시의 GPU 튜브 게이트와 같은 런타임 판정(RHI + SM5)
#include "RopeGPUSolver.h"
// DrawDebug*(SDPG_Foreground) — 콜라이더 전경 오버레이(에디터 셀렉션처럼 위에 그림)
#include "DrawDebugHelpers.h"

namespace
{
	// GPU 튜브 적격성을 노드 수 + Subdiv(로프의 TubeSmoothingSubdiv)로 도출한다. 프록시의 bUseGpuTube를
	// 읽은 값이 아니라 같은 수식을 게임 스레드에서 재현한 추정치다(렌더 스레드 상태를 크로스스레드로 읽지
	// 않는다) — 프록시는 생성 시점에 판정을 굳히므로 런타임 속성 변경 후에는 갈릴 수 있고, 그래서 화면
	// 라벨도 tube-eligible이다. 판정식은 프록시와 동일: IsRuntimeSupported() && NumRings<=MaxTubeRings,
	// NumRings=(NumNodes-1)*Subdiv+1.
	// NumNodes는 프록시와 같은 소스인 NumParticles(설정값)를 받는다 — 라이브 노드 수를 넘기면 시드 전이나
	// 노드 수 변경 중에 프록시와 다른 링 수가 나온다.
	FString TubeDiagString(int32 NumNodes, int32 WantedSubdiv)
	{
		if (!RopeGPU::IsRuntimeSupported())
		{
			// 렌더 가능 RHI가 없거나 SM5 미만 — 링 수와 무관하게 프록시가 CPU BuildTube로 간다.
			return FString(TEXT("{red}cpu{grey}(no gpu runtime)"));
		}
		int32 Subdiv = FMath::Clamp(WantedSubdiv, 1, 8);
		const int32 Nodes = FMath::Max(2, NumNodes);
		// 프록시(RopeComputeTubeSubdiv)와 동일하게 Subdiv를 링 상한에 맞춰 자동 하향 → 실제 사용 버킷/링을 표시.
		const int32 MaxForGpu = (Nodes > 2) ? FMath::Max(1, (RopeGPU::MaxTubeRings() - 1) / (Nodes - 1)) : Subdiv;
		Subdiv = FMath::Min(Subdiv, MaxForGpu);
		const int32 NumRings = (Nodes - 1) * Subdiv + 1;
		const int32 Bucket = RopeGPU::TubeRingBucket(NumRings);
		if (Bucket > 0)
		{
			return FString::Printf(TEXT("{green}gpu{grey}(bucket %d, rings %d)"), Bucket, NumRings);
		}
		return FString::Printf(TEXT("{red}cpu{grey}(rings %d > %d)"), NumRings, RopeGPU::MaxTubeRings());
	}

	// 이번 프레임 이 로프가 밟은 솔브 경로. bGpuStepped 하나로는 판정할 수 없다 — 서브시스템이 솔브
	// 프레임과 override-only 프레임(Wrapping/Releasing/GuidedThrow)을 똑같이 GPU에 실으므로
	// (TryBuildResidentStep) GPU=true가 "물리 솔브 중"을 뜻하지 않는다.
	const TCHAR* SolvePathToken(bool bSleeping, bool bSolved, bool bGpuStepped, bool bLogicOverride)
	{
		if (bSleeping)
		{
			// 슬립은 솔브 자체가 없다 — 다른 어떤 상태보다 먼저 본다.
			return TEXT("{cyan}SLEEP");
		}
		if (bGpuStepped)
		{
			return bSolved ? TEXT("{green}GPU_SOLVE") : TEXT("{green}GPU_OVERRIDE");
		}
		if (bSolved)
		{
			// GPU 상주 대상이 아니어서 CPU로 푼 프레임(노드 수 초과·RHI 없음 등).
			return TEXT("{red}CPU_SOLVE");
		}
		// 솔브도 GPU 디스패치도 없지만 로직이 위치를 갱신한 프레임 vs 아무것도 안 한 프레임.
		return bLogicOverride ? TEXT("{yellow}CPU_OVERRIDE") : TEXT("{grey}IDLE");
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
		case ERopePhase::Reel:       return TEXT("Reel");
		default:                     return TEXT("?");
		}
	}

	// 도달 모드 표시명(자동 해제 문구에 함께 낸다).
	const TCHAR* DebugResolveModeName(ERopeWrapResolveMode Mode)
	{
		switch (Mode)
		{
		case ERopeWrapResolveMode::FullSimulation: return TEXT("①FullSimulation");
		case ERopeWrapResolveMode::AssistedJudged: return TEXT("②Assisted");
		case ERopeWrapResolveMode::GuaranteedWrap: return TEXT("③Guaranteed");
		default:                                   return TEXT("?");
		}
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
}

FGameplayDebuggerCategory_Rope::FGameplayDebuggerCategory_Rope()
{
	bShowOnlyWithDebugActor = false;

	// 하위 보기 토글 키. 카테고리가 활성일 때 입력된다. cvar(r.DynamicRope.Debug.*) 대체.
	// 키는 FName 리터럴로 지정한다 — EKeys/FKey는 InputCore 모듈 심볼이라 링크 의존을 피한다.
	const FGameplayDebuggerInputHandlerConfig CenterlineCfg(TEXT("ToggleCenterline"), TEXT("P"));
	const FGameplayDebuggerInputHandlerConfig FlightCfg(TEXT("ToggleFlight"), TEXT("U"));
	const FGameplayDebuggerInputHandlerConfig WrappedCfg(TEXT("ToggleWrapped"), TEXT("I"));
	const FGameplayDebuggerInputHandlerConfig CollidersCfg(TEXT("ToggleColliders"), TEXT("O"));
	const FGameplayDebuggerInputHandlerConfig AimCfg(TEXT("ToggleAim"), TEXT("J"));
	BindKeyPress(CenterlineCfg, this, &FGameplayDebuggerCategory_Rope::OnToggleCenterline);
	BindKeyPress(FlightCfg, this, &FGameplayDebuggerCategory_Rope::OnToggleFlight);
	BindKeyPress(WrappedCfg, this, &FGameplayDebuggerCategory_Rope::OnToggleWrapped);
	BindKeyPress(CollidersCfg, this, &FGameplayDebuggerCategory_Rope::OnToggleColliders);
	BindKeyPress(AimCfg, this, &FGameplayDebuggerCategory_Rope::OnToggleAim);
}

TSharedRef<FGameplayDebuggerCategory> FGameplayDebuggerCategory_Rope::MakeInstance()
{
	return MakeShareable(new FGameplayDebuggerCategory_Rope());
}

void FGameplayDebuggerCategory_Rope::OnToggleCenterline() { ViewMask ^= static_cast<uint8>(EView::Centerline); }
void FGameplayDebuggerCategory_Rope::OnToggleFlight()    { ViewMask ^= static_cast<uint8>(EView::Flight); }
void FGameplayDebuggerCategory_Rope::OnToggleWrapped()   { ViewMask ^= static_cast<uint8>(EView::Wrapped); }
void FGameplayDebuggerCategory_Rope::OnToggleColliders() { ViewMask ^= static_cast<uint8>(EView::Colliders); }
void FGameplayDebuggerCategory_Rope::OnToggleAim()       { ViewMask ^= static_cast<uint8>(EView::Aim); }

void FGameplayDebuggerCategory_Rope::CollectData(APlayerController* OwnerPC, AActor* DebugActor)
{
	if (!DebugActor)
	{
		AddTextLine(TEXT("{grey}no debug actor"));
		return;
	}

	// 대상 액터를 등록 → sim tick(GT)이 다음 프레임 이 액터의 로프만 캡처한다.
	URopeDebugSubsystem* Dbg = URopeDebugSubsystem::Get(DebugActor->GetWorld());
	if (Dbg)
	{
		Dbg->SetTarget(DebugActor);
	}

	auto OnOff = [](bool b) { return b ? TEXT("{green}on") : TEXT("{grey}off"); };
	AddTextLine(FString::Printf(
		TEXT("{white}views  [P]centerline=%s{white} [U]flight=%s{white} [I]wrapped=%s{white} [O]colliders=%s{white} [J]aim=%s"),
		OnOff(HasView(EView::Centerline)), OnOff(HasView(EView::Flight)), OnOff(HasView(EView::Wrapped)),
		OnOff(HasView(EView::Colliders)), OnOff(HasView(EView::Aim))));

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
		DrawRope(Count, *Rope, Snap);
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
	if (!Wielder.UsesAimRay())
	{
		AddTextLine(TEXT("  {white}aim: {grey}not an aim ray mode (①FullSimulation)"));
		return;
	}
	if (!Wielder.IsAimActive())
	{
		// 조준 모드는 맞지만 지금 던질 수 없는 phase — ③는 Reel(장전)에서만 조준이 성립한다.
		AddTextLine(TEXT("  {white}aim: {grey}inactive — ③ aims from Reel only"));
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

void FGameplayDebuggerCategory_Rope::DrawRope(int32 Index, const URopeComponent& Rope, const FRopeDebugSnapshot* Snap)
{
	// 화면 한 장은 하나의 시간 기준만 쓴다 — 스냅샷이 있으면 헤더·centerline·오버레이가 모두 그 스냅샷을
	// 읽는다. 헤더만 라이브로 두면 같은 노드가 두 시점에 겹쳐 그려져 시뮬 떨림/latch 불안정처럼 보인다.
	// 스냅샷이 아직 없는 첫 프레임에만 라이브로 헤더를 내고 (live) 라벨을 붙인다.
	const ERopePhase PhaseEnd = Snap ? Snap->Phase : Rope.GetPhase();
	const ERopePhase PhaseStart = Snap ? Snap->PhaseAtFrameStart : PhaseEnd;
	const TArray<FVector>& Points = Snap ? Snap->Positions : Rope.GetCenterlinePositions();
	const FName Bone = Snap ? Snap->WrapBoneName : Rope.GetWrappedBoneName();
	const bool bSleeping = Snap ? Snap->bSleeping : Rope.IsSleeping();
	const float LODScale = Snap ? Snap->LodScale : Rope.GetSolverLODScale();

	// 프레임 안에서 전이했으면 시작→종료를 함께 낸다. 이 조합이 곧 "무엇 때문에 넘어갔나"의 단서다
	// (예: Flight→Contacting 프레임의 flight 오버레이 = 전이를 일으킨 관측).
	const FString PhaseText = (PhaseStart != PhaseEnd)
		? FString::Printf(TEXT("%s{grey}→{white}%s"), DebugPhaseName(PhaseStart), DebugPhaseName(PhaseEnd))
		: FString(DebugPhaseName(PhaseEnd));

	// 스냅샷 나이(프레임). 0이면 이번 프레임 것, 커지면 sim이 캡처를 못 낸 것이다(대상 해제/일시정지).
	const FString AgeText = Snap
		? ((GFrameCounter > Snap->FrameStamp)
			? FString::Printf(TEXT("  {grey}age=%lluf"), static_cast<unsigned long long>(GFrameCounter - Snap->FrameStamp))
			: FString())
		: FString(TEXT("  {grey}(live — diag pending)"));

	// 스케일링 상태: 슬립(솔브 스킵) 여부 + 거리 LOD iteration 배율(1 미만이면 감쇠 중).
	AddTextLine(FString::Printf(
		TEXT("{yellow}Rope #%d{white} phase=%s nodes=%d wrapBone=%s%s%s%s"),
		Index, *PhaseText, Points.Num(),
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
			Snap ? Snap->TubeSmoothingSubdiv : Rope.TubeSmoothingSubdiv)));

	//~ centerline -------------------------------------------------------
	if (HasView(EView::Centerline))
	{
		// 노드 점만 찍는다 — 연결 세그먼트는 튜브 메시가 이미 보여주므로 중복이고, phase 색은 위 헤더
		// 줄의 phase=... 텍스트가 낸다. 노드 단위 상태 구분은 [U]flight / [I]wrapped 오버레이 담당.
		for (const FVector& Point : Points)
		{
			AddShape(FGameplayDebuggerShape::MakePoint(Point, 2.0f, FColor::Yellow));
		}
		// latch 노드 강조(인덱스와 위치가 같은 스냅샷에서 온다).
		if (Snap)
		{
			for (int32 NodeIdx : Snap->LatchedNodes)
			{
				if (Points.IsValidIndex(NodeIdx))
				{
					AddShape(FGameplayDebuggerShape::MakePoint(Points[NodeIdx], 4.0f, FColor::Red));
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
	// colliders 총계의 단일 소스는 S.Colliders(항상 채워짐). [O] 뷰는 여기 총계를 반복하지 않고 분류만 낸다.
	AddTextLine(FString::Printf(TEXT("  {grey}diag: colliders=%d"), S.Colliders.Num()));

	//~ flight -----------------------------------------------------------
	if (HasView(EView::Flight) && S.bHasFlight)
	{
		// 이 노드의 접촉이 유효 후보로 이어졌는가. 같은 노드가 한 프레임에 여러 대상에 닿을 수 있으므로
		// 노드 인덱스만으로는 판정할 수 없고, 대상 식별 계약대로 (NodeIndex, Bone, Mesh) 3자를 모두 본다.
		// 후보 수가 작아 선형 탐색으로 충분하다.
		auto HasValidCandidateFor = [&S](const FRopeFlightNodeDebug& Node)
		{
			for (int32 i = 0; i < S.Candidates.Num(); ++i)
			{
				const FRopeContactCandidate& Candidate = S.Candidates[i];
				if (!Candidate.bValid || Candidate.NodeIndex != Node.NodeIndex || Candidate.Bone != Node.Contact.Bone)
				{
					continue;
				}
				// 키 배열은 Candidates와 1:1. 없으면(구 스냅샷) 본까지만 맞은 것으로 본다.
				if (!S.CandidateMeshKeys.IsValidIndex(i) || S.CandidateMeshKeys[i] == Node.ContactMeshKey)
				{
					return true;
				}
			}
			return false;
		};

		for (const FRopeFlightNodeDebug& Node : S.NodeDebug)
		{
			AddShape(FGameplayDebuggerShape::MakeSegment(Node.PrevPosition, Node.Position, 1.0f, FColor::White));

			if (Node.bNearBody)
			{
				AddShape(FGameplayDebuggerShape::MakePoint(Node.Position, 2.5f, FColor::Yellow));
			}

			if (Node.Contact.bHit)
			{
				const FColor HitColor = HasValidCandidateFor(Node) ? FColor::Green : FColor::Red;
				AddShape(FGameplayDebuggerShape::MakePoint(Node.Contact.SurfacePoint, 3.0f, HitColor));
				AddShape(FGameplayDebuggerShape::MakeSegment(Node.Contact.SurfacePoint,
					Node.Contact.SurfacePoint + Node.Contact.Normal.GetSafeNormal() * 22.0f, 1.0f, FColor::Blue));
			}
		}

		// 후보 자체를 정렬하면 짝을 이루는 CandidateMeshKeys와 인덱스 대응이 깨진다 — 인덱스를 정렬한다.
		TArray<int32> SortedIdx;
		SortedIdx.Reserve(S.Candidates.Num());
		for (int32 i = 0; i < S.Candidates.Num(); ++i)
		{
			SortedIdx.Add(i);
		}
		SortedIdx.Sort([&S](int32 A, int32 B)
			{
				return S.Candidates[A].Penetration > S.Candidates[B].Penetration;
			});

		// flight 진단은 3D 도형만 남긴다 — Flight phase는 찰나라 좌측 패널 텍스트를 읽을 시간이 없다.
		// (후보 상세 줄과 요약 줄 모두 그래서 제거했다.) 후보는 상위 N개만 박스로.
		const int32 MaxCandidateShapes = FMath::Min(5, SortedIdx.Num());
		for (int32 n = 0; n < MaxCandidateShapes; ++n)
		{
			const int32 i = SortedIdx[n];
			const FRopeContactCandidate& Candidate = S.Candidates[i];
			const FColor SourceColor = CandidateSourceColor(Candidate.Source);
			// dominant 판정도 (Mesh, Bone) 쌍으로 — 같은 스켈레톤을 쓰는 두 액터가 붙어 있으면 본 이름만
			// 으로는 반대편 액터의 후보와 구별되지 않는다.
			const bool bIsTracker = (Candidate.Bone == S.TrackerBone)
				&& (!S.CandidateMeshKeys.IsValidIndex(i) || S.CandidateMeshKeys[i] == S.TrackerMeshKey);
			const FColor CandidateColor = bIsTracker
				? FColor(FMath::Min(255, SourceColor.R + 40), FMath::Min(255, SourceColor.G + 20), FMath::Min(255, SourceColor.B + 40))
				: SourceColor;
			AddShape(FGameplayDebuggerShape::MakeBox(Candidate.WorldPoint, FVector(3.5f), CandidateColor));
		}

		// whip 가이드.
		if (S.bWhipActive && S.Positions.Num() >= 2)
		{
			const int32 LastNode = S.Positions.Num() - 1;
			AddShape(FGameplayDebuggerShape::MakeBox(S.Positions[0], FVector(4.5f), FColor::White));
			for (int32 i = 1; i <= LastNode; ++i)
			{
				const float Frac = static_cast<float>(i) / static_cast<float>(LastNode);
				if (Frac <= S.WhipGuidedEnd)
				{
					AddShape(FGameplayDebuggerShape::MakeBox(S.Positions[i], FVector(3.5f), FColor::Cyan));
				}
				else
				{
					AddShape(FGameplayDebuggerShape::MakePoint(S.Positions[i], 2.5f, FColor::Green));
				}
			}
			for (int32 i = 0; i < S.WhipGuideTargets.Num(); ++i)
			{
				AddShape(FGameplayDebuggerShape::MakePoint(S.WhipGuideTargets[i], 3.0f, FColor::Cyan));
				if (i + 1 < S.WhipGuideTargets.Num())
				{
					AddShape(FGameplayDebuggerShape::MakeSegment(S.WhipGuideTargets[i], S.WhipGuideTargets[i + 1], 1.0f, FColor::Cyan));
				}
				if (S.WhipGuideNodeIndices.IsValidIndex(i) && S.Positions.IsValidIndex(S.WhipGuideNodeIndices[i]))
				{
					AddShape(FGameplayDebuggerShape::MakeSegment(S.Positions[S.WhipGuideNodeIndices[i]],
						S.WhipGuideTargets[i], 1.0f, FColor(255, 140, 0)));
				}
			}
		}
	}

	//~ wrapped ----------------------------------------------------------
	if (HasView(EView::Wrapped) && S.bHasWrapped)
	{
		TSet<int32> LatchedSet(S.LatchedNodes);
		for (int32 i = 0; i < S.Positions.Num(); ++i)
		{
			if (LatchedSet.Contains(i))
			{
				AddShape(FGameplayDebuggerShape::MakeBox(S.Positions[i], FVector(4.5f), FColor::Yellow));
			}
			else
			{
				AddShape(FGameplayDebuggerShape::MakePoint(S.Positions[i], 2.5f, FColor::Green));
			}
		}

		AddTextLine(FString::Printf(TEXT("  {green}wrapped{white} bone=%s mesh=%s latched=%d"),
			*S.WrapBone.ToString(), *S.MeshName, S.Latched.Num()));
		// 장력(λ/h² 상대 힘). 임계치와 경고색은 자동 해제가 실제로 도는 모드에서만 낸다 — ③ Guaranteed는
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

		// Pull 방향 진단 시각화(장력 유무와 무관하게 bPullValid면 항상). 방향 문제를 눈으로 확정하기 위한 것:
		//  - 청록 선/점 = 앵커 → walk가 멈춘 조준 노드(첫 직선 다리). 이 끝이 벽 모서리에 놓여야 정상이고,
		//    프레임마다 조준 노드(aim=node#)가 튀면 방향이 통째로 점프하는 신호.
		//  - 노랑 화살표 = 스무딩 전 raw look-ahead 방향(프레임 지터가 여기서 보인다).
		//  - 초록 화살표 = EMA 스무딩 후(실제 인가) 방향. raw 대비 안정적이어야 한다.
		//  - 텍스트 raw↔smooth = 두 방향의 각도차(도) = 이번 프레임 지터 크기.
		if (S.bPullValid)
		{
			const float DiagLen = 40.0f;
			AddShape(FGameplayDebuggerShape::MakeSegment(S.PullPoint, S.PullAimPoint, 3.0f, FColor::Cyan));
			AddShape(FGameplayDebuggerShape::MakePoint(S.PullAimPoint, 6.0f, FColor::Cyan));
			AddShape(FGameplayDebuggerShape::MakeArrow(S.PullPoint, S.PullPoint + S.PullDirRaw * DiagLen,
				6.0f, 1.5f, FColor::Yellow));
			AddShape(FGameplayDebuggerShape::MakeArrow(S.PullPoint, S.PullPoint + S.PullDirection * DiagLen,
				8.0f, 2.0f, FColor::Green));
			const float JitterDeg = FMath::RadiansToDegrees(FMath::Acos(
				FMath::Clamp(static_cast<float>(FVector::DotProduct(S.PullDirRaw, S.PullDirection)), -1.0f, 1.0f)));
			AddTextLine(FString::Printf(TEXT("    {grey}pull-dir aim=node%d raw<->smooth=%.1f deg"),
				S.PullAimNode, JitterDeg));
		}

		// Pull 상태(샘플은 항상 산출) — 유효+장력>0(수치), 유효+슬랙, 무효(앵커가 손 노드거나 없음).
		// tether = 가용 로프 길이 초과분(자동 견인 입력), active = 능동 Pull 힘(입력 홀드).
		if (S.bPullValid && S.PullTension > KINDA_SMALL_NUMBER)
		{
			// 거리 release가 실제로 도는 모드에서 켜져 있으면 초과분이 한계에 근접/초과할 때 색으로
			// 경고(노랑 80%+, 빨강 초과). ③ Guaranteed는 거리 해제도 무효라 경고 대상이 아니다.
			const TCHAR* OvershootColor = TEXT("{white}");
			if (S.bAutoReleaseEnabled && S.DistanceReleaseSlack > 0.0f)
			{
				OvershootColor = (S.TetherOvershoot > S.DistanceReleaseSlack) ? TEXT("{red}")
					: (S.TetherOvershoot > S.DistanceReleaseSlack * 0.8f) ? TEXT("{yellow}") : TEXT("{white}");
			}
			// Constraint(λ) 모드는 테더 장력(λ/dt)과 상한을 덧붙인다 — 상한 80%+ 노랑, 도달 빨강(클램프 중).
			FString ConstraintInfo;
			if (S.bConstraintTetherMode)
			{
				const TCHAR* TensionColor = TEXT("{cyan}");
				if (S.MaxTetherTension > 0.0f)
				{
					TensionColor = (S.TetherTension >= S.MaxTetherTension * 0.999f) ? TEXT("{red}")
						: (S.TetherTension > S.MaxTetherTension * 0.8f) ? TEXT("{yellow}") : TEXT("{cyan}");
				}
				// 수신자 요약: 종류(sim/chr/anc)+유효질량 — 랙돌이 anc(무한질량)로 오분류되면 여기서 바로 보인다.
				auto KindTag = [](uint8 Kind) -> const TCHAR*
				{
					switch (Kind)
					{
					case 1: return TEXT("sim");
					case 2: return TEXT("chr");
					case 3: return TEXT("anc");
					default: return TEXT("none");
					}
				};
				const FString RecvInfo = S.bTetherEndpointsValid
					? FString::Printf(TEXT(" recv=%s%.0f/%s%.0fkg"),
						KindTag(S.TetherTargetKind), S.TetherMassTarget,
						KindTag(S.TetherWielderKind), S.TetherMassWielder)
					: FString();
				ConstraintInfo = FString::Printf(TEXT(" constraint %sT=%.0f{white}/%.0f%s"),
					TensionColor, S.TetherTension, S.MaxTetherTension, *RecvInfo);
			}
			AddTextLine(FString::Printf(TEXT("    {orange}pull{white} tension=%.0f taut=%s{white} chain=%s{white}(%.0f/%.0fcm, minT=%.0f, sag=%.0f) dir=%s tether=%s%.0fcm{white}(x%.2f, release=%.0f) active=%.0f%s"),
				S.PullTension, S.bPullTaut ? TEXT("{green}Y") : TEXT("{grey}N"),
				S.bChainTaut ? TEXT("{green}Y") : TEXT("{grey}N"), S.TautChordLen, S.FreeRestLen, S.MinFreeTension, S.MaxLegSag,
				*S.PullDirection.ToCompactString(), OvershootColor, S.TetherOvershoot,
				S.TetherResponse, S.DistanceReleaseSlack, S.ActivePullForce, *ConstraintInfo));
		}
		else if (S.bPullValid)
		{
			AddTextLine(FString::Printf(TEXT("    {grey}pull slack (tension 0, chain=%s %.0f/%.0fcm minT=%.0f sag=%.0f, tether=%.0fcm x%.2f%s)"),
				S.bChainTaut ? TEXT("Y") : TEXT("N"), S.TautChordLen, S.FreeRestLen, S.MinFreeTension, S.MaxLegSag,
				S.TetherOvershoot, S.TetherResponse,
				S.bConstraintTetherMode ? *FString::Printf(TEXT(", constraint T=%.0f"), S.TetherTension) : TEXT("")));
		}
		else
		{
			AddTextLine(TEXT("    {grey}pull n/a (no hand-side anchor)"));
		}

		const int32 MaxRows = FMath::Min(12, S.Latched.Num());
		for (int32 i = 0; i < MaxRows; ++i)
		{
			const FRopeLatchNode& Latch = S.Latched[i];
			const FVector WorldPos = S.Positions.IsValidIndex(Latch.NodeIndex)
				? S.Positions[Latch.NodeIndex] : FVector::ZeroVector;
			AddTextLine(FString::Printf(TEXT("    {grey}node=%d bone=%s local=%s world=%s"),
				Latch.NodeIndex, *Latch.Bone.ToString(),
				*Latch.BoneLocalPos.ToCompactString(), *WorldPos.ToCompactString()));
		}
		if (S.Latched.Num() > MaxRows)
		{
			AddTextLine(FString::Printf(TEXT("    {grey}... %d more"), S.Latched.Num() - MaxRows));
		}
	}

	//~ colliders --------------------------------------------------------
	// 콜라이더는 AddShape 대신 DrawDebug*(SDPG_Foreground)로 직접 그린다 — 에디터 셀렉션 라인처럼 항상 위에
	// 보여 메시와 겹쳐도 형상이 뚜렷하다. AddShape는 depth priority가 SDPG_World로 하드코딩돼 있어
	// (엔진 FGameplayDebuggerShape::Draw) 지오메트리에 가린다. 형상 표현력의 문제는 아니다 —
	// MakeCapsule/MakeBox 모두 회전 인자를 받는다.
	// 주의: DrawDebug*는 호출된 월드에만 그려져 원격 클라이언트로 복제되지 않는다. 지원 범위가
	// Standalone/로컬 시뮬레이션이라 성립하는 선택이며, 멀티플레이를 지원하게 되면 다시 봐야 한다.
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
				TEXT("  {grey}colliders [{green}wrappable{grey}/{red}rejected{grey}/{cyan}collision-only{grey}]  staticWrapTargets=%d"),
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

			// 노드별 접촉 진단: 각 접촉 노드에서 바깥 법선 화살표(= 어느 면인지) + 면 라벨(n<idx> ±축/edge).
			// 정적 월드=마젠타, 스켈레탈 본=주황. "붙는 노드가 어느 면에 어느 법선으로 닿았나"를 스크린샷으로 확정.
			for (const FRopeNodeContactDebug& NC : S.NodeContacts)
			{
				const FColor NColor = NC.bWorldStatic ? FColor(255, 0, 255) : FColor(255, 128, 0);
				const FVector Tip = NC.Position + NC.Normal * 15.0f;
				DrawDebugDirectionalArrow(World, NC.Position, Tip, 6.0f, NColor, false, -1.0f, FG, 2.0f);
				const FVector AN = NC.Normal.GetAbs();
				FString Face;
				if (AN.X > 0.9) { Face = NC.Normal.X > 0.0 ? TEXT("+X") : TEXT("-X"); }
				else if (AN.Y > 0.9) { Face = NC.Normal.Y > 0.0 ? TEXT("+Y") : TEXT("-Y"); }
				else if (AN.Z > 0.9) { Face = NC.Normal.Z > 0.0 ? TEXT("+Z") : TEXT("-Z"); }
				// 대각 법선 = 볼록 모서리 접촉.
				else { Face = TEXT("edge"); }
				// 라벨은 n<idx> <면>만(간결). 본 이름은 색(주황=스켈레탈)으로 갈음 — 정보량 과다 방지.
				DrawDebugString(World, Tip, FString::Printf(TEXT("n%d %s"), NC.NodeIndex, *Face), nullptr, NColor, 0.0f, true, 1.0f);
			}

			// 감김 축(Wrapping에서 결정된 경로 축): 대상을 관통하는 노란 선 + 방향 화살표. 같은 기둥을 여러
			// 각도로 던져 이 선이 늘 장축(기둥 세로)을 따르는지(일관성) 눈으로 확인한다. Wrapping 페이즈에서만 뜬다.
			if (S.bHasWrapAxis)
			{
				const FVector AxisDir = S.WrapAxisDirection.GetSafeNormal();
				const FVector AxisO = S.WrapAxisOrigin;
				// 축 길이는 로프 스케일에 비례(max(80, SegmentLength×6)) — 구 r.DynamicRope.Debug.DrawWrappingAxis
				// 즉시모드 드로우를 여기로 일원화하며 그 수식을 이식. 축이 퇴화(0벡터)면 선/화살표는 생략하고 텍스트만.
				const float AxisLen = FMath::Max(80.0f, S.WrapAxisSegmentLength * 6.0f);
				if (!AxisDir.IsNearlyZero())
				{
					DrawDebugLine(World, AxisO - AxisDir * AxisLen, AxisO + AxisDir * AxisLen, FColor::Yellow, false, -1.0f, FG, 3.0f);
					DrawDebugDirectionalArrow(World, AxisO, AxisO + AxisDir * AxisLen, 16.0f, FColor::Yellow, false, -1.0f, FG, 3.0f);
				}
				AddTextLine(FString::Printf(TEXT("  {yellow}wrapAxis{grey} dir=%s"), *AxisDir.ToCompactString()));
			}
		}
	}
}

#endif // WITH_GAMEPLAY_DEBUGGER
