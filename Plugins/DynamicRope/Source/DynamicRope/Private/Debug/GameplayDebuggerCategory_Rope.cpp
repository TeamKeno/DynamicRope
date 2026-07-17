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
// DrawDebug*(SDPG_Foreground) — 콜라이더 전경 오버레이(에디터 셀렉션처럼 위에 그림)
#include "DrawDebugHelpers.h"

namespace
{
	// GPU 튜브 경로/버킷을 로프 노드 수 + Subdiv(로프의 TubeSmoothingSubdiv)로 도출한다 — 프록시의 bUseGpuTube
	// 판정과 동일 수식(NumRings=(NumNodes-1)*Subdiv+1, NumRings<=MaxTubeRings면 GPU). 렌더 스레드 프록시 상태를
	// 크로스스레드로 읽지 않고 게임 스레드에서 재현(결정적). 버킷 표시 = 실제 디스패치가 고르는 스레드그룹 크기.
	FString TubeDiagString(int32 NumNodes, int32 WantedSubdiv)
	{
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

	FColor PhaseColor(ERopePhase Phase)
	{
		switch (Phase)
		{
		case ERopePhase::Flight:     return FColor::Cyan;
		case ERopePhase::Contacting: return FColor::Yellow;
		case ERopePhase::Wrapped:    return FColor::Green;
		case ERopePhase::Wrapping:   return FColor(255, 160, 0);
		case ERopePhase::Releasing:  return FColor::Orange;
		// GuidedThrow는 확정 경로를 따라가는 비행 — Flight와 구분되게 파랑.
		case ERopePhase::GuidedThrow: return FColor(80, 140, 255);
		// Reel은 장전 대기 — 던지기 전 상태라 Free보다 밝은 회색.
		case ERopePhase::Reel:       return FColor(210, 210, 210);
		case ERopePhase::Free:
		default:                     return FColor(160, 160, 160);
		}
	}

	const TCHAR* CandidateSourceName(ERopeContactCandidateSource Source)
	{
		switch (Source)
		{
		case ERopeContactCandidateSource::Actual: return TEXT("actual");
		case ERopeContactCandidateSource::PredictiveFree: return TEXT("predFree");
		case ERopeContactCandidateSource::PredictiveGuided: return TEXT("predGuided");
		default: return TEXT("?");
		}
	}

	FString CandidateSourceMaskName(uint8 SourceMask)
	{
		TArray<const TCHAR*> Parts;
		if ((SourceMask & static_cast<uint8>(ERopeContactCandidateSource::Actual)) != 0)
		{
			Parts.Add(TEXT("actual"));
		}
		if ((SourceMask & static_cast<uint8>(ERopeContactCandidateSource::PredictiveFree)) != 0)
		{
			Parts.Add(TEXT("predFree"));
		}
		if ((SourceMask & static_cast<uint8>(ERopeContactCandidateSource::PredictiveGuided)) != 0)
		{
			Parts.Add(TEXT("predGuided"));
		}

		FString Result;
		for (int32 i = 0; i < Parts.Num(); ++i)
		{
			if (i > 0)
			{
				Result += TEXT("+");
			}
			Result += Parts[i];
		}
		return Result.IsEmpty() ? FString(TEXT("?")) : Result;
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

	FString NodeListString(const TArray<int32>& Nodes)
	{
		FString Result;
		for (int32 i = 0; i < Nodes.Num(); ++i)
		{
			if (i > 0)
			{
				Result += TEXT(",");
			}
			Result += LexToString(Nodes[i]);
		}
		return Result;
	}
}

FGameplayDebuggerCategory_Rope::FGameplayDebuggerCategory_Rope()
{
	bShowOnlyWithDebugActor = false;

	// 하위 보기 토글 키. 카테고리가 활성일 때 입력된다. cvar(r.DynamicRope.Debug.*) 대체.
	// 키는 FName 리터럴로 지정한다 — EKeys/FKey는 InputCore 모듈 심볼이라 링크 의존을 피한다.
	const FGameplayDebuggerInputHandlerConfig FlightCfg(TEXT("ToggleFlight"), TEXT("U"));
	const FGameplayDebuggerInputHandlerConfig WrappedCfg(TEXT("ToggleWrapped"), TEXT("I"));
	const FGameplayDebuggerInputHandlerConfig CollidersCfg(TEXT("ToggleColliders"), TEXT("O"));
	const FGameplayDebuggerInputHandlerConfig LabelsCfg(TEXT("ToggleLabels"), TEXT("P"));
	const FGameplayDebuggerInputHandlerConfig AimCfg(TEXT("ToggleAim"), TEXT("J"));
	BindKeyPress(FlightCfg, this, &FGameplayDebuggerCategory_Rope::OnToggleFlight);
	BindKeyPress(WrappedCfg, this, &FGameplayDebuggerCategory_Rope::OnToggleWrapped);
	BindKeyPress(CollidersCfg, this, &FGameplayDebuggerCategory_Rope::OnToggleColliders);
	BindKeyPress(LabelsCfg, this, &FGameplayDebuggerCategory_Rope::OnToggleLabels);
	BindKeyPress(AimCfg, this, &FGameplayDebuggerCategory_Rope::OnToggleAim);
}

TSharedRef<FGameplayDebuggerCategory> FGameplayDebuggerCategory_Rope::MakeInstance()
{
	return MakeShareable(new FGameplayDebuggerCategory_Rope());
}

void FGameplayDebuggerCategory_Rope::OnToggleFlight()    { ViewMask ^= static_cast<uint8>(EView::Flight); }
void FGameplayDebuggerCategory_Rope::OnToggleWrapped()   { ViewMask ^= static_cast<uint8>(EView::Wrapped); }
void FGameplayDebuggerCategory_Rope::OnToggleColliders() { ViewMask ^= static_cast<uint8>(EView::Colliders); }
void FGameplayDebuggerCategory_Rope::OnToggleLabels()    { ViewMask ^= static_cast<uint8>(EView::Labels); }
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
		TEXT("{white}views  [U]flight=%s{white} [I]wrapped=%s{white} [O]colliders=%s{white} [P]labels=%s{white} [J]aim=%s"),
		OnOff(HasView(EView::Flight)), OnOff(HasView(EView::Wrapped)),
		OnOff(HasView(EView::Colliders)), OnOff(HasView(EView::Labels)),
		OnOff(HasView(EView::Aim))));

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

	// collider와 같은 이유로 AddShape 대신 DrawDebug*(전경): FGameplayDebuggerShape::MakeCapsule에는
	// 회전 인자가 없어 임의 방향 ray를 표현할 수 없다. 수명은 다음 수집까지만 남게 짧게 준다.
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
		DrawDebugSphere(World, Aim.HitWorldPos, 8.0f, 12, FColor::Yellow, false, LifeTime, FG, 2.0f);
		if (HasView(EView::Labels))
		{
			DrawDebugString(World, Aim.HitWorldPos + FVector(0.0f, 0.0f, 14.0f),
				Aim.Bone.IsNone() ? TEXT("(no bone)") : *Aim.Bone.ToString(),
				nullptr, FColor::Yellow, LifeTime, false, 1.0f);
		}
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
	// 상시 정보는 라이브 컴포넌트에서 — 디버거 수집 주기에 따른 지연 없이 항상 현재 값.
	const ERopePhase LivePhase = Rope.GetPhase();
	const FName Bone = Rope.GetWrappedBoneName();
	const TArray<FVector>& Points = Rope.GetCenterlinePositions();

	// 스케일링 상태: 슬립(솔브 스킵) 여부 + 거리 LOD iteration 배율(1 미만이면 감쇠 중).
	const float LODScale = Rope.GetSolverLODScale();
	AddTextLine(FString::Printf(
		TEXT("{yellow}Rope #%d{white} phase=%s nodes=%d wrapBone=%s%s%s%s"),
		Index, DebugPhaseName(LivePhase), Points.Num(),
		Bone.IsNone() ? TEXT("-") : *Bone.ToString(),
		Rope.IsSleeping() ? TEXT("  {cyan}asleep") : TEXT(""),
		LODScale < 0.999f ? *FString::Printf(TEXT("  {cyan}lod=x%.2f"), LODScale) : TEXT(""),
		Snap ? TEXT("") : TEXT("  {grey}(diag pending)")));

	// GPU 경로 진단: 솔버 step 여부(이번 프레임 실제 GPU step, false면 CPU 폴백/off) + 튜브 경로/버킷.
	AddTextLine(FString::Printf(
		TEXT("  {grey}gpu: solver=%s{grey} tube=%s"),
		Rope.IsGpuSteppedThisFrame() ? TEXT("{green}on") : TEXT("{red}cpu-fallback"),
		*TubeDiagString(Points.Num(), Rope.TubeSmoothingSubdiv)));

	//~ centerline(라이브 위치/페이즈) -----------------------------------
	if (HasView(EView::Centerline))
	{
		const FColor LineColor = PhaseColor(LivePhase);
		for (int32 i = 0; i < Points.Num(); ++i)
		{
			AddShape(FGameplayDebuggerShape::MakePoint(Points[i], 2.0f, FColor::Yellow));
			if (i + 1 < Points.Num())
			{
				AddShape(FGameplayDebuggerShape::MakeSegment(Points[i], Points[i + 1], 2.0f, LineColor));
			}
		}
		// latch 노드 강조(스냅샷 인덱스를 라이브 위치에 적용).
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

	// 이하 진단 오버레이는 transient 캡처 데이터 — 스냅샷이 있을 때만. 수집 주기만큼 지연될 수 있다.
	if (!Snap)
	{
		return;
	}
	const FRopeDebugSnapshot& S = *Snap;
	AddTextLine(FString::Printf(TEXT("  {grey}diag: solve=%d colliders=%d"),
		S.bSolveThisFrame ? 1 : 0, S.FrameColliderCount));

	//~ flight -----------------------------------------------------------
	if (HasView(EView::Flight) && S.bHasFlight)
	{
		TSet<int32> ValidCandidateNodes;
		for (const FRopeContactCandidate& Candidate : S.Candidates)
		{
			if (Candidate.bValid)
			{
				ValidCandidateNodes.Add(Candidate.NodeIndex);
			}
		}

		for (const FRopeFlightNodeDebug& Node : S.NodeDebug)
		{
			AddShape(FGameplayDebuggerShape::MakeSegment(Node.PrevPosition, Node.Position, 1.0f, FColor::White));

			if (Node.bNearBody)
			{
				AddShape(FGameplayDebuggerShape::MakePoint(Node.Position, 2.5f, FColor::Yellow));
			}

			if (Node.Contact.bHit)
			{
				const FColor HitColor = ValidCandidateNodes.Contains(Node.NodeIndex) ? FColor::Green : FColor::Red;
				AddShape(FGameplayDebuggerShape::MakePoint(Node.Contact.SurfacePoint, 3.0f, HitColor));
				AddShape(FGameplayDebuggerShape::MakeSegment(Node.Contact.SurfacePoint,
					Node.Contact.SurfacePoint + Node.Contact.Normal.GetSafeNormal() * 22.0f, 1.0f, FColor::Blue));
			}
		}

		TArray<FRopeContactCandidate> Sorted = S.Candidates;
		Sorted.Sort([](const FRopeContactCandidate& A, const FRopeContactCandidate& B)
			{
				return A.Penetration > B.Penetration;
			});

		const int32 MaxLabels = FMath::Min(5, Sorted.Num());
		for (int32 i = 0; i < MaxLabels; ++i)
		{
			const FRopeContactCandidate& Candidate = Sorted[i];
			const FColor SourceColor = CandidateSourceColor(Candidate.Source);
			const FColor CandidateColor = (Candidate.Bone == S.TrackerBone)
				? FColor(FMath::Min(255, SourceColor.R + 40), FMath::Min(255, SourceColor.G + 20), FMath::Min(255, SourceColor.B + 40))
				: SourceColor;
			AddShape(FGameplayDebuggerShape::MakeBox(Candidate.WorldPoint, FVector(3.5f), CandidateColor));

			if (HasView(EView::Labels))
			{
				AddTextLine(FString::Printf(
					TEXT("  {grey}cand node=%d src=%s primary=%s bone=%s pen=%.2f relTan=%.1f wrap=%.2f"),
					Candidate.NodeIndex, *CandidateSourceMaskName(Candidate.SourceMask),
					CandidateSourceName(Candidate.Source), *Candidate.Bone.ToString(),
					Candidate.Penetration, Candidate.RelativeTangentialSpeed, Candidate.WrapDirectionScore));
			}
		}

		// 요약(원래 on-screen 텍스트).
		int32 ActualCount = 0, PredFreeCount = 0, PredGuidedCount = 0;
		for (const FRopeContactCandidate& C : S.Candidates)
		{
			if ((C.SourceMask & static_cast<uint8>(ERopeContactCandidateSource::Actual)) != 0) { ++ActualCount; }
			if ((C.SourceMask & static_cast<uint8>(ERopeContactCandidateSource::PredictiveFree)) != 0) { ++PredFreeCount; }
			if ((C.SourceMask & static_cast<uint8>(ERopeContactCandidateSource::PredictiveGuided)) != 0) { ++PredGuidedCount; }
		}
		AddTextLine(FString::Printf(
			TEXT("  {cyan}flight{white} cand=%d (actual=%d predFree=%d predGuided=%d)  trackerBone=%s  nodes=%d/%d [%s]  capture=%s"),
			S.Candidates.Num(), ActualCount, PredFreeCount, PredGuidedCount,
			*S.TrackerBone.ToString(), S.TrackerNodes.Num(), S.MinLatchNodes,
			*NodeListString(S.TrackerNodes), S.bShouldCapture ? TEXT("yes") : TEXT("no")));

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
		// 장력(λ/h² 상대 힘): 임계치가 켜져 있으면 초과 여부를 색으로(노랑=근접 80%+, 빨강=초과).
		if (S.TensionReleaseForce > 0.0f)
		{
			const TCHAR* Color = (S.WrapTension > S.TensionReleaseForce) ? TEXT("{red}")
				: (S.WrapTension > S.TensionReleaseForce * 0.8f) ? TEXT("{yellow}") : TEXT("{white}");
			AddTextLine(FString::Printf(TEXT("    tension=%s%.0f{white} / release=%.0f"),
				Color, S.WrapTension, S.TensionReleaseForce));
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
			// 거리 release가 켜져 있으면 초과분이 한계에 근접/초과할 때 색으로 경고(노랑 80%+, 빨강 초과).
			const TCHAR* OvershootColor = TEXT("{white}");
			if (S.DistanceReleaseSlack > 0.0f)
			{
				OvershootColor = (S.TetherOvershoot > S.DistanceReleaseSlack) ? TEXT("{red}")
					: (S.TetherOvershoot > S.DistanceReleaseSlack * 0.8f) ? TEXT("{yellow}") : TEXT("{white}");
			}
			AddTextLine(FString::Printf(TEXT("    {orange}pull{white} tension=%.0f taut=%s{white} chain=%s{white}(%.0f/%.0fcm) dir=%s tether=%s%.0fcm{white}(x%.2f, release=%.0f) active=%.0f"),
				S.PullTension, S.bPullTaut ? TEXT("{green}Y") : TEXT("{grey}N"),
				S.bChainTaut ? TEXT("{green}Y") : TEXT("{grey}N"), S.TautChordLen, S.FreeRestLen,
				*S.PullDirection.ToCompactString(), OvershootColor, S.TetherOvershoot,
				S.TetherResponse, S.DistanceReleaseSlack, S.ActivePullForce));
		}
		else if (S.bPullValid)
		{
			AddTextLine(FString::Printf(TEXT("    {grey}pull slack (tension 0, chain=%s %.0f/%.0fcm, tether=%.0fcm x%.2f)"),
				S.bChainTaut ? TEXT("Y") : TEXT("N"), S.TautChordLen, S.FreeRestLen,
				S.TetherOvershoot, S.TetherResponse));
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
	// 콜라이더는 AddShape(SDPG_World 하드코딩 → 지오메트리에 가림) 대신 DrawDebug*(SDPG_Foreground)로 직접
	// 그린다 — 에디터 셀렉션 라인처럼 항상 위에 보여, 메시와 겹쳐도 형상이 뚜렷하다. 이 카테고리는 이미
	// CollectData에서 live 컴포넌트를 읽는 로컬 전용 설계라 DrawDebug 직접 호출과 정합(네트워크 리플리케이션 무관).
	if (HasView(EView::Colliders))
	{
		if (UWorld* World = Rope.GetWorld())
		{
			constexpr uint8 FG = SDPG_Foreground;
			constexpr float LineThick = 1.5f;

			// 색 범례 + 랩 대상 개수(로프가 실제로 감길 추출 셰이프가 몇 개 질의됐는지).
			int32 WrapTargetCount = 0;
			for (const FRopeDebugCollider& C : S.Colliders)
			{
				if (C.bWrapTarget) { ++WrapTargetCount; }
			}
			AddTextLine(FString::Printf(
				TEXT("  {grey}colliders=%d  {blue}wrapTarget=%d{grey} [{blue}wrap{grey}/{cyan}worldStatic{grey}/{green}bone{grey}]"),
				S.Colliders.Num(), WrapTargetCount));

			for (const FRopeDebugCollider& C : S.Colliders)
			{
				// 소스별 색: 정적 메시 랩 대상(URopeWrapTargetComponent가 서빙한 추출 박스/캡슐 = 로프가 실제로
				// 감길 셰이프)은 파랑, 정적 월드(박스/컨벡스/정적 캡슐)는 cyan, 스켈레탈 본 캡슐은 초록.
				const FColor Color = C.bWrapTarget ? FColor(40, 120, 255)
					: (C.bWorldStatic ? FColor::Cyan : FColor::Green);
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
				constexpr float AxisLen = 150.0f;
				DrawDebugLine(World, AxisO - AxisDir * AxisLen, AxisO + AxisDir * AxisLen, FColor::Yellow, false, -1.0f, FG, 3.0f);
				DrawDebugDirectionalArrow(World, AxisO, AxisO + AxisDir * AxisLen, 14.0f, FColor::Yellow, false, -1.0f, FG, 3.0f);
				AddTextLine(FString::Printf(TEXT("  {yellow}wrapAxis{grey} dir=%s"), *AxisDir.ToCompactString()));
			}
		}
	}
}

#endif // WITH_GAMEPLAY_DEBUGGER
