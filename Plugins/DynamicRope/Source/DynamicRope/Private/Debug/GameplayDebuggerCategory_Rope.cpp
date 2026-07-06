// Copyright Epic Games, Inc. All Rights Reserved.

#include "Debug/GameplayDebuggerCategory_Rope.h"

#if WITH_GAMEPLAY_DEBUGGER

#include "RopeComponent.h"
#include "Debug/RopeDebugSnapshot.h"
#include "Subsystem/RopeDebugSubsystem.h"
#include "GameFramework/Actor.h"

namespace
{
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
	BindKeyPress(FlightCfg, this, &FGameplayDebuggerCategory_Rope::OnToggleFlight);
	BindKeyPress(WrappedCfg, this, &FGameplayDebuggerCategory_Rope::OnToggleWrapped);
	BindKeyPress(CollidersCfg, this, &FGameplayDebuggerCategory_Rope::OnToggleColliders);
	BindKeyPress(LabelsCfg, this, &FGameplayDebuggerCategory_Rope::OnToggleLabels);
}

TSharedRef<FGameplayDebuggerCategory> FGameplayDebuggerCategory_Rope::MakeInstance()
{
	return MakeShareable(new FGameplayDebuggerCategory_Rope());
}

void FGameplayDebuggerCategory_Rope::OnToggleFlight()    { ViewMask ^= static_cast<uint8>(EView::Flight); }
void FGameplayDebuggerCategory_Rope::OnToggleWrapped()   { ViewMask ^= static_cast<uint8>(EView::Wrapped); }
void FGameplayDebuggerCategory_Rope::OnToggleColliders() { ViewMask ^= static_cast<uint8>(EView::Colliders); }
void FGameplayDebuggerCategory_Rope::OnToggleLabels()    { ViewMask ^= static_cast<uint8>(EView::Labels); }

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
		TEXT("{white}views  [U]flight=%s{white} [I]wrapped=%s{white} [O]colliders=%s{white} [P]labels=%s"),
		OnOff(HasView(EView::Flight)), OnOff(HasView(EView::Wrapped)),
		OnOff(HasView(EView::Colliders)), OnOff(HasView(EView::Labels))));

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

void FGameplayDebuggerCategory_Rope::DrawRope(int32 Index, const URopeComponent& Rope, const FRopeDebugSnapshot* Snap)
{
	// 상시 정보는 라이브 컴포넌트에서 — 디버거 수집 주기에 따른 지연 없이 항상 현재 값.
	const ERopePhase LivePhase = Rope.GetPhase();
	const FName Bone = Rope.GetWrappedBoneName();
	const TArray<FVector>& Points = Rope.GetCenterlinePositions();

	AddTextLine(FString::Printf(
		TEXT("{yellow}Rope #%d{white} phase=%s nodes=%d wrapBone=%s%s"),
		Index, DebugPhaseName(LivePhase), Points.Num(),
		Bone.IsNone() ? TEXT("-") : *Bone.ToString(),
		Snap ? TEXT("") : TEXT("  {grey}(diag pending)")));

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

		// Pull 상태(샘플은 항상 산출) — 유효+장력>0(화살표), 유효+슬랙, 무효(앵커가 손 노드거나 없음).
		// tether = 가용 로프 길이 초과분(자동 견인 입력), active = 능동 Pull 힘(입력 홀드).
		if (S.bPullValid && S.PullTension > KINDA_SMALL_NUMBER)
		{
			const float ArrowLen = FMath::Clamp(S.PullTension * 0.01f, 15.0f, 120.0f);
			AddShape(FGameplayDebuggerShape::MakeArrow(S.PullPoint, S.PullPoint + S.PullDirection * ArrowLen,
				8.0f, 2.0f, FColor::Orange));
			AddTextLine(FString::Printf(TEXT("    {orange}pull{white} tension=%.0f dir=%s tether=%.0fcm(x%.2f) active=%.0f"),
				S.PullTension, *S.PullDirection.ToCompactString(), S.TetherOvershoot, S.TetherResponse, S.ActivePullForce));
		}
		else if (S.bPullValid)
		{
			AddTextLine(FString::Printf(TEXT("    {grey}pull slack (tension 0, tether=%.0fcm x%.2f)"),
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
	if (HasView(EView::Colliders))
	{
		for (const FRopeDebugCollider& C : S.Colliders)
		{
			if (C.bIsCapsule)
			{
				// 실제 충돌 볼륨(sphyl) 그대로 그린다: 세그먼트+끝점 구만 그리면 원통 몸통이 빠져
				// 팔다리처럼 가늘고 긴 캡슐이 본을 못 덮는 것처럼 보인다. DrawDebugCapsule의
				// HalfHeight는 반구 포함 전체 절반이므로 세그먼트 절반 + Radius. 축퇴(A==B,
				// physics asset 구 셰이프)는 방향이 없으므로 구(Point)로 그린다.
				const FVector Axis = C.B - C.A;
				const float SegLen = static_cast<float>(Axis.Size());
				if (SegLen > KINDA_SMALL_NUMBER)
				{
					const FVector Center = (C.A + C.B) * 0.5f;
					const FRotator Rot = FRotationMatrix::MakeFromZ(Axis).Rotator();
					AddShape(FGameplayDebuggerShape::MakeCapsule(Center, Rot, C.Radius, SegLen * 0.5f + C.Radius, FColor::Green));
				}
				else
				{
					AddShape(FGameplayDebuggerShape::MakePoint(C.A, C.Radius, FColor::Green));
				}
			}
			else if (C.Bounds.IsValid)
			{
				AddShape(FGameplayDebuggerShape::MakeBox(C.Bounds.GetCenter(), C.Bounds.GetExtent(), FColor::Green));
			}
		}
	}
}

#endif // WITH_GAMEPLAY_DEBUGGER
