// Copyright Epic Games, Inc. All Rights Reserved.
//
// rope 디버그의 단일 진입점. 인게임에서 게이트플레이 디버거를 켜고 'Rope' 카테고리를 토글하면 디버그
// 대상 액터의 URopeComponent들을 표시한다. 매 프레임 대상 액터를 URopeDebugSubsystem에 등록하면
// sim tick(GT)이 그 로프만 캡처해 스냅샷을 남기고, 여기서 읽어 AddShape/AddTextLine으로 그린다.
// 하위 보기(Nodes/Flight/Wrap/Colliders/Aim)와 상세 수준(Advanced)은 카테고리 input binding 키로 토글한다.
// Aim 보기만은 로프가 아니라 대상 액터의 URopeWielderComponent가 매 틱 캐시하는 FRopeAimHudSample을
// 라이브로 읽어 그린다 — 조준은 Wielder 소유라 로프 스냅샷에 실을 수 없다.
// WITH_GAMEPLAY_DEBUGGER가 꺼진 빌드(shipping 등)에서는 전체가 컴파일에서 제외된다.

#pragma once

#include "CoreMinimal.h"

#if WITH_GAMEPLAY_DEBUGGER

#include "GameplayDebuggerCategory.h"

class APlayerController;
class AActor;
class URopeComponent;
class URopeWielderComponent;
struct FRopeDebugSnapshot;

class FGameplayDebuggerCategory_Rope : public FGameplayDebuggerCategory
{
public:
	FGameplayDebuggerCategory_Rope();

	virtual void CollectData(APlayerController* OwnerPC, AActor* DebugActor) override;

	static TSharedRef<FGameplayDebuggerCategory> MakeInstance();

private:
	// 하위 보기 토글 비트(input binding 키로 켜고 끈다). 분류 기준은 "무엇에 대한 진단인가"다:
	// 노드 상태는 Nodes, 감김 경로/결과는 Wrap, 충돌 형상은 Colliders. Summary(헤더 두 줄)는 비트가
	// 없고 항상 나온다.
	enum class EView : uint8
	{
		// 노드 점 + latch 강조 + 노드별 근접 재질의(법선 화살표).
		Nodes     = 1 << 0,
		// Flight 후보/sweep/whip 가이드.
		Flight    = 1 << 1,
		// Wrapping 경로 축 + Wrapped 결과(latch/장력/pull).
		Wrap      = 1 << 2,
		// collider 형상만.
		Colliders = 1 << 3,
		Aim       = 1 << 4,
		// 켜진 보기의 상세 수치까지 낸다(알고리즘 튜닝용). 끄면 통합 사용자용 요약만 남는다.
		Advanced  = 1 << 5,
	};

	// 기본값: 통합 사용자가 처음 켰을 때 화면이 덮이지 않도록 최소로 둔다. 상세 보기는 필요할 때 키로.
	static constexpr uint8 DefaultViewMask = static_cast<uint8>(EView::Aim);

	bool HasView(EView Flag) const { return (ViewMask & static_cast<uint8>(Flag)) != 0; }

	// 키 핸들러(카테고리 활성 중 해당 키로 토글).
	void OnToggleNodes();
	void OnToggleFlight();
	void OnToggleWrap();
	void OnToggleColliders();
	void OnToggleAim();
	void OnToggleAdvanced();

	// 한 로프를 그린다. **한 화면은 하나의 시간 기준만 쓴다** — 헤더(phase/nodes/wrapBone/solve)와
	// centerline, 진단 오버레이가 모두 같은 스냅샷에서 나온다. 헤더만 라이브로 두면 같은 노드가 두 시점에
	// 겹쳐 그려져 시뮬 떨림이나 latch 불안정으로 오독된다. 스냅샷 나이는 헤더의 age=Nf로 드러낸다.
	// Snap==null(캡처 첫 프레임)일 때만 헤더를 라이브로 내고 (live) 라벨을 붙이며, 오버레이는 생략한다.
	void DrawRope(int32 Index, const URopeComponent& Rope, const FRopeDebugSnapshot* Snap);

	// 조준 ray를 그린다. 질의는 하지 않는다 — Wielder가 이미 매 틱 스윕해 캐시한 샘플을 읽기만 한다
	// (green=감김 가능 / red=걸렸지만 감김 불가 / cyan=미스). aim ray 모드가 아니면 빈 샘플이라 no target.
	void DrawAim(const URopeWielderComponent& Wielder);

	uint8 ViewMask = DefaultViewMask;
};

#endif // WITH_GAMEPLAY_DEBUGGER
