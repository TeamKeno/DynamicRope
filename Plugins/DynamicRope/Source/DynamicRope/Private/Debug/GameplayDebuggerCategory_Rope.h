// Copyright Epic Games, Inc. All Rights Reserved.
//
// rope 디버그의 단일 진입점. 인게임에서 게이트플레이 디버거를 켜고 'Rope' 카테고리를 토글하면 디버그
// 대상 액터의 URopeComponent들을 표시한다. 매 프레임 대상 액터를 URopeDebugSubsystem에 등록하면
// sim tick(GT)이 그 로프만 캡처해 스냅샷을 남기고, 여기서 읽어 AddShape/AddTextLine으로 그린다.
// 하위 보기(Flight/Wrapped/Colliders/Labels/Aim)는 카테고리 input binding 키로 토글한다(cvar 폐지).
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
	// 하위 보기 토글 비트(input binding 키로 켜고 끈다). 기본은 전부 on.
	enum class EView : uint8
	{
		Centerline = 1 << 0,
		Flight     = 1 << 1,
		Wrapped    = 1 << 2,
		Colliders  = 1 << 3,
		Labels     = 1 << 4,
		Aim        = 1 << 5,
	};

	bool HasView(EView Flag) const { return (ViewMask & static_cast<uint8>(Flag)) != 0; }

	// 키 핸들러(카테고리 활성 중 해당 키로 토글).
	void OnToggleFlight();
	void OnToggleWrapped();
	void OnToggleColliders();
	void OnToggleLabels();
	void OnToggleAim();

	// 한 로프를 그린다. phase/centerline/wrapBone 같은 상시 정보는 **라이브 컴포넌트**에서 읽어 항상
	// 정확하게(디버거 수집 주기에 따른 지연 없음), flight 후보·sweep·wrapped 상세 같은 transient 진단
	// 오버레이만 스냅샷에서 읽는다(Snap==null이면 오버레이는 생략). 진단은 수집 주기만큼 지연될 수 있다.
	void DrawRope(int32 Index, const URopeComponent& Rope, const FRopeDebugSnapshot* Snap);

	// 조준 ray를 그린다. 질의는 하지 않는다 — Wielder가 이미 매 틱 스윕해 캐시한 샘플을 읽기만 한다
	// (green=감김 가능 / red=걸렸지만 감김 불가 / cyan=미스). aim ray 모드가 아니면 빈 샘플이라 no target.
	void DrawAim(const URopeWielderComponent& Wielder);

	uint8 ViewMask = 0xFF;
};

#endif // WITH_GAMEPLAY_DEBUGGER
