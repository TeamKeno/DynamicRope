// Copyright Epic Games, Inc. All Rights Reserved.
//
// URopeComponent의 디테일 패널 클래스 커스터마이즈. ResolveMode == ③ GuaranteedWrap일 때 의미 없는
// 튜닝을 회색처리(edit-const)한다.
//
// 역할 분담:
//  - WrapConfig / WhipConfig: 컴포넌트 직속 struct 멤버라 ResolveMode를 볼 수 있어 RopeComponent.h의
//    구조체-멤버 EditCondition으로 회색처리한다(여기서 손대지 않음). ShowOnlyInnerProperties로 승격된
//    인라인 자식까지 edit-const가 프로퍼티 노드 트리를 타고 함께 회색이 된다.
//  - 이 클래스가 담당하는 유일한 대상: HoldConfig 내부의 Release 필드 3종. 구조체 *안*이라 EditCondition이
//    컴포넌트의 ResolveMode를 볼 수 없으므로, 여기서 HideProperty로 숨긴다(자동 release는 ①②만 유효 —
//    ③은 명시 해제만이라 무의미. 제자리 회색처리가 마땅치 않아 숨김으로 정함 — 2026-07-25 결정).
//    Hold의 나머지(Pull/Tether/Taut)는 ③에서도 유효하므로 건드리지 않는다.
//
// ①FullSimulation·②AssistedJudged는 전부 그대로 편집 가능하다.

#pragma once

#include "CoreMinimal.h"
#include "IDetailCustomization.h"

class IDetailLayoutBuilder;

class FRopeComponentDetails : public IDetailCustomization
{
public:
	static TSharedRef<IDetailCustomization> MakeInstance();

	//~ IDetailCustomization
	virtual void CustomizeDetails(IDetailLayoutBuilder& DetailBuilder) override;
};
