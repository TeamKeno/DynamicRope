// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtrTemplates.h"
#include "RopeLifecycleTypes.generated.h"

class USceneComponent;
class URopeComponent;

/**
 * 라이프사이클 단계. 물리(solver)는 Free/Flight에서 전체를, Wrapping/Wrapped에서는 마스크되지 않은
 * 자유 구간만 굴린다. Contacting/Wrapping/Wrapped/Releasing의 판정·구동은 로직(Logic/ F-클래스) 담당.
 */
UENUM(BlueprintType)
enum class ERopePhase : uint8
{
	Free = 0,
	Flight = 1,

	/** 접촉 후보를 매 프레임 재수집하며 트래커 dwell로 wrap 진입을 판정하는 중. */
	Contacting = 2,

	/** 감기는 중(표면 경로 점진 생성 + front 모션 + 질량 마스크). */
	Wrapping = 3,

	Wrapped = 4,

	/** ③ 전용. 물리 Flight를 타지 않는다 — 조준 던지기는 확정 preview path를, 허공 던지기는 레이 끝점
	 *  아치를 따라간다(bFreeThrow). 전자는 Wrapped로, 후자는 Free로 빠진다. */
	GuidedThrow = 5,

	Releasing = 6,

	/** ③(GuaranteedWrap) 전용: 창(팁)을 손에 든 던지기 준비 상태. 로프는 숨기고, 이 상태에서만 throw가 성립한다.
	 *  꽂힌 뒤 release로 Free가 된 상태에서 EnterLoaded()로 진입한다. */
	Loaded = 7
};

/** 이 로프의 engagement(접촉/성립/③ 조준 던지기)가 끝난 이유. **성립(wrap) 전 abort도 포함**한다 —
 *  OnRopeReleased는 wrap 없이도 발화한다(Bone이 None일 수 있다). 중앙 OnAnyRopeReleased만 커밋된 wrap 전용. */
UENUM(BlueprintType)
enum class ERopeReleaseReason : uint8
{
	/** 게임플레이가 명시적으로 해제(URopeComponent::ReleaseWrap). */
	Manual = 0,

	/** 손~앵커 거리가 가용 로프 길이 + DistanceReleaseSlack 초과(자동). */
	Distance = 1,

	/** 최대 장력이 TensionReleaseForce를 지속 초과(자동). */
	Tension = 2,

	/** 대상 소실/wrap 실패 등 내부 사유. 성립 전 abort(접촉/감김/③ 연출 중 대상 소실)도 여기다. */
	Broken = 3,

	/** 외부 게임플레이가 로프를 절단(URopeComponent::CutRope). */
	Cut = 4,

	/**
	 * ③ 연출(GuidedThrow) 중 게임 규칙이 보장을 깼다 — ShouldAbortGuaranteedThrow 오버라이드가 true를 반환.
	 * 내부 실패(Broken)와 달리 **의도된 게임플레이 결과**다(대상이 회피/텔레포트했다 등). 소비자가 둘을
	 * 구분해야 "엔진 문제"와 "설계된 회피"에 다르게 반응할 수 있다.
	 */
	ThrowAborted = 5
};

/**
 * 감김 해결(도달) 모드 — 이 로프가 던지기~결착까지 무엇을 보장하는지의 계약.
 * 조준·preview의 지위와 판정 관문 사용 여부를 결정하고, Wielder의 조준/던지기 방식도 여기서 유도된다.
 * Wrapped 성립 이후(Hold/Pull/테더/release)는 모드 무관 공통. 근거: Docs/PoC/02_WrapResolveModes.md.
 */

UENUM(BlueprintType)
enum class ERopeWrapResolveMode : uint8
{
	// 날리기부터 결착까지 전부 창발. 조준 보정/preview가 없어 빗나감·스침·판정 미달이 전부 정상
	// 결과다(현실 대응).

	/** ① 전체 시뮬 — 아무것도 보장하지 않는다. 빗나감도 정상(샌드박스/리서치). */
	FullSimulation = 0 UMETA(DisplayName = "Full Simulation"),

	// aim ray가 대상을 잠가 명중은 보장하되, 결착 성립은 판정(감싼 각도/커버리지 관문)이 결정한다.
	// preview는 표시용(비구속).

	/** ② 보조+판정 — 명중은 보장, 결착은 판정. 실패(release)도 정상(전투/스킬). */
	AssistedJudged = 1 UMETA(DisplayName = "Assisted (Judged)"),

	// 던지는 순간 확정한 preview가 곧 실행 경로라 연출 후 실패가 없다. 조준이 안 잡히면(대상 없음/
	// 사거리 밖) 거부가 아니라 레이 끝점을 향해 아치로 날아가 안 꽂히고 Free로 떨어진다 — 보장은
	// '조준한 대상'에 대한 것이라 이것도 정상 결과다. 자동 release(장력/거리)는 무효 — 명시 해제만.

	/** ③ 무조건 성립 — 조준한 대상에 실패 없이 결착. Loaded(장전)에서만 던질 수 있다(데모/연출/이동기). */
	GuaranteedWrap = 2 UMETA(DisplayName = "Guaranteed")
};

/** 도달 모드가 강제하는 제약의 단일 소스 — phase 게이트(CanThrowInPhase). 던지기 진입·조준 HUD·
 *  테스트가 공용 소비한다. UObject/월드 의존이 없어 헤더 인라인 + 단위 테스트가 가능하다. */
namespace RopeWrapModes
{
	/**
	 * 이 모드에서 이 phase에 throw가 성립하는가. ③(GuaranteedWrap)는 Loaded(장전) 전용이고,
	 * ①②는 phase 게이트가 없어 **항상 true**다.
	 *
	 * [함정] 이건 "던지기 게이트에 걸리지 않는다"는 뜻이지 **"③이고 Loaded이다"가 아니다**.
	 * `X && Phase == Loaded` 꼴을 이 함수 단독으로 바꾸면 ①②가 true로 새어 들어간다 —
	 * 그런 자리는 반드시 `X && CanThrowInPhase(...)` 꼴을 유지할 것.
	 */
	inline bool CanThrowInPhase(ERopeWrapResolveMode Mode, ERopePhase Phase)
	{
		return Mode != ERopeWrapResolveMode::GuaranteedWrap || Phase == ERopePhase::Loaded;
	}
}

/**
 * Wrapped 성립 이벤트 페이로드(OnRopeWrapped / NotifyWrapped). 종전의 본 이름 하나에서 확장
 * (2026-07-13 회의 결정 G — Pierce 데미지 훅, 포획 강도 게임 규칙의 입구; 시그니처 변경은
 * 모드 도입과 함께 1회로 끝내는 클린 브레이크).
 */
USTRUCT(BlueprintType)
struct FRopeWrappedEventInfo
{
	GENERATED_BODY()

	/** 대표(지배) 본 — 종전 OnRopeWrapped(FName)와 같은 값. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope")
	FName Bone;

	/** 앵커가 걸친 모든 본(대표 본 우선, 중복 제거) — 양다리처럼 복수 본 성립의 전체 정보. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope")
	TArray<FName> Bones;

	/** 감긴 대상 mesh(cross-actor 포함). 이벤트 시점 이후 파괴될 수 있으니 weak. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope")
	TWeakObjectPtr<USceneComponent> Mesh;

	/** 성립 당시 이 로프의 도달 모드(③ Guaranteed 성립은 판정값이 -1이다 — preview 기반). */
	UPROPERTY(BlueprintReadOnly, Category = "Rope")
	ERopeWrapResolveMode ResolveMode = ERopeWrapResolveMode::AssistedJudged;

	/** 커밋 시점 누적 감싼 각도(도). 계산 불가/preview 기반(③) = -1. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope")
	float AngleDeg = -1.0f;

	/** 커밋 시점 축 둘레 커버리지(도, 0~360 — "빠져나갈 공백이 없는가"). 계산 불가/③ = -1. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope")
	float CoverageDeg = -1.0f;

	/** 성립 앵커(래치 노드) 수 — 포획 강도의 보조 지표. */
	UPROPERTY(BlueprintReadOnly, Category = "Rope")
	int32 AnchorCount = 0;

	/**
	 * 이 wrap을 성립시킨 로프 — 중앙 신호(OnAnyRopeWrapped) 구독자가 **어느 로프가 감았는지**를 알기 위한
	 * 식별자다. 대상 하나를 여러 로프가 동시에 감을 수 있으므로(양팔 포박 등), 구독자는 이 값으로 활성
	 * engagement 집합을 유지해야 한다 — mesh만 보면 로프 하나가 풀렸을 때 나머지가 남아 있는데도 반응을
	 * 되돌린다(URopeRagdollResponseComponent의 조기 복구 버그, 2026-07-20). 짝이 되는 해제 신호
	 * OnAnyRopeReleased도 같은 로프 포인터를 싣는다. 이벤트 이후 파괴될 수 있으니 weak.
	 */
	UPROPERTY(BlueprintReadOnly, Category = "Rope")
	TWeakObjectPtr<URopeComponent> Rope;
};
