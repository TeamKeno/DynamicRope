// Copyright Epic Games, Inc. All Rights Reserved.
//
// 로프에 감긴 스켈레탈 대상의 랙돌 반응 컴포넌트(옵트인). 감길 대상(마네킹/동물/드래곤 — ACharacter
// 또는 스켈레탈 메시를 가진 아무 액터)에 붙이면:
//  - bRagdollOnWrapped(기본 켜짐): 로프가 이 메시를 감으면(Wrapped) RagdollOnWrappedDelay 후 자동
//    랙돌 전환. bOnlyBelowWrappedBone이면 감긴 본 이하만 부분 랙돌.
//  - bRecoverRagdollOnRopeRelease(기본 켜짐): *자동 전환된* 랙돌은 감았던 로프가 풀리면 자동 복귀
//    (감으면 자빠지고 놓으면 일어난다 — 대칭). 수동/치트로 진입한 랙돌은 로프 release와 무관하게 유지.
//  - EnterRagdoll/EnterPartialRagdoll/RecoverFromRagdoll: BP/코드에서 직접 제어하는 정식 API.
//
// 자기를 감을 로프를 미리 알 수 없으므로(cross-actor throw가 흔함), 매 프레임 로프를 전수 순회하는 대신
// 서브시스템의 중앙 wrap/release 신호(URopeSimSubsystem::OnAnyRopeWrapped/OnAnyRopeReleased)에 구독해
// 자기 메시가 감겼는지/풀렸는지로 반응한다. 랙돌 전환 자체는 게임/컴포넌트 책임이라는 계약(플러그인
// 코어는 랙돌을 요구하지 않는다 — RopeRagdollTransitionTests.cpp 주석)에 따라 이 컴포넌트는 어디까지나
// 옵트인 편의/레퍼런스 구현이며, 게임이 자체 랙돌 로직을 그대로 쓸 수도 있다.
//
// #if !UE_BUILD_SHIPPING 콘솔 명령(이 컴포넌트가 붙은 월드 내 모든 액터에 일괄 적용 — 개발 확인용):
//   Rope.Ragdoll             풀 랙돌 토글(랙돌 중이면 복귀)
//   Rope.Ragdoll spine_01    해당 본 이하만 부분 랙돌
//   Rope.Ragdoll.Recover     애니메이션 복귀
//   Rope.Ragdoll.Destroy     대상 액터 파괴(감긴 중 파괴 → 로프 release 확인용)

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
// FTimerHandle(자동 전환 지연) 멤버.
#include "Engine/TimerHandle.h"
// ERopeReleaseReason / FRopeWrappedEventInfo — 중앙 신호 페이로드.
#include "Core/RopeTypes.h"
#include "RopeRagdollResponseComponent.generated.h"

class USkeletalMeshComponent;
class USceneComponent;

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopeRagdollResponseComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	URopeRagdollResponseComponent();

	//~ UActorComponent — 중앙 wrap/release 신호에 구독/해제.
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	/** 로프가 이 액터의 메시를 감으면(Wrapped) 자동으로 랙돌 전환한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Ragdoll")
	bool bRagdollOnWrapped = true;

	/** 자동 전환까지의 지연(초). 감긴 직후 vs 잠시 뒤 전환의 차이를 조절. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Ragdoll", meta = (ClampMin = "0.0", Units = "s", EditCondition = "bRagdollOnWrapped"))
	float RagdollOnWrappedDelay = 0.3f;

	/**
	 * 자동 전환 시 풀 랙돌 대신 감긴 본 이하만 부분 랙돌(회의의 "감긴 본만 전환" 안).
	 *
	 * ⚠ 켜기 전에 읽을 것 — 켜면 **테더(자동 회수)가 대상을 못 끈다**(2026-07-15 조사, 미수정: 재현 조건이
	 * 기본 off이고 쓸 계획이 없어 보류). 부분 랙돌은 정의상 캡슐/무브먼트를 살려두므로(EnterPartialRagdoll)
	 * 감긴 본만 시뮬이고 그 부모는 키네마틱이다. 로프 쪽에서 두 겹으로 깨진다:
	 *  1) 인가: 테더는 감긴 본에만 서보를 넣는데 키네마틱 부모 구속(무한질량)이 그걸 흡수해 액터로 전달되지
	 *     않는다 — 팔이 관절 한계까지 휘적일 뿐 캐릭터는 안 움직인다. 능동 Pull은 이 경우 이동체에도 같은
	 *     힘을 함께 준다(URopeComponent::ApplyPullForce의 이중 인가) → **키 입력 Pull은 되는데 테더만 안 되는**
	 *     비대칭으로 보인다.
	 *  2) 질량/판정: 테더의 수신자 해석(ResolveTetherEndpoint)은 그 본의 *바디* 질량(팔뚝 ≈ 3kg)을 유효질량으로
	 *     보고한다 — 실제로는 키네마틱에 묶여 유효질량이 무한인데도. 그 거짓값이 MassShare의 몫 분배(가벼운
	 *     대상 = 거의 전량 배정 → wielder는 양보 안 함)와 BinaryPullable의 끌림 판정(pullable=true → wielder
	 *     완전 자유)을 모두 오염시켜, overshoot가 닫히지 않고 로프만 늘어난다.
	 * 고치려면 두 겹 다 필요하다(테더도 CMC 동반 구동 + 키네마틱에 묶인 본은 캐릭터 질량 보고). 인가만 고치면
	 * 몫 분배가 여전히 틀린다. 기본값 false로 두는 한 무해하다 — 풀 랙돌은 전 바디가 시뮬이라 키네마틱 앵커가
	 * 없고 관절로 몸 전체가 끌려오므로 정상 동작한다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Ragdoll", meta = (EditCondition = "bRagdollOnWrapped"))
	bool bOnlyBelowWrappedBone = false;

	/**
	 * 자동 전환된 랙돌을, 감았던 로프가 풀리면(release) 자동으로 복귀시킨다(기본 켜짐 — 감으면 자빠지고
	 * 놓으면 일어나는 대칭). 끄면 로프가 풀려도 랙돌을 유지한다(RecoverFromRagdoll을 직접 부르기 전까지).
	 * 수동/치트로 진입한 랙돌에는 적용되지 않는다 — 로프가 멋대로 일으키지 않도록 자동 전환분만 대상.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Ragdoll")
	bool bRecoverRagdollOnRopeRelease = true;

	/**
	 * 풀 랙돌 복귀 시 캡슐(액터)을 랙돌이 멈춘 위치로 수평 이동한다(기본 켜짐). 랙돌 동안 무브먼트가
	 * 꺼져 캡슐은 제자리인데 메시만 물리로(예: pull) 끌려가므로, 그냥 복귀하면 메시가 원래 캡슐로
	 * 되돌아가며 크게 순간이동한다 — 대신 캡슐을 메시(RecoverAnchorBoneName 본) 쪽으로 옮겨 그
	 * 되돌아감이 시각적 no-op이 되게 한다. 위치만(수평), 회전/높이는 유지하고 지면 스냅은 이어지는
	 * MOVE_Walking이 처리한다. 부분 랙돌에는 적용 안 함(메시를 리셋하지 않아 순간이동이 없다).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Ragdoll")
	bool bMoveCapsuleToMeshOnRecover = true;

	/**
	 * 위 캡슐 재정렬의 기준 본 — "랙돌이 어디서 멈췄나"를 대표하는 본(보통 몸통 중심). 마네킹 기본은
	 * pelvis. 드래곤/동물 등 스켈레톤이 다르면 주 물리 바디 본으로 바꾼다. 스켈레톤에 없으면 재정렬을
	 * 건너뛴다(경고 후 종전 동작).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Ragdoll", meta = (EditCondition = "bMoveCapsuleToMeshOnRecover"))
	FName RecoverAnchorBoneName = TEXT("pelvis");

	/** 랙돌 동안 메시에 줄 콜리전 프로파일. 마네킹 기본(CharacterMesh)은 물리 충돌이 없어 전환이 필수. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Ragdoll")
	FName RagdollCollisionProfileName = TEXT("Ragdoll");

	/** 풀 랙돌 전환: 메시 전체 물리 시뮬 + 캡슐 콜리전/무브먼트 정지(ACharacter일 때). */
	UFUNCTION(BlueprintCallable, Category = "Rope|Ragdoll")
	void EnterRagdoll();

	/** 부분 랙돌 전환: BoneName 이하 바디만 물리 시뮬(blend weight 1). 나머지는 애니메이션 유지. */
	UFUNCTION(BlueprintCallable, Category = "Rope|Ragdoll")
	void EnterPartialRagdoll(FName BoneName);

	/** 애니메이션 복귀. 풀 랙돌이었다면 메시를 원래 부착/상대 트랜스폼으로 되돌린다(의도된 포즈 팝). */
	UFUNCTION(BlueprintCallable, Category = "Rope|Ragdoll")
	void RecoverFromRagdoll();

	UFUNCTION(BlueprintPure, Category = "Rope|Ragdoll")
	bool IsRagdolled() const { return bRagdolled; }

private:
	/** 대상 메시: ACharacter면 GetMesh(), 아니면 owner의 첫 USkeletalMeshComponent. */
	USkeletalMeshComponent* ResolveMesh() const;

	/** 전환 직전 원상복구용 상태 저장(프로파일/부착/상대 트랜스폼). */
	void SaveRestoreState(USkeletalMeshComponent* Mesh);

	//~ 중앙 신호 핸들러(URopeSimSubsystem).
	/** 월드 어느 로프든 wrap 성립 시 — Info.Mesh가 내 메시면 (지연 후) 자동 랙돌 예약. */
	void HandleAnyRopeWrapped(const FRopeWrappedEventInfo& Info);
	/** 월드 어느 로프든 release 시 — 내 메시가 풀렸고 자동 랙돌이었으면 복귀(+예약 취소). */
	void HandleAnyRopeReleased(const USceneComponent* WrappedMesh, FName Bone, ERopeReleaseReason Reason);

	/** RagdollOnWrappedDelay 만료 시 실제 전환(예약된 본은 PendingWrappedBone). */
	void FireAutoRagdoll();

	bool bRagdolled = false;
	bool bPartial = false;
	// 현재 랙돌이 wrap 자동 전환으로 들어간 것인가(수동/치트 진입과 구분 — 자동 복귀 대상 게이트).
	bool bRagdollWasAutoTriggered = false;
	// 자동 전환 예약 시점에 감긴 본(부분 랙돌 대상). FireAutoRagdoll이 소비.
	FName PendingWrappedBone = NAME_None;
	FTimerHandle AutoRagdollTimer;

	FName SavedCollisionProfile = NAME_None;
	FTransform SavedMeshRelative = FTransform::Identity;
	TWeakObjectPtr<USceneComponent> SavedAttachParent;
	FName SavedAttachSocket = NAME_None;
	TEnumAsByte<ECollisionEnabled::Type> SavedCapsuleCollision = ECollisionEnabled::QueryAndPhysics;

	// 신호 구독 핸들(EndPlay 해제용).
	FDelegateHandle WrappedHandle;
	FDelegateHandle ReleasedHandle;
};
