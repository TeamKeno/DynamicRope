// Fill out your copyright notice in the Description page of Project Settings.
//
// 랙돌 전환 사이드이펙트 "눈으로 확인"용 데모 컴포넌트(7.09 마일스톤 — 정식 기능 아님, 확인부터).
// 랙돌 전환 자체는 플러그인 밖(게임 코드) 책임이라는 계약(RopeRagdollTransitionTests.cpp 주석 참고)에
// 따라 게임 모듈 Demo 폴더에 둔다. 쓸 만하면 플러그인으로 승격.
//
// 로프에 감길 대상(마네킹/동물/드래곤 — ACharacter 또는 스켈레탈 메시를 가진 아무 액터)에 붙이면:
//  - PIE 콘솔 명령으로 즉석 전환(이 컴포넌트가 붙은 모든 액터에 적용):
//      Rope.Demo.Ragdoll            풀 랙돌 토글(랙돌 중이면 복귀)
//      Rope.Demo.Ragdoll spine_01   해당 본 이하만 부분 랙돌
//      Rope.Demo.RagdollRecover     애니메이션 복귀
//      Rope.Demo.RagdollDestroy     대상 액터 파괴(감긴 중 파괴 → 로프 release 확인용)
//  - bRagdollOnWrapped를 켜면 로프가 이 메시를 감은(Wrapped) 뒤 RagdollOnWrappedDelay 후 자동 전환.
//    bOnlyBelowWrappedBone이면 감긴 본 이하만 — 회의의 "감긴 본만 랙돌 전환 토글" 안(案) 미리보기.
//
// PIE 체크리스트(노션 빌드점검2 Track A) 6항목 대응 시나리오:
//  1) 랙돌 catch     : 먼저 Rope.Demo.Ragdoll → 쓰러진 몸에 로프를 던져 wrap되는지
//  2) 전이 프레임    : 먼저 wrap → Rope.Demo.Ragdoll(또는 자동 전환) → 포즈 팝에 로프가 튀는지
//  3) 부분 랙돌      : Rope.Demo.Ragdoll spine_01 (또는 bOnlyBelowWrappedBone) → 캡슐 재빌드 추종
//  4) 실물리 본 Pull : 랙돌 상태 wrap + Pull 홀드 → AddForceAtLocation 분기로 몸이 끌리는지
//  5) 대상 파괴      : wrap 중 Rope.Demo.RagdollDestroy → weak mesh 경로로 안전 release되는지
//  6) 랙돌 복귀      : wrap 중 Rope.Demo.RagdollRecover → 본 스냅백을 로프가 무속도 추종하는지

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RopeRagdollDemoComponent.generated.h"

class URopeComponent;
class USkeletalMeshComponent;

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPEPROJECT_API URopeRagdollDemoComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	URopeRagdollDemoComponent();

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/** 로프가 이 액터의 메시를 감으면(Wrapped) 자동으로 랙돌 전환한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|RagdollDemo")
	bool bRagdollOnWrapped = false;

	/** 자동 전환까지의 지연(초). 감긴 직후 vs 잠시 뒤 전환의 차이를 관찰할 때 조절. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|RagdollDemo", meta = (ClampMin = "0.0", Units = "s", EditCondition = "bRagdollOnWrapped"))
	float RagdollOnWrappedDelay = 0.3f;

	/** 자동 전환 시 풀 랙돌 대신 감긴 본 이하만 부분 랙돌(회의의 "감긴 본만 전환" 안 미리보기). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|RagdollDemo", meta = (EditCondition = "bRagdollOnWrapped"))
	bool bOnlyBelowWrappedBone = false;

	/** 랙돌 동안 메시에 줄 콜리전 프로파일. 마네킹 기본(CharacterMesh)은 물리 충돌이 없어 전환이 필수. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|RagdollDemo")
	FName RagdollCollisionProfileName = TEXT("Ragdoll");

	/** 풀 랙돌 전환: 메시 전체 물리 시뮬 + 캡슐 콜리전/무브먼트 정지(ACharacter일 때). */
	UFUNCTION(BlueprintCallable, Category = "Rope|RagdollDemo")
	void EnterRagdoll();

	/** 부분 랙돌 전환: BoneName 이하 바디만 물리 시뮬(blend weight 1). 나머지는 애니메이션 유지. */
	UFUNCTION(BlueprintCallable, Category = "Rope|RagdollDemo")
	void EnterPartialRagdoll(FName BoneName);

	/** 애니메이션 복귀. 풀 랙돌이었다면 메시를 원래 부착/상대 트랜스폼으로 되돌린다(의도된 포즈 팝 —
	 *  wrap 중이면 로프의 무속도 추종을 관찰하는 체크리스트 6번 재료). */
	UFUNCTION(BlueprintCallable, Category = "Rope|RagdollDemo")
	void RecoverFromRagdoll();

	UFUNCTION(BlueprintPure, Category = "Rope|RagdollDemo")
	bool IsRagdolled() const { return bRagdolled; }

private:
	/** 대상 메시: ACharacter면 GetMesh(), 아니면 owner의 첫 USkeletalMeshComponent. */
	USkeletalMeshComponent* ResolveMesh() const;

	/** 전환 직전 원상복구용 상태 저장(프로파일/부착/상대 트랜스폼). */
	void SaveRestoreState(USkeletalMeshComponent* Mesh);

	/** 이 메시를 Wrapped로 감고 있는 로프를 찾는다(없으면 null). OutBone = 감긴 본. */
	URopeComponent* FindRopeWrappingUs(FName& OutWrappedBone) const;

	bool bRagdolled = false;
	bool bPartial = false;

	FName SavedCollisionProfile = NAME_None;
	FTransform SavedMeshRelative = FTransform::Identity;
	TWeakObjectPtr<USceneComponent> SavedAttachParent;
	FName SavedAttachSocket = NAME_None;
	TEnumAsByte<ECollisionEnabled::Type> SavedCapsuleCollision = ECollisionEnabled::QueryAndPhysics;

	// 자동 전환 대기 누적 시간(<0 = 대기 아님). 감김이 풀리면 리셋.
	float PendingAutoRagdollTime = -1.0f;
	// wrap 1회당 자동 전환 1회(복귀 후 같은 wrap에서 재발화 방지).
	bool bAutoFiredThisWrap = false;
};
