// Copyright Epic Games, Inc. All Rights Reserved.
//
// 정적 메시 랩 opt-in(피드백 5번). 랩 가능한 정적/무버블 액터(기둥·가로등·갈고리 등)에 붙이는
// 마커 겸 collider provider. 매 프레임 대상 지오메트리 주위에 "랩 가능한" 해석적 캡슐 하나를 만들어
// 서빙한다 — 이 캡슐은 IsWorldStatic()=false라(정적 월드 push-out 콜라이더와 달리) 접촉 감지(detect)
// 파이프라인에 포함되고, 합성(가상) 본 이름 + SourceMesh(=대상 컴포넌트)를 보고해 기존 DecideWrap
// 경로를 그대로 탄다(FRopeContact FROZEN 계약 준용 — 필드 추가 없음).
//
// 감긴 뒤에는 앵커가 대상 컴포넌트 트랜스폼을 따라간다(ResolveBindingWorld의 정적 분기) — 정적은
// 불변이라 hold가 단순하고, 무버블 프롭(엘리베이터 기둥 등)도 컴포넌트 추종으로 공짜 지원된다.
//
// 스코프(v1): 대상 컴포넌트 1개당 축정렬 캡슐 1개(기둥류가 주 타깃이므로 충분). 스태틱 메시 SDF
// 베이크·복합 형상은 후속. 그룹(양쪽 다리)은 별개 작업(피드백 3번).

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Collision/RopeColliderProvider.h"
#include "Collision/RopeCollider.h" // FCapsuleCollider(값 멤버)
#include "RopeWrapTargetComponent.generated.h"

class USceneComponent;

/** 랩 캡슐 장축(대상 컴포넌트 로컬). EAxis::Type은 UENUM이 아니라 UPROPERTY로 못 쓰므로 전용 enum. */
UENUM(BlueprintType)
enum class ERopeWrapAxis : uint8
{
	X,
	Y,
	Z
};

UCLASS(ClassGroup = (DynamicRope), meta = (BlueprintSpawnableComponent))
class DYNAMICROPE_API URopeWrapTargetComponent : public UActorComponent, public IRopeColliderProvider
{
	GENERATED_BODY()

public:
	URopeWrapTargetComponent();

	//~ UActorComponent — RopeSimSubsystem 중앙 레지스트리에 등록/해제(프레임당 1회 중앙 gather).
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	/** 감길 지오메트리(정적/무버블). 비우면 owner의 첫 UStaticMeshComponent, 없으면 루트 컴포넌트. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap Target")
	TObjectPtr<USceneComponent> TargetComponent = nullptr;

	/** 랩 캡슐 반지름(cm). 0 이하이면 대상 컴포넌트 로컬 bounds에서 자동 추정한다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap Target", meta = (ClampMin = "0.0", Units = "cm"))
	float Radius = 0.0f;

	/** 캡슐 장축을 대상 bounds의 최장축으로 자동 선택할지. 끄면 아래 Axis를 쓴다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap Target")
	bool bAutoAxis = true;

	/** 캡슐 장축(대상 컴포넌트 로컬). 기둥=Z(기본), 가로보=X/Y. bAutoAxis가 꺼져 있을 때만 쓴다. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap Target", meta = (EditCondition = "!bAutoAxis"))
	ERopeWrapAxis Axis = ERopeWrapAxis::Z;

	/**
	 * 이 랩 대상의 합성(가상) 본 이름 — FRopeContact::Bone 귀속에 쓰인다(DecideWrap이 이 이름으로 랩을
	 * 건다). 비우면 대상 컴포넌트 이름 기반으로 자동 발급한다. 스태틱 메시의 실재 소켓 이름을 넣으면
	 * hold가 그 소켓을 따르고, 그 외(가상 이름)면 컴포넌트 트랜스폼을 따른다.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Wrap Target")
	FName WrapBoneName = NAME_None;

	//~ IRopeColliderProvider
	virtual void GatherColliders(TArrayView<const FBox> RopeRegions, TArray<IRopeCollider*>& OutColliders) override;
	// 정적 "월드" push-out 프로바이더가 아니다 — 랩 대상이라 detect에 포함되어야 하고, owner 제외 규칙도
	// 스켈레탈 provider와 동일하게 적용받는다. 따라서 기본값(false)을 그대로 둔다.

private:
	// 프레임당 1회 재구성되는 백킹 스토리지. 넘겨준 포인터는 해당 프레임 solve가 끝날 때까지 유효.
	FCapsuleCollider Capsule;
	FName            ResolvedBone = NAME_None;
	uint64           BuiltFrame = static_cast<uint64>(-1);

	// 무버블 프롭 표면 속도용: 이전 프레임 끝점 + 1/dt. 정적이면 InvDeltaTime 0(속도 0)로 남는다.
	FVector PrevA = FVector::ZeroVector;
	FVector PrevB = FVector::ZeroVector;
	bool    bHasPrevEndpoints = false;

	// 대상 컴포넌트 해석(+가상 본 이름 확정). 실패 시 null.
	USceneComponent* ResolveTarget();
	// 대상 로컬 bounds에서 축정렬 랩 캡슐을 만들어 Capsule에 채운다(가상 본 + SourceMesh=대상).
	void BuildCapsule(USceneComponent* Comp);
};
