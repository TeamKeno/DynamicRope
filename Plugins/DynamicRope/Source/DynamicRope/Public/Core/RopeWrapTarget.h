// Copyright Epic Games, Inc. All Rights Reserved.
//
// 랩 대상 추상화(Wrap Target Abstraction).
// "랩 대상 = 스켈레탈 메시의 본 하나"라는 가정을 코드에서 뽑아내, 정적 메시 랩(피드백 5번)과
// 본 그룹 랩(피드백 3번)이 공유하는 공통 기반을 제공한다. 설계 근거: Notion "랩 대상 추상화 설계 초안".
//
// 두 개의 seam으로 분해된다:
//   A. 바인딩(binding)     — 앵커 하나가 매 프레임 "무엇"을 따라가는가.
//                            FRopeBindingFrame + ResolveBindingWorld.  (Hold/BeginWrap이 소비)
//   B. 집계(aggregation)   — 접촉 집계가 노드를 "무슨 단위"로 묶는가.
//                            FRopeWrapTargetKey + IRopeWrapTargetRegistry.
//                            (소비자: 현행 런타임은 Contacting 트래커, DecideWrap은 유닛테스트 기준점)
//
// [배선 상태] seam A(ResolveBindingWorld)는 랩 경로 전체(Hold/BeginWrap/경로 빌드/앵커 복원/프리뷰)에
// 배선 완료. 스켈레톤 구조 질의(RopeWrapTargets:: — 부모/자식 키, 스켈레탈 판별)도 랩 경로의 인라인
// Cast를 대체해 배선됐다 — 랩 흐름의 스켈레탈 가정은 이 파일 쌍(.h/.cpp)에만 존재한다.
// seam B(FRopeWrapTargetKey + IRopeWrapTargetRegistry)는 미배선 — 본 그룹(피드백 3번)/디자이너 축
// 요구가 구체화되면 RopeWrapTargets:: 구현 내부를 registry 조회로 교체한다(설계 초안 Increment 2~4).

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtrTemplates.h"
#include "Core/RopeTypes.h"

class USceneComponent;

/**
 * 앵커/랩 노드가 "무엇에 붙어 매 프레임 따라가는가"를 기술하는 바인딩 프레임(POD — 핫 루프 규칙 준수).
 * 지금 코드 곳곳에 흩어진 Mesh->GetSocketTransform(Bone) 을 이 한 타입 + ResolveBindingWorld 로 대체한다.
 * Component 는 weak — cross-actor 대상이 파괴되면 안전하게 null 이 된다(기존 FRopeWrapState::Mesh 규칙 계승).
 */
struct FRopeBindingFrame
{
	/**
	 * 붙는 대상 컴포넌트. 스켈레탈이면 USkeletalMeshComponent 로 다운캐스트해 소켓(스키닝) 트랜스폼을,
	 * 그 외(정적/무버블 프롭)면 컴포넌트/소켓 트랜스폼을 쓴다. ResolveBindingWorld 가 종류를 판별한다.
	 */
	TWeakObjectPtr<const USceneComponent> Component = nullptr;

	/** 스켈레탈: 본/소켓 이름. 정적: 소켓이 있으면 그 이름, 없으면 None(= 컴포넌트 트랜스폼). */
	FName SocketOrBone = NAME_None;

	/** 유효한 트랜스폼을 낼 수 있는가(대상 파괴 감지). 호출자는 false면 release 한다. */
	bool IsValid() const { return Component.IsValid(); }
};

/**
 * 바인딩 프레임 → 이번 프레임 월드 트랜스폼. 랩 경로의 Mesh->GetSocketTransform(Bone) 을 통째로 대체하는
 * 단일 해석 지점이다. 스켈레탈이면서 본 이름이 있으면 스키닝된 소켓 트랜스폼(기존 경로와 100% 동일),
 * 그 외면 컴포넌트/소켓 트랜스폼(정적은 트랜스폼 불변이라 자동으로 "안 움직이는 hold", 무버블 프롭은 추종).
 * Component 가 유효하지 않으면 Identity 를 반환한다(호출자는 IsValid()로 먼저 걸러 release 하는 것을 권장).
 */
DYNAMICROPE_API FTransform ResolveBindingWorld(const FRopeBindingFrame& Frame);

/**
 * 위와 동일한 해석의 raw 포인터 오버로드 — (Mesh, Bone)을 이미 들고 있는 호출자(경로 빌드/앵커 배치 등
 * 프레임 내 다회 호출)가 weak 프레임을 만들지 않고 직접 쓴다. null Component 는 Identity.
 */
DYNAMICROPE_API FTransform ResolveBindingWorld(const USceneComponent* Component, FName SocketOrBone);

/**
 * 랩 대상의 스켈레톤 *구조* 질의 모음. 랩 흐름(감김 축/본 그래프/분류)이 대상 종류를 직접 Cast로
 * 판별하는 대신 이 질의를 쓴다 — 스켈레탈 가정이 이 구현 파일 하나에 격리된다.
 * [확장 지점] 본 그룹(피드백 3번)/디자이너 전환 edge가 들어오면 이 함수들 내부를
 * IRopeWrapTargetRegistry 조회로 교체한다 — 호출부는 그대로.
 */
namespace RopeWrapTargets
{
	/** 대상이 스켈레탈(본 그래프 보유)인가. 정적/가상 본 대상은 false. */
	DYNAMICROPE_API bool IsSkeletalTarget(const USceneComponent* Mesh);

	/** 대상 키(본)의 부모 키. 스켈레탈 = 부모 본 이름, 그 외(정적/가상 본/루트) = None(그래프 없음). */
	DYNAMICROPE_API FName GetParentTargetKey(const USceneComponent* Mesh, FName Bone);

	/**
	 * 대상 키(본)의 자식 키들을 OutChildren에 append. 스켈레탈 = 스켈레톤에서 부모가 Bone인 본 전부,
	 * 그 외 = 없음. SurfaceVectorField 본 그래프 확장(bounded Dijkstra)의 이웃 열거에 쓴다.
	 */
	DYNAMICROPE_API void AppendChildTargetKeys(const USceneComponent* Mesh, FName Bone, TArray<FName>& OutChildren);
}

/**
 * 접촉 집계가 노드를 묶는 단위(POD). 기존엔 FName Bone 하나였다.
 *   단일 본   : Name = 그 본 이름       (기존과 동일)
 *   정적 opt-in: Name = 합성(가상) 본 이름 (5번)
 *   본 그룹   : Name = 그룹 이름         — 그룹에 속한 여러 본이 같은 Key 로 접힌다(3번)
 * TMap 키로 쓰이므로 == 와 GetTypeHash 를 제공한다(FName 위임).
 */
struct FRopeWrapTargetKey
{
	FName Name = NAME_None;

	bool IsValid() const { return !Name.IsNone(); }
	bool operator==(const FRopeWrapTargetKey& Other) const { return Name == Other.Name; }
	bool operator!=(const FRopeWrapTargetKey& Other) const { return Name != Other.Name; }

	friend uint32 GetTypeHash(const FRopeWrapTargetKey& Key) { return GetTypeHash(Key.Name); }
};

/**
 * "무엇이 wrap 가능하고, 접촉이 어느 대상으로 묶이는가"를 선언하는 레지스트리.
 * 로프가 아니라 "감기는 대상" 액터가 provider 로 소유/등록한다 — cross-actor 랩이라 로프는 자신이 어떤
 * 스켈레톤에 맞을지 미리 모르기 때문이다(콜라이더 provider 와 동일한 lifecycle / gather 경로).
 * 피드백 5번(정적 opt-in)과 3번(본 그룹 선언)의 디자이너 opt-in 이 전부 여기 모인다.
 */
class DYNAMICROPE_API IRopeWrapTargetRegistry
{
public:
	virtual ~IRopeWrapTargetRegistry() = default;

	/**
	 * 이 접촉을 wrap 대상으로 인정하는가? 인정하면 집계 키 + 이 노드의 바인딩 프레임을 채우고 true.
	 *   스켈레탈 단일 본 : OutKey.Name = Contact.Bone,   OutFrame = { SourceMesh, Contact.Bone }
	 *   스켈레탈 + 그룹  : OutKey.Name = 그룹명,          OutFrame = { SourceMesh, Contact.Bone(각자 유지) }
	 *   정적 opt-in      : OutKey.Name = 가상 본 이름,    OutFrame = { 정적 컴포넌트, 소켓? }
	 *   정적 비-opt-in   : false(충돌만 되고 랩 불가 — 기존 동작 유지)
	 */
	virtual bool ResolveTarget(const FRopeContact& Contact,
		FRopeWrapTargetKey& OutKey, FRopeBindingFrame& OutFrame) const = 0;

	/**
	 * 대상의 "대표" 바인딩 프레임 — Wrapping 경로 빌드(FRopeWrappingPhase)의 축/원점 기준으로 쓴다.
	 * 단일 본이면 그 본 프레임을 그대로, 그룹이면 멤버 본들의 합성(두 캡슐 convex hull 실린더 축, 3번)을
	 * 반환한다. Hold 는 이걸 쓰지 않는다(앵커가 각자 자기 Binding 프레임으로 따라감) — 경로 형상 유도 전용.
	 */
	virtual FRopeBindingFrame GetRepresentativeFrame(const FRopeWrapTargetKey& Key) const = 0;
};
