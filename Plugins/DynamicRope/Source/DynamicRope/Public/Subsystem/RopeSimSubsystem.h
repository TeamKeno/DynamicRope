// Copyright Epic Games, Inc. All Rights Reserved.
//
// 모든 활성 URopeComponent의 시뮬레이션을 한 곳에서 구동하는 world subsystem. 컴포넌트가 각자
// tick하던 것을 대체하는 단일 오케스트레이션 지점이다. 현재는 순차 구동만 — 추후 이 위에
// collider gather 디둡 / ParallelFor 병렬 솔브 / LOD·sleep / 프레임당 예산 상한을 얹는다.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Engine/EngineBaseTypes.h" // FTickFunction (TG_PostPhysics 틱)
#include "RopeGPUSolver.h" // FRopeGPUSolver (DynamicRopeShaders): 비동기 GPU 솔브 인스턴스
#include "RopeSimSubsystem.generated.h"

class URopeComponent;
class UActorComponent;
class USkeletalMeshComponent;
class IRopeCollider;
class AActor;

/**
 * 서브시스템 Tick을 TG_PostPhysics에서 구동하는 틱 함수(기존 tickable 대체).
 * tickable(TickObjects)은 엔진의 호출 위치(현재 TG_PostPhysics 뒤)에 묵시적으로 얹혀 있었다 — 명시
 * 그룹 + 스켈레탈 메시 틱 선행조건으로 "본 트랜스폼(애니 평가) 이후 로프 시뮬" 순서를 계약으로 만든다.
 * 같은 그룹 내 순서는 선행조건이 담당: 메시 틱 완료는 병렬 애니 완료 태스크를 DontCompleteUntil로
 * 물고 있어(SkeletalMeshComponent), 선행조건만으로 포즈 버퍼 플립(최신 포즈)까지 보장된다.
 */
USTRUCT()
struct FRopeSimTickFunction : public FTickFunction
{
	GENERATED_BODY()

	// 대상 서브시스템(월드 수명). 틱 함수는 OnWorldBeginPlay~Deinitialize 동안만 등록된다.
	class URopeSimSubsystem* Target = nullptr;

	virtual void ExecuteTick(float DeltaTime, ELevelTick TickType, ENamedThreads::Type CurrentThread,
		const FGraphEventRef& MyCompletionGraphEvent) override;
	virtual FString DiagnosticMessage() override;
	virtual FName DiagnosticContext(bool bDetailed) override;
};

template <>
struct TStructOpsTypeTraits<FRopeSimTickFunction> : public TStructOpsTypeTraitsBase2<FRopeSimTickFunction>
{
	enum { WithCopy = false };
};

UCLASS()
class DYNAMICROPE_API URopeSimSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	/** 활성 로프를 시뮬레이션 목록에 등록/해제한다(컴포넌트 BeginPlay/EndPlay에서 호출). */
	void RegisterRope(URopeComponent* Rope);
	void UnregisterRope(URopeComponent* Rope);

	/**
	 * collider provider(IRopeColliderProvider를 구현한 UActorComponent)를 중앙 레지스트리에 등록/해제한다
	 * (provider BeginPlay/EndPlay에서 호출). 로프마다 월드를 스캔하던 것을 대체 — 프레임당 1회 중앙 빌드.
	 */
	void RegisterColliderProvider(UActorComponent* Provider);
	void UnregisterColliderProvider(UActorComponent* Provider);

	/** 월드의 rope sim subsystem(게임/PIE 월드에서 유효, 그 외엔 nullptr). */
	static URopeSimSubsystem* Get(const UWorld* World);

	/** GPU 상주 솔버 포인터(월드 수명). M5b: scene proxy가 resident PosBuf SRV를 가져오는 데 쓴다. */
	FRopeGPUSolver* GetGpuSolver() { return &GpuSolver; }

	/**
	 * wrap 핸드오프 정밀 동기(M5c): GPU 상주 로프의 CPU 미러(Sim)는 1~2프레임 낡으므로,
	 * Wrapping 진입 순간 1회 동기 리드백으로 최신 위치를 Sim에 반영한다(시드 정밀도 확보).
	 * GPU 솔버가 꺼져 있거나 상주 버퍼가 없거나 generation이 어긋나면 no-op(false).
	 * 블로킹(GPU idle 대기) — 이벤트당 1회 용도로만 호출할 것.
	 */
	bool SyncGpuPositionsForHandoff(URopeComponent& Rope);

	/** 프레임 시뮬 구동 — FRopeSimTickFunction이 TG_PostPhysics에서 호출한다(테스트는 직접 호출 가능). */
	void Tick(float DeltaTime);

	//~ UWorldSubsystem
	virtual bool DoesSupportWorldType(const EWorldType::Type WorldType) const override;
	//~ 틱 함수 등록 + 씬→솔버 등록(GDF 통합 경로에서 뷰 확장이 솔버를 찾는 용도). 해제는 Deinitialize.
	virtual void OnWorldBeginPlay(UWorld& InWorld) override;
	virtual void Deinitialize() override;

private:
	// TG_PostPhysics 틱 함수(월드 BeginPlay~Deinitialize 동안 등록). 선행조건은 아래 SetAnimPrerequisites가 관리.
	FRopeSimTickFunction SimTickFunction;

	// 소스 컴포넌트(로프/provider) 소유 액터의 스켈레탈 메시 틱을 SimTickFunction 선행조건으로 등록/해제한다.
	// 등록·해제 사이에 액터의 메시 구성이 바뀌어 잔여 항목이 남아도 FTickPrerequisite는 weak라 무해(스킵됨).
	void SetAnimPrerequisites(const UActorComponent* Source, bool bAdd);

	// 등록된 활성 로프(컴포넌트는 UObject → GC 추적).
	UPROPERTY(Transient)
	TArray<TObjectPtr<URopeComponent>> Ropes;

	// 등록된 collider provider(IRopeColliderProvider 구현 컴포넌트). GC 추적.
	UPROPERTY(Transient)
	TArray<TObjectPtr<UActorComponent>> ColliderProviders;

	// OnWorldBeginPlay에서 자동 스폰한 로프 매니저 액터(정적 월드 충돌 프로바이더 호스트). 세팅
	// StaticBodyControllerClass가 None이면 null(자동 스폰 opt-out). Deinitialize에서 파괴한다.
	UPROPERTY(Transient)
	TObjectPtr<AActor> SpawnedStaticBodyController = nullptr;

	// 프레임당 1회 중앙 빌드한 collider(provider별 소유 액터 + collider 포인터). 포인터는 provider 소유라 해당 프레임만 유효.
	struct FFrameProviderColliders
	{
		AActor* Owner = nullptr;          // 소스 필터링용(provider 컴포넌트의 owner 액터).
		bool bWorldStatic = false;        // 정적 월드 provider — 로프별 소유자 제외 면제(ProvidesWorldStaticColliders).
		TArray<IRopeCollider*> Colliders; // provider->GatherColliders가 채운 포인터(provider 백킹 스토리지를 가리킴).
		TArray<FBox> Bounds;              // collider별 월드 bounds 캐시(로프별 거리 컬링용 — 프레임당 1회 계산).
	};
	TArray<FFrameProviderColliders> FrameProviders;

	// 등록된 provider 전부에서 1회 collider를 모은다(Prepare 이전). provider에는 로프별 region 리스트를 넘긴다.
	void BuildFrameColliders();
	// 한 로프의 collider를 중앙 빌드에서 모은다: 기본은 전체, 자기 owner provider만 제외(bIncludeOwnerColliders로 옵트인).
	void GatherCollidersForRope(const URopeComponent& Rope, TArray<IRopeCollider*>& OutColliders) const;
	// 한 로프의 broad-phase 질의 bounds(Pos∪Prev tight AABB + 접촉/예측 마진). provider에 넘기는 region과
	// per-rope collider 컬링이 동일 박스를 쓰도록 한 곳에서 계산한다(무효면 !IsValid 박스 반환).
	static FBox ComputeRopeQueryBounds(const URopeComponent& Rope);

	// GPU 상주 솔버(M5). 영속 버퍼(로프별)를 매 프레임 in-place 전진. 인스턴스 상태라 월드별 1개.
	// G4: 렌더 가능 RHI면 이게 유일 런타임 경로. RHI 없으면(쿡/-nullrhi/서버) CPU 솔버로 자동 폴백.
	FRopeGPUSolver GpuSolver;

	// GetLatest로 회수한 RopeId별 최신(약간 지연) 위치 캐시. 매 프레임 갱신분을 각 Sim에 매핑한다.
	TMap<uint32, FRopeResidentLatest> GpuLatest;

	// GetLatestContacts로 회수한 RopeId별 최신(약간 지연) GPU 접촉 감지 결과(G3). Finalize 전에 귀속.
	TMap<uint32, FRopeResidentContacts> GpuLatestContacts;

	// GPU 감지 결과(콜라이더 인덱스)를 로프의 귀속 테이블로 FRopeContactCandidate로 복원해 컴포넌트에
	// 채운다(Finalize의 Flight 접촉 소스). 지연분이 현재 시드 generation과 맞을 때만 유효.
	void BuildGpuFlightCandidates(URopeComponent& Rope);

	// Phase 2(GPU) 헬퍼 — 한 로프의 GPU 상주 step을 구성한다. GPU 상주 대상이면 OutStep을 채우고 true를
	// 반환(디스패치 목록에 추가), 노드수 초과 등 폴백이면 내부에서 CPU 솔브 후 false. Rope.bGpuSteppedThisFrame도 세팅.
	bool TryBuildResidentStep(URopeComponent& Rope, float DeltaTime, FRopeGPUResidentStep& OutStep);
	// G3: Flight 로프의 접촉 감지 요청(+ whip 예측 입력)을 Step에 세팅하고 귀속 테이블을 리셋한다.
	void RequestContactDetection(URopeComponent& Rope, float DeltaTime, FRopeGPUResidentStep& Step) const;
	// 이 로프의 FrameColliders를 capsule/SDF로 분류해 Step에 싣는다. bDetectThisRope면 귀속 테이블도 병행 채움.
	void PackStepColliders(URopeComponent& Rope, bool bDetectThisRope, FRopeGPUResidentStep& Step) const;
	// G1: Flight whip 가이드 타깃을 override로 Step에 패킹한다(적분 전 적용, 비-Flight면 no-op).
	void PackWhipOverride(const URopeComponent& Rope, FRopeGPUResidentStep& Step) const;
};
