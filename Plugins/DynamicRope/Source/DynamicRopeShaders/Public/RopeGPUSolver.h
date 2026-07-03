// Copyright Epic Games, Inc. All Rights Reserved.
//
// XPBD 로프 솔버의 GPU(compute) 구현 — M5: 센터라인 GPU 상주(resident).
// 위치 버퍼를 프레임 간 영속시켜 매 프레임 GPU에서 in-place로 전진한다 → 순차 의존성이 GPU 안에서
// 충족되어 라운드트립 스톨/슬로모가 없다(M4 비동기 리드백의 한계 해소). 리드백(렌더/충돌용)은 전부
// 렌더 스레드에서 처리(매 프레임 step 커맨드가 직전 리드백을 Lock·consume 후 재무장) → GT 스톨 없음.
// CPU FRopeXPBDSolver와 동일 수식이되 제약은 red-black/stride-3 컬러링으로 푼다(병렬 안전).
// 이 모듈(DynamicRopeShaders)은 DynamicRope 런타임 타입에 의존하지 않는다 → step은 self-contained POD.
// 호출자(런타임 서브시스템)가 FRopeSimState/Config로부터 step을 채워 넘긴다.

#pragma once

#include "CoreMinimal.h"

/** GPU 충돌(M2)용 해석적 capsule. 월드 공간 세그먼트(A-B) + 반지름. 호출자가 collider에서 추출해 채운다. */
struct FRopeGPUCapsule
{
	FVector A = FVector::ZeroVector;
	FVector B = FVector::ZeroVector;
	float   Radius = 0.0f;
};

/**
 * GPU 충돌(M3)용 per-bone SDF collider. 본 로컬 distance grid + 본→월드 트랜스폼.
 * Distances는 호출자(에셋) 소유 포인터(Step 호출 동안 유효 — 렌더 커맨드로 옮기기 전 GT에서 복사된다).
 * Distances는 uint8 양자화 코드 — GT 평탄화 시 비대칭 밴드로 dequant해 float 버퍼로 업로드한다.
 * VolumeKey가 같으면 같은 step 내에서 GPU 업로드를 공유(dedup)한다.
 */
struct FRopeGPUSDFCollider
{
	const uint8* Distances = nullptr;    // 코드 바이트 블롭(복셀당 BytesPerCode, 행 우선, 리틀엔디안). 바깥 +.
	int32        BytesPerCode = 1;       // 복셀당 바이트(1=uint8 max255, 2=uint16 max65535).
	float        NarrowBandInner = 0.0f; // 안쪽 dequant 밴드(cm). 코드 0 → -NarrowBandInner.
	float        NarrowBandOuter = 0.0f; // 바깥 dequant 밴드(cm). 코드 max → +NarrowBandOuter.
	int32        ResX = 0;
	int32        ResY = 0;
	int32        ResZ = 0;
	FVector      LocalMin = FVector::ZeroVector;
	FVector      LocalSize = FVector::ZeroVector;
	FTransform   BoneToWorld = FTransform::Identity;
	FTransform   PrevBoneToWorld = FTransform::Identity; // 이전 프레임 본 트랜스폼(CCD/표면속도 드래그).
	float        InvDeltaTime = 0.0f;                    // 1/프레임dt(표면 속도용). 0이면 정적.
	const void*  VolumeKey = nullptr;
};

/**
 * FRopeGPUResidentStep::OverrideFlags의 노드별 비트(G0). RopeXPBD.usf의 override 스테이지와 1:1.
 * "타깃 계산은 GT, 적용은 GPU" — 로직 페이즈(whip/wrapping/hold/releasing)가 계산한 노드별
 * 타깃/질량을 재시드 없이 상주 버퍼에 직접 기록하는 통로다. 적용 순서: Position → Prev →
 * PrevFromPosition → InvMass (PrevFromPosition은 Position 적용 *후*의 Pos를 복사한다).
 */
enum class ERopeGPUOverride : uint8
{
	None             = 0,
	Position         = 1 << 0, // Pos[i]  = OverridePositions[i]
	Prev             = 1 << 1, // Prev[i] = OverridePrevPositions[i] (Pos와의 차이가 Verlet 속도가 된다 — whip)
	PrevFromPosition = 1 << 2, // Prev[i] = Pos[i] — 속도 0 고정(wrapping/hold). GPU측 현재 Pos 기준(CPU 미러 아님).
	InvMass          = 1 << 3, // InvMass[i] = OverrideInvMass[i] — 상주 InvMass 버퍼에 영속(질량 마스크/복원)
};
ENUM_CLASS_FLAGS(ERopeGPUOverride)

/**
 * 상주 로프 1개의 한 프레임 step 입력. self-contained(전부 값/TArray) — GT에서 채워 렌더 스레드로 MoveTemp.
 * RopeId는 영속 버퍼를 식별하는 안정 키(예: 컴포넌트 UniqueID). Generation은 throw/리사이즈 등 CPU가 Sim을
 * out-of-band로 바꿨을 때 증가시킨다 → RT가 generation 변화/노드수 변화/최초를 감지해 GPU 버퍼를 재시드한다.
 * SeedPositions/PrevPositions/InvMass는 매 프레임 제공하되 RT는 재시드가 필요할 때만 실제 업로드한다(평시 무시).
 */
struct FRopeGPUResidentStep
{
	uint32 RopeId = 0;
	uint32 Generation = 0;
	int32  NumNodes = 0;

	// 시드 데이터(매 프레임 제공; RT는 재시드 시에만 GPU 업로드).
	TArray<FVector> SeedPositions;
	TArray<FVector> SeedPrevPositions;
	TArray<float>   InvMass;

	// sim / config 스칼라.
	float   SegmentLength = 0.0f;
	bool    bStartPinned = false;
	FVector StartPinPrev = FVector::ZeroVector;
	FVector StartPinTarget = FVector::ZeroVector;
	float   StretchCompliance = 0.0f;
	float   BendCompliance = 0.0f;
	float   Damping = 0.0f;
	int32   Iterations = 1;
	int32   CollisionPasses = 1; // substep당 충돌 해소 패스 수(Iterations로 상한). 1=substep 끝 1회(기존).
	FVector Gravity = FVector::ZeroVector;

	// 충돌(M2/M3). 이 로프에 적용할 collider 목록(값 복사라 step 수명 동안 유효).
	float CollisionRadius = 0.0f; // 로프 노드 두께(= FRopeSolverConfig::CollisionRadius).
	float Friction = 0.0f;        // 접선 감쇠 [0..1](Coulomb μ).
	float TipFrictionScale = 1.0f; // 자유단 마찰 배율(고정점=1, 끝=이 값). 끝 노드를 잘 놔주게 함.
	float SweepStep = 2.0f;       // swept 샘플 간격(cm).
	int32 MaxSweepSamples = 16;   // 세그먼트당 샘플 상한.
	TArray<FRopeGPUCapsule>     Capsules;
	TArray<FRopeGPUSDFCollider> SDFColliders;

	// 이번 프레임 substep 스케줄(호출자가 RopeSolverSubsteps로 계산해 전달). NumSub<=0이면 적분 없이 유지.
	int32 NumSub = 0;
	float FixedDt = 0.0f;

	// --- Override(G0): 로직 페이즈(GT)가 계산한 노드별 타깃을 상주 버퍼에 직접 기록(재시드 대체).
	// 비어 있으면 오버라이드 없음. 채울 때 OverrideFlags는 정확히 NumNodes 길이(불일치 시 전체 무시+경고),
	// 값 배열은 해당 비트를 쓰는 노드가 있을 때만 NumNodes 길이로 제공하면 된다.
	// NumSub=0이어도 오버라이드가 있으면 dispatch되어 적분 없이 기록만 한다(예: Wrapping/Releasing 프레임).
	TArray<uint8>   OverrideFlags;         // 노드별 ERopeGPUOverride 비트 OR
	TArray<FVector> OverridePositions;     // Position 비트 노드만 유효
	TArray<FVector> OverridePrevPositions; // Prev 비트 노드만 유효
	TArray<float>   OverrideInvMass;       // InvMass 비트 노드만 유효

	bool HasOverrides() const { return OverrideFlags.Num() > 0; }
};

/** GT가 회수하는 상주 로프의 최신(약간 지연) 위치. RT 리드백이 채우고 GT가 락 하에 복사한다. */
struct FRopeResidentLatest
{
	TArray<FVector> Positions;
	TArray<FVector> PrevPositions;
	uint32 Generation = 0; // 이 위치가 대응하는 시드 generation(재시드 경계의 stale 적용 방지).
	int32  NumNodes = 0;
};

/**
 * XPBD 로프 솔버의 GPU(compute) 구현. 센터라인 GPU 상주(M5a).
 *  - Step      : 로프별 영속 GPU 버퍼를 매 프레임 in-place로 한 프레임 전진(라운드트립/스톨 없음).
 *                필요 시(최초/노드수·generation 변화) CPU Sim에서 재시드. 리드백은 RT에서 consume+재무장.
 *  - GetLatest : RT 리드백이 채운 최신 위치(약 1~2프레임 지연)를 락 하에 복사. 렌더/충돌용.
 *  - ReleaseRope: 로프 영속 버퍼/리드백 해제(EndPlay 등).
 * 인스턴스 상태(영속 버퍼 맵)는 렌더 스레드 소유 — GT 메서드는 렌더 커맨드를 enqueue하거나 공유 결과를 읽는다.
 * 월드별 1개를 소유한다. CPU 솔버는 ground-truth로 유지.
 */
class FRHIGPUBufferReadback;
class FRHIShaderResourceView;

class DYNAMICROPESHADERS_API FRopeGPUSolver
{
public:
	/** 로프당 최대 노드 수(= compute 스레드그룹 크기). 호출자는 이 한도 내 step만 넘겨야 한다. */
	static constexpr int32 MaxNodes = 256;

	FRopeGPUSolver();
	~FRopeGPUSolver();

	/**
	 * 렌더 스레드. 로프의 resident PosBuf(StructuredBuffer<float4>, 월드 위치) SRV를 반환(없으면 null).
	 * M5b B2-lite: scene proxy가 이 SRV를 직접 읽어 튜브를 GPU 생성 → 위치 무지연(렌더 리드백 없음).
	 * 솔버가 이 로프를 step한 적이 없으면(= GPU 솔버 off) null → 호출자는 CPU 경로로 폴백한다.
	 */
	FRHIShaderResourceView* GetResidentPositionSRV_RenderThread(uint32 RopeId, int32& OutNumNodes);

	/** 이번 프레임 상주 step들을 렌더 스레드로 넘겨 GPU에서 in-place 전진(블록 없음). step은 소비된다(MoveTemp). */
	void Step(TArray<FRopeGPUResidentStep>&& Steps);

	/** RT 리드백이 채운 최신 위치를 RopeId별로 복사(락). 새로 도착한 게 없으면 직전 값을 유지한 채 반환할 수 있다. */
	void GetLatest(TMap<uint32, FRopeResidentLatest>& Out);

	/**
	 * GT 블로킹 동기 리드백(M5c): 이 로프의 상주 Pos/Prev를 *지금* 값으로 가져온다(GPU idle 대기 포함).
	 * wrap 핸드오프처럼 "이벤트당 1회, 최신 위치가 꼭 필요한" 곳 전용 — 매 프레임 호출 금지.
	 * OutGeneration은 버퍼가 대응하는 시드 generation(호출자가 자기 generation과 대조해 stale 거부).
	 * @return 상주 버퍼가 있고 회수에 성공하면 true.
	 */
	bool ReadbackNow(uint32 RopeId, TArray<FVector>& OutPositions, TArray<FVector>& OutPrevPositions, uint32& OutGeneration);

	/** 로프의 영속 버퍼/리드백을 해제(렌더 스레드에서). 컴포넌트 EndPlay/Unregister에서 호출. */
	void ReleaseRope(uint32 RopeId);

private:
	// 상주 상태(렌더 스레드 전용 영속 버퍼 맵 + GT<->RT 공유 결과)를 pimpl로 숨긴다 — 헤더에 RDG/RHI 타입을
	// 노출하지 않고, 불완전 타입을 멤버로 by-value 보관할 때의 sizeof 요구도 피한다(포인터 멤버).
	struct FImpl;
	TUniquePtr<FImpl> Impl;

	void ReleaseAll_RenderThread();
};
