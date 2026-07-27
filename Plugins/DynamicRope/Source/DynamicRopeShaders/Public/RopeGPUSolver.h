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

	/**
	 * 이전 프레임 끝점 + 1/프레임dt(표면 속도 드래그/substep 상대 운동 CCD용 — SDF의 PrevBoneToWorld 대응).
	 * InvDeltaTime=0(기본)이면 정적 — 패킹이 prev=현재로 폴백하므로 안 채워도 기존 동작과 동일.
	 */
	FVector PrevA = FVector::ZeroVector;
	FVector PrevB = FVector::ZeroVector;
	float   InvDeltaTime = 0.0f;
};

/**
 * GPU 충돌용 해석적 박스(OBB). 정적 월드 지오메트리(스태틱 바디 심플 콜리전)가 기본이고, 움직이는
 * 바디/랩 가능 박스는 아래 프레임 모션(PrevCenter/PrevRot + InvDeltaTime)으로 표면 속도·CCD에 참여한다.
 * 모서리/엣지에서 정확한 대각 normal을 주는 해석적 질의가 존재 이유(GDF 복셀 라운딩 관통 대체).
 * 호출자가 IRopeCollider::GetGPUBox(+GetGPUBoxMotion)로 추출해 채운다.
 */
struct FRopeGPUBox
{
	/** 월드 공간 박스 중심. */
	FVector Center = FVector::ZeroVector;
	/** 월드 공간 박스 회전. */
	FQuat   Rot = FQuat::Identity;
	/** 로컬 반폭(스케일 반영 후). */
	FVector HalfExtents = FVector::ZeroVector;

	/**
	 * 이전 프레임 center/rot + 1/프레임dt(움직이는 바디 표면 속도/substep CCD). InvDeltaTime=0이면 정적 —
	 * 패킹이 prev=현재로 채우므로 안 채워도 기존 동작과 동일(캡슐의 Prev* 대응).
	 */
	FVector PrevCenter = FVector::ZeroVector;
	FQuat   PrevRot = FQuat::Identity;
	float   InvDeltaTime = 0.0f;
};

/**
 * GPU 충돌용 해석적 컨벡스(평면 집합). 정적/동적 월드 지오메트리(convex 심플 콜리전 + 전단 박스).
 * 평면은 Step의 ConvexPlanes 평탄 풀 [PlaneOffset, PlaneOffset+PlaneCount)에 저장(바디-로컬, 단위
 * 법선·바깥, PlaneDot(p)=dot(N,p)-W, 강체 미적용). 월드 = 로컬 ∘ 강체(Rot,Trans). LocalBounds는 로컬
 * AABB(질의 컬). 움직이는 바디는 prev 강체 + InvDeltaTime으로 표면 속도/CCD 처리. 호출자가 GetGPUConvex로 추출.
 */
struct FRopeGPUConvex
{
	/** ConvexPlanes 풀 내 시작 인덱스. */
	int32   PlaneOffset = 0;
	/** 평면 수. */
	int32   PlaneCount = 0;
	/** 바디-로컬 AABB 중심. */
	FVector LocalBoundsCenter = FVector::ZeroVector;
	/** 바디-로컬 AABB 반크기. */
	FVector LocalBoundsExtent = FVector::ZeroVector;
	/** 강체 회전(curr). */
	FQuat   Rot = FQuat::Identity;
	/** 강체 평행이동(curr). */
	FVector Trans = FVector::ZeroVector;
	/** 강체 회전(prev). */
	FQuat   PrevRot = FQuat::Identity;
	/** 강체 평행이동(prev). */
	FVector PrevTrans = FVector::ZeroVector;
	/** 1/프레임dt(0이면 정적). */
	float   InvDeltaTime = 0.0f;
};

/**
 * GPU 충돌(M3)용 per-bone SDF collider. 본 로컬 distance grid + 본→월드 트랜스폼.
 * Distances는 호출자(에셋) 소유 포인터(Step 호출 동안 유효 — 렌더 커맨드로 옮기기 전 GT에서 복사된다).
 * Distances는 uint8 양자화 코드 — GT 평탄화 시 비대칭 밴드로 dequant해 float 버퍼로 업로드한다.
 * VolumeKey가 같으면 같은 step 내에서 GPU 업로드를 공유(dedup)한다.
 */
struct FRopeGPUSDFCollider
{
	/** 코드 바이트 블롭(복셀당 BytesPerCode, 행 우선, 리틀엔디안). 바깥 +. */
	const uint8* Distances = nullptr;
	/** 복셀당 바이트(1=uint8 max255, 2=uint16 max65535). */
	int32        BytesPerCode = 1;
	/** 안쪽 dequant 밴드(cm). 코드 0 → -NarrowBandInner. */
	float        NarrowBandInner = 0.0f;
	/** 바깥 dequant 밴드(cm). 코드 max → +NarrowBandOuter. */
	float        NarrowBandOuter = 0.0f;
	int32        ResX = 0;
	int32        ResY = 0;
	int32        ResZ = 0;
	FVector      LocalMin = FVector::ZeroVector;
	FVector      LocalSize = FVector::ZeroVector;
	FTransform   BoneToWorld = FTransform::Identity;
	/** 이전 프레임 본 트랜스폼(CCD/표면속도 드래그). */
	FTransform   PrevBoneToWorld = FTransform::Identity;
	/** 1/프레임dt(표면 속도용). 0이면 정적. */
	float        InvDeltaTime = 0.0f;
	uint64       VolumeKey = 0;
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
	// Pos[i]  = OverridePositions[i]
	Position         = 1 << 0,
	// Prev[i] = OverridePrevPositions[i] (Pos와의 차이가 Verlet 속도가 된다 — whip)
	Prev             = 1 << 1,
	// Prev[i] = Pos[i] — 속도 0 고정(wrapping/hold). GPU측 현재 Pos 기준(CPU 미러 아님).
	PrevFromPosition = 1 << 2,
	// InvMass[i] = OverrideInvMass[i] — 상주 InvMass 버퍼에 영속(질량 마스크/복원)
	InvMass          = 1 << 3,
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

	/** 시드 데이터(매 프레임 제공; RT는 재시드 시에만 GPU 업로드). */
	TArray<FVector> SeedPositions;
	TArray<FVector> SeedPrevPositions;
	TArray<float>   InvMass;

	/** sim / config 스칼라. */
	float   SegmentLength = 0.0f;
	bool    bStartPinned = false;
	FVector StartPinPrev = FVector::ZeroVector;
	FVector StartPinTarget = FVector::ZeroVector;
	float   StretchCompliance = 0.0f;
	/** Strain limiting: substep solve 뒤 각 세그먼트를 ≤ 이 배율 × SegmentLength로 하드 투영(1.5=기본,
	 *  <1=비활성). 긴 체인이 앵커 핀에 매달릴 때 iteration 부족으로 생기는 앵커 인접 과신장/지터를 막는다. */
	float   MaxStretchRatio = 1.5f;
	float   BendCompliance = 0.0f;
	/** 각도-허용 벤딩: straightness ≤ 이 값이면 펴는 힘 0(코너/랩 경계 각짐 완화). */
	float   BendReleaseRatio = 0.70f;
	/** straightness ≥ 이 값이면 펴는 힘 100%(완만한 굽힘은 기존처럼 편다). */
	float   BendFullRatio = 0.92f;
	float   Damping = 0.0f;
	int32   Iterations = 1;
	/** substep당 충돌 해소 패스 수(Iterations로 상한). 1=substep 끝 1회(기존). */
	int32   CollisionPasses = 1;
	FVector Gravity = FVector::ZeroVector;

	/**
	 * 충돌(M2/M3). 이 로프에 적용할 collider 목록(값 복사라 step 수명 동안 유효).
	 * false면 solve 커널은 collider/GDF를 무시하지만 detect 커널은 아래 목록을 그대로 사용할 수 있다.
	 */
	bool  bSolveCollisions = true;
	/** 로프 노드 두께(= FRopeSolverConfig::CollisionRadius). */
	float CollisionRadius = 0.0f;
	/** 접선 감쇠 [0..1](Coulomb μ). */
	float Friction = 0.0f;
	/** 자유단 마찰 배율(고정점=1, 끝=이 값). 끝 노드를 잘 놔주게 함. */
	float TipFrictionScale = 1.0f;
	/** swept 샘플 간격(cm). */
	float SweepStep = 2.0f;
	/** 세그먼트당 샘플 상한. */
	int32 MaxSweepSamples = 16;
	/** Phase 2c: 엔진 GDF로 정적 월드 밀어내기(씬 그래프 dispatch에서만 유효). */
	bool  bUseWorldGDF = false;
	TArray<FRopeGPUCapsule>     Capsules;
	TArray<FRopeGPUSDFCollider> SDFColliders;
	/** 해석적 박스(OBB). solve에 사용하며 앞쪽 NumDetectBoxes개는 접촉 감지에도 참여한다. */
	TArray<FRopeGPUBox>         Boxes;
	/** 해석적 컨벡스(평면 집합). solve 전용. */
	TArray<FRopeGPUConvex>      Convexes;
	/** 전 컨벡스의 바디-로컬 평면 평탄 풀((nx,ny,nz,w), 바깥 방향 법선). */
	TArray<FVector4>            ConvexPlanes;

	/**
	 * 접촉 감지(detect) 커널이 볼 capsule 수. Capsules 앞쪽 [0, NumDetectCapsules)만 감지에 참여한다 —
	 * 호출자(PackStepColliders)가 비-정적 캡슐을 앞에, 정적(월드) 캡슐을 뒤에 2-pass로 패킹해 채운다.
	 * 감지는 노드당 최심 접촉 1개만 남기므로, 벽(정적) 접촉이 본(스켈레탈) 접촉을 가려 랩 캡처가
	 * 조용히 실패하는 것을 막는다. -1(기본) = 전부 참여(기존 동작/테스트 호환).
	 */
	int32 NumDetectCapsules = -1;

	/**
	 * 감지 커널이 볼 랩 가능 박스(OBB) 수. Boxes 앞쪽 [0, NumDetectBoxes)만 감지에 참여한다(캡슐과 동일
	 * 2-pass 패킹: 랩 가능 박스 앞, 정적 박스 뒤). 0(기본) = 감지 미참여(정적 박스 전용 — 기존 동작).
	 */
	int32 NumDetectBoxes = 0;

	/**
	 * 감지 커널이 볼 랩 가능 convex 수. Convexes 앞쪽 [0, NumDetectConvexes)만 감지에 참여한다
	 * (박스와 동일 계약 — 정적 convex는 뒤에 append돼 자동 제외).
	 */
	int32 NumDetectConvexes = 0;

	/** 감지 스윕 샘플 간격(cm)과 샘플 수 상한 — CPU FParams::ContactSweepStep/ContactMaxSweepSamples 미러. */
	float ContactSweepStep = 2.0f;
	int32 ContactMaxSweepSamples = 16;

	/**
	 * 이 dispatch가 쓰는 콜라이더 귀속 집합의 서명(호출자가 계산). 감지 리드백에 그대로 실려 돌아와,
	 * 소비 시점에 ColliderIndex를 해석해도 되는지 판정하는 근거가 된다(FRopeResidentContacts::AttribSig).
	 */
	uint32 AttribSig = 0;

	/** 이번 프레임 substep 스케줄(호출자가 RopeSolverSubsteps로 계산해 전달). NumSub<=0이면 적분 없이 유지. */
	int32 NumSub = 0;
	float FixedDt = 0.0f;

	/**
	 * --- 접촉 감지(G3): Flight에서 솔브 후 PosBuf/PrevBuf를 스윕해 노드당 최심 접촉을 감지한다.
	 * bDetectContacts면 솔브 dispatch 뒤에 감지 커널을 돌리고 결과를 리드백한다(GetLatestContacts).
	 * ContactRadius는 감지 질의 반경(= FRopeWrapConfig::ContactQueryRadius; 솔버의 CollisionRadius와 별개).
	 */
	bool  bDetectContacts = false;
	float ContactRadius = 0.0f;

	/**
	 * --- 예측 접촉(G3b): 노드의 다음 위치를 외삽한 경로도 스윕해 곧 닿을 접촉을 감지한다. 노드당 2슬롯
	 * (actual + predictive) 출력. PredictionFrames<=0이면 예측 없음. whip 활성 프레임엔 가이드 노드의
	 * 현재/직전/다음 타깃으로 외삽하고(PredictiveGuided), 그 외엔 프레임 변위로 외삽한다(PredictiveFree).
	 * WhipGuided*는 whip 활성 시에만 NumNodes 길이로 채운다(아니면 비움 → free 예측만).
	 */
	float           PredictionFrames = 0.0f;
	/**
	 * substep→프레임 변위 환산 계수(= DeltaTime / FixedDt). free 노드 예측이 로프 Verlet 변위(마지막
	 * substep 델타)를 프레임 변위로 올리는 데 쓴다 — 1이면 환산 없음(substep 단위 축소 버그). 가이드 노드
	 * 예측은 프레임 단위 타깃 차분이라 이 계수를 안 쓴다. CPU FParams::FrameDeltaTime 경로와 동일 의미/값.
	 */
	float           ContactFrameToSubstepRatio = 1.0f;
	/** 노드별 가이드 여부(1=guided). */
	TArray<uint8>   WhipGuidedMask;
	TArray<FVector> WhipCurrentTargets;
	TArray<FVector> WhipPrevTargets;
	TArray<FVector> WhipNextTargets;

	/**
	 * --- Override(G0): 로직 페이즈(GT)가 계산한 노드별 타깃을 상주 버퍼에 직접 기록(재시드 대체).
	 * 비어 있으면 오버라이드 없음. 채울 때 OverrideFlags는 정확히 NumNodes 길이(불일치 시 전체 무시+경고),
	 * 값 배열은 해당 비트를 쓰는 노드가 있을 때만 NumNodes 길이로 제공하면 된다.
	 * NumSub=0이어도 오버라이드가 있으면 dispatch되어 적분 없이 기록만 한다(예: Wrapping/Releasing 프레임).
	 * 노드별 ERopeGPUOverride 비트 OR
	 */
	TArray<uint8>   OverrideFlags;
	/** Position 비트 노드만 유효. */
	TArray<FVector> OverridePositions;
	/** Prev 비트 노드만 유효. */
	TArray<FVector> OverridePrevPositions;
	/** InvMass 비트 노드만 유효. */
	TArray<float>   OverrideInvMass;

	bool HasOverrides() const { return OverrideFlags.Num() > 0; }
};

/** GT가 회수하는 상주 로프의 최신(약간 지연) 위치. RT 리드백이 채우고 GT가 락 하에 복사한다. */
struct FRopeResidentLatest
{
	TArray<FVector> Positions;
	TArray<FVector> PrevPositions;
	/**
	 * 세그먼트별 장력(NumNodes-1개, F = max(0,-λ)/h² — FRopeSimState::SegmentTension과 동일 단위/의미).
	 * 솔브(NumSub>0) 프레임에만 무장·회수되므로 위치보다 드물게 갱신될 수 있다(비어 있으면 미회수).
	 */
	TArray<float>   SegmentTension;
	/** 이 위치가 대응하는 시드 generation(재시드 경계의 stale 적용 방지). */
	uint32 Generation = 0;
	int32  NumNodes = 0;
};

/**
 * GPU 접촉 감지(G3) 결과 1건. 실제 접촉과 예측 접촉이 각각 노드당 최대 1개씩 나올 수 있다.
 * GPU는 bone/mesh(FName/포인터, GT 개념)를 만들 수 없으므로 콜라이더 인덱스만 emit하고,
 * 호출자(런타임)가 인덱스 → (bone, mesh)
 * 귀속 테이블로 복원한다. HLSL FRopeGPUContact와 1:1 미러(레이아웃/의미 동일).
 */
struct FRopeGPUContactResult
{
	int32   NodeIndex = INDEX_NONE;
	/** 0=capsule, 1=SDF, 2=box (step의 Capsules/SDFColliders/Boxes 배열 구분). */
	int32   ColliderType = 0;
	/** 해당 배열 내 인덱스(귀속 복원 키). */
	int32   ColliderIndex = 0;
	/** ERopeContactCandidateSource: 1=Actual, 2=PredictiveFree, 4=PredictiveGuided. */
	uint8   Source = 1;
	float   Penetration = 0.0f;
	/** 표면 접촉점(FRopeContact.SurfacePoint 대응). */
	FVector WorldPoint = FVector::ZeroVector;
	/** 바깥(collider→node) 단위 법선. */
	FVector Normal = FVector::UpVector;
	/** 접촉점 표면 속도(cm/s; 정적이면 0). */
	FVector SurfaceVelocity = FVector::ZeroVector;
};

/** GT가 회수하는 상주 로프의 최신(약간 지연) 접촉 감지 결과. GetLatestContacts로 복사. */
struct FRopeResidentContacts
{
	/** bHit 슬롯만(GPU가 채운 유효 접촉). */
	TArray<FRopeGPUContactResult> Contacts;
	/** 대응 시드 generation(stale 적용 방지). */
	uint32 Generation = 0;
	/**
	 * 이 결과를 만든 **dispatch 시점**의 콜라이더 귀속 서명(FRopeGPUResidentStep::AttribSig 그대로).
	 * ColliderIndex는 그때의 집합 순서를 가리키므로, 소비자는 자기 현재 서명과 이 값을 직접 비교해
	 * 인덱스가 아직 같은 뜻인지 판정한다 — "최근 N프레임이 안 변했다"는 근사가 아니라 정확한 대응이다.
	 */
	uint32 AttribSig = 0;
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
class FRHIShaderResourceView;
class FRDGBuilder;
class FGlobalDistanceFieldParameterData;
class FSceneView;

namespace RopeGPU
{
	/**
	 * 이 런타임에서 GPU 경로(솔버·감지·튜브)를 쓸 수 있는가. 렌더 가능한 RHI가 있는지에 더해
	 * **feature level이 SM5 이상인지**까지 본다 — 커널은 전부 SM5 가드로 컴파일되므로(각 CS의
	 * ShouldCompilePermutation) ES3.1/모바일에서는 퍼뮤테이션이 존재하지 않는다. RHI 유무만 보면
	 * 렌더는 되는 모바일에서 없는 셰이더를 요청해 assert/크래시/미출력으로 간다.
	 *
	 * 판정 기준은 전역 GMaxRHIFeatureLevel(= 이 기기가 실제로 낼 수 있는 최대치)이다. 에디터의
	 * 모바일 프리뷰는 씬 feature level만 낮추고 실 RHI는 SM6 그대로라 GPU 경로가 유지되는데,
	 * 이는 의도한 동작이다(프리뷰에서 시뮬 경로까지 바뀌면 재현이 어긋난다).
	 *
	 * 호출자는 솔버(URopeSimSubsystem)와 튜브(FRopeSceneProxy) 둘 다 — 두 게이트가 어긋나면
	 * 솔버는 GPU인데 튜브는 CPU 같은 반쪽 상태가 되므로 판정을 단일 소스로 둔다.
	 */
	DYNAMICROPESHADERS_API bool IsRuntimeSupported();
}

class DYNAMICROPESHADERS_API FRopeGPUSolver
{
public:
	/** 로프당 최대 노드 수(= 최상단 compute 스레드그룹 버킷). 호출자는 이 한도 내 step만 넘겨야 한다.
	 *  RopeGPUSolver.cpp의 노드 버킷 배열 최상단과 일치해야 한다(그쪽 static_assert가 강제). */
	static constexpr int32 MaxNodes = 512;

	FRopeGPUSolver();
	~FRopeGPUSolver();

	/**
	 * 렌더 스레드. 로프의 resident PosBuf(StructuredBuffer<float4>, 월드 위치) SRV를 반환(없으면 null).
	 * M5b B2-lite: scene proxy가 이 SRV를 직접 읽어 튜브를 GPU 생성 → 위치 무지연(렌더 리드백 없음).
	 * 솔버가 이 로프를 step한 적이 없으면(= GPU 솔버 off) null → 호출자는 CPU 경로로 폴백한다.
	 *
	 * OutGeneration은 이 버퍼가 담고 있는 시드 generation이다. 호출자는 자기 generation과 대조해야 한다 —
	 * 노드 수만 보면 **같은 노드 수로 재시드**(재던지기 등)한 프레임에 직전 로프의 포즈를 그대로 읽어
	 * 한 프레임 유령이 뜬다(버퍼는 그 프레임 dispatch 전까지 옛 세대를 들고 있다).
	 */
	FRHIShaderResourceView* GetResidentPositionSRV_RenderThread(uint32 RopeId, int32& OutNumNodes,
		uint32& OutGeneration);

	/** 이번 프레임 상주 step들을 렌더 스레드로 넘겨 GPU에서 in-place 전진(블록 없음). step은 소비된다(MoveTemp).
	    전용(자체) RDG 그래프에서 즉시 실행 — 씬 렌더러 없이 도는 유닛 테스트 하네스 경로(런타임은 EnqueueSteps). */
	void Step(TArray<FRopeGPUResidentStep>&& Steps);

	/**
	 * 런타임 dispatch 경로: step을 렌더 스레드 pending 큐에 쌓아만 둔다(dispatch 안 함). 뷰 확장이 이번 프레임
	 * PreRenderBasePass에서 씬 렌더러 그래프에 DispatchPending_RenderThread로 flush한다(GDF 파라미터 유효 타이밍).
	 */
	void EnqueueSteps(TArray<FRopeGPUResidentStep>&& Steps);

	/** 렌더 스레드. 쌓인 pending step들을 전달받은 (씬 렌더러) GraphBuilder에 얹는다(자체 Execute 안 함).
	    GDF는 이 뷰의 Global Distance Field 파라미터(null 가능), PreViewTranslation은 월드→TranslatedWorld 오프셋. */
	void DispatchPending_RenderThread(FRDGBuilder& GraphBuilder, const FSceneView* View,
		const FGlobalDistanceFieldParameterData* GDF, const FVector3f& PreViewTranslation);

	/**
	 * 소비되지 못하고 교체된 pending step의 시뮬 시간을 RopeId별 초 단위로 회수한다(호출 즉시 비운다, 락).
	 *
	 * EnqueueSteps는 교체 시맨틱이라, 뷰 확장이 도는 base pass가 없던 프레임의 step은 다음 프레임 step에
	 * 덮여 그대로 사라진다. 그런데 그 substep 시간은 GT에서 이미 accumulator를 깎고 만든 것이라, 놔두면
	 * **시뮬 시간이 영구히 없어진다**(프레임을 건너뛴 게 아니라 시간을 잃은 것이라 이후에도 안 메워진다).
	 * 호출자는 이 값을 다음 스케줄 계산 전에 accumulator로 되돌린다 — 그러면 accumulator가 다시
	 * "시뮬된 시간"의 단일 진실이 되고, 몰아치기는 RopeSolverSubsteps의 기존 상한이 알아서 막는다.
	 */
	void DrainDroppedSimTime(TMap<uint32, float>& Out);

	/** RT 리드백이 채운 최신 위치를 RopeId별로 복사(락). 새로 도착한 게 없으면 직전 값을 유지한 채 반환할 수 있다. */
	void GetLatest(TMap<uint32, FRopeResidentLatest>& Out);

	/** RT 리드백이 채운 최신 접촉 감지 결과를 RopeId별로 복사(락). GetLatest와 같은 지연 특성(약 1~2프레임). */
	void GetLatestContacts(TMap<uint32, FRopeResidentContacts>& Out);

	/**
	 * GT 블로킹 동기 리드백(M5c): 이 로프의 RT pending non-GDF step을 먼저 실행한 뒤 상주 Pos/Prev를
	 * *지금* 값으로 가져온다(GPU idle 대기 포함). Scene GDF가 필요한 pending step은 유효한 View 없이
	 * 실행하지 않고 false를 반환하며, 다음 scene dispatch까지 보존한다.
	 * wrap 핸드오프처럼 "이벤트당 1회, 최신 위치가 꼭 필요한" 곳 전용 — 매 프레임 호출 금지.
	 * OutGeneration은 버퍼가 대응하는 시드 generation(호출자가 자기 generation과 대조해 stale 거부).
	 * @return 상주 버퍼가 있고 회수에 성공하면 true.
	 */
	bool ReadbackNow(uint32 RopeId, TArray<FVector>& OutPositions, TArray<FVector>& OutPrevPositions, uint32& OutGeneration);

	/** 로프의 영속 버퍼/리드백을 해제(렌더 스레드에서). 컴포넌트 EndPlay/Unregister에서 호출. */
	void ReleaseRope(uint32 RopeId);

private:
	/**
	 * 상주 상태(렌더 스레드 전용 영속 버퍼 맵 + GT<->RT 공유 결과)를 pimpl로 숨긴다 — 헤더에 RDG/RHI 타입을
	 * 노출하지 않고, 불완전 타입을 멤버로 by-value 보관할 때의 sizeof 요구도 피한다(포인터 멤버).
	 */
	struct FImpl;
	TUniquePtr<FImpl> Impl;

	void ReleaseAll_RenderThread();

	/** Step()/DispatchPending_RenderThread 공용 실행부: 상주 seed/register/dispatch/리드백을 전달받은
	    GraphBuilder에 얹는다(Execute는 호출자 책임). Steps는 소비 후 호출자가 비운다. */
	void RunSteps_RenderThread(FRDGBuilder& GraphBuilder, TArray<FRopeGPUResidentStep>& Steps,
		const FSceneView* View, const FGlobalDistanceFieldParameterData* GDF, const FVector3f& PreViewTranslation);
};
