// Copyright Epic Games, Inc. All Rights Reserved.
//
// URopeComponent ↔ URopeSimSubsystem의 "프레임 계약" 묶음. 서브시스템이 friend 접근으로 쓰거나
// 읽는 프레임 단위 입출력을 한 타입으로 모아 경계를 명시한다 — 컴포넌트의 나머지 private 상태와
// 달리, 이 안의 값들은 프레임마다 서브시스템 주도로 채워지거나 소비된다.
// 필드 이름은 컴포넌트 낱개 멤버 시절 그대로다(접근 경로만 SimFrame.X로 변경 — CL 303).

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtrTemplates.h"
#include "Core/RopeContactTrackingTypes.h"
#include "Core/RopeSimTypes.h"

class IRopeCollider;
class USceneComponent;

/**
 * 로프 하나의 한 프레임 시뮬 입출력(서브시스템 프레임 계약).
 * 수명 규약 요약 — 자세한 흐름은 URopeComponent::PrepareSimFrame 3단계 계약 주석 참조:
 *  - 프레임 스코프(매 프레임 리셋/재작성): FrameColliders, AimFrameColliders, OverrideFrame, bSolveThisFrame,
 *    bSolveCollisionsThisFrame, bForceNonStretchThisFrame, bGpuSteppedThisFrame, Gpu*Attribution, GpuFlightCandidates,
 *    bGpuContactsThisFrame.
 *  - 프레임을 넘어 유지: SimGeneration(진짜 시드에만 증가), AimRayColliderQueryBounds(에임 모드 동안 유지),
 *    LockedTargetColliderQueryBounds(aim lock Flight/Contacting/Wrapping 동안 유지).
 */
struct FRopeSimFrameIO
{
	/**
	 * 한 프레임 collider 스냅샷. RopeSimSubsystem이 Tick에서 중앙 수집해 채운다(provider 레지스트리 → 로프 필터).
	 * Prepare/Solve/Finalize에서 read. provider 소유라 raw 포인터(해당 프레임 동안 유효).
	 */
	TArray<IRopeCollider*> FrameColliders;

	/**
	 * 조준 전용 collider 스냅샷(로프 AABB ∪ aim ray 영역에서 수집). 물리/접촉/디버그가 쓰는 위
	 * FrameColliders와 **의도적으로 분리**돼 있다 — 멀리 있는 대상을 조준했다는 이유만으로 그 대상의
	 * 본 콜라이더 전부가 솔버 패킹·접촉 감지·노드 근접 디버그 질의에 실리면 안 되기 때문이다.
	 * 소비처: aim ray hit 판정, GuaranteedWrap preview 빌드. 수집은 FrameColliders와 같은 규칙
	 * (owner 제외 / 정적 예산 / cross-actor)을 따르고, 같은 collider가 양쪽 목록에 들어올 수 있다.
	 * AimRayColliderQueryBounds가 무효여도 active lock의 cached target bounds로 다시 채워질 수 있다.
	 * 포인터 수명은 FrameColliders와 동일.
	 */
	TArray<IRopeCollider*> AimFrameColliders;

	/** 조준 ray가 검사할 영역 AABB. 위 AimFrameColliders의 수집 영역이며, 물리 수집 영역과는 무관하다. */
	FBox AimRayColliderQueryBounds = FBox(ForceInit);

	/**
	 * aim throw를 확정한 첫 프레임의 target collider 유니언 bounds. Wielder가 다음 프레임에 ray bounds를
	 * 지운 뒤에도 GPU 지연 CPU mirror보다 앞서 움직이는 target을 AimFrameColliders에 계속 재수집하기 위한
	 * throw 수명 캐시다. FilterFrameCollidersForAimWrapTarget이 허용 target만으로 갱신/해제하며, 이 bounds로
	 * 모은 collider도 target 필터를 거친 뒤에만 FrameColliders로 승격된다.
	 */
	FBox LockedTargetColliderQueryBounds = FBox(ForceInit);

	/**
	 * 이번 프레임에 Solver.Step을 돌릴지. Free/Flight/Wrapping/Wrapped true(Wrapping/Wrapped는
	 * position override/anchor 노드 InvMass=0), Contacting/Releasing은 로직 구동이라 false.
	 */
	bool bSolveThisFrame = false;

	/** Wrapping→Wrapped 커밋처럼 동적 노드 소유권이 바뀌는 이번 프레임만 최대 신장을 1.0으로 제한한다. */
	bool bForceNonStretchThisFrame = false;

	/** Aim-hit Flight는 거리/굽힘/감쇠만 풀고 SDF/collider push-out은 끌 수 있다. 접촉 감지 목록과는 독립. */
	bool bSolveCollisionsThisFrame = true;

	/**
	 * 로직 페이즈의 한 프레임 산출물(G2). Prepare 동안 로직(Wrapping/Wrapped/Releasing 등)이 위치·질량을
	 * 여기에 scatter하면 Prepare 끝에서 CPU Sim에 1회 적용되고, GPU 상주 로프에는 서브시스템이 같은
	 * 데이터를 override 패스로 실어 재시드 없이 커널에서 적용한다. 매 Prepare 시작에 리셋(프레임 스코프).
	 */
	FRopeNodeOverrideFrame OverrideFrame;

	/**
	 * GPU 상주 솔버(M5)용 시드 generation. 진짜 시드(init/throw/노드 수 변경)에만 증가한다 →
	 * 서브시스템이 변화를 감지해 GPU 영속 버퍼를 재시드한다. 로직 페이즈/whip의 위치·질량 쓰기는
	 * override 패스로 주입되므로(G1/G2) 재시드하지 않는다 — 상주가 페이즈 전체에 걸쳐 유지된다.
	 */
	uint32 SimGeneration = 0;

	/**
	 * 이번 프레임에 이 로프가 실제로 GPU에서 step됐는가(서브시스템이 매 프레임 설정). M5b: GPU 튜브 렌더가
	 * resident PosBuf를 직접 읽을지(true) CPU Sim 미러로 그릴지(false, CPU-폴백/솔버 off) 가른다.
	 * whip 프레임도 G1부터 GPU(override 주입)라 true — PosBuf가 가이드 타깃을 같은 프레임에 반영한다.
	 */
	bool bGpuSteppedThisFrame = false;

	/**
	 * GPU 접촉 감지(G3) 귀속 테이블: GPU가 emit한 콜라이더 인덱스 → (bone, mesh) 복원용.
	 * 서브시스템이 GPU step 프레임마다 Step.Capsules/SDFColliders와 같은 순서로 채운다. mesh는 지연
	 * 동안 파괴될 수 있어 weak. 지연된 GPU 접촉(1~2프레임)의 ColliderIndex는 *디스패치 시점* 집합
	 * 기준인데 이 테이블은 *이번 프레임* 것으로 재빌드되므로, 지연 창 동안 집합이 바뀌면 인덱스가
	 * 다른 본으로 어긋난다. 아래 GpuAttribSig를 dispatch에 실어 보내고 결과와 함께 돌려받아, 인덱스가
	 * 아직 같은 뜻인지 정확히 비교해 드롭한다(오귀속 방지).
	 */
	struct FGpuColliderAttribution
	{
		FName Bone = NAME_None;
		TWeakObjectPtr<const USceneComponent> Mesh;
	};

	/** GPU Capsules와 평행. */
	TArray<FGpuColliderAttribution> GpuCapsuleAttribution;

	/** GPU SDFColliders와 평행. */
	TArray<FGpuColliderAttribution> GpuSdfAttribution;

	/** GPU Boxes와 평행(랩 가능 박스 감지 귀속). */
	TArray<FGpuColliderAttribution> GpuBoxAttribution;

	/**
	 * 위 귀속 집합의 순서 있는 (bone, mesh) 서명. 서브시스템이 GPU step(detect) 프레임마다 재계산해
	 * dispatch(FRopeGPUResidentStep::AttribSig)에 싣고, 감지 결과가 그 값을 그대로 되싣고 돌아온다
	 * (FRopeResidentContacts::AttribSig). BuildGpuFlightCandidates는 **결과의 서명과 현재 서명이 같을 때만**
	 * 지연 접촉을 소비한다 — 다르면 generation 미스와 동일하게 드롭(이번 프레임 GPU 후보 없음).
	 *
	 * 종전에는 직전 2프레임분을 함께 들고 "최근 3프레임이 모두 같으면 안정"으로 근사했는데, 지연이
	 * 3프레임을 넘으면(GPU stall) 그 사이 콜라이더가 재정렬됐다 안정된 상태도 통과해 팔 접촉이 다리로
	 * 귀속될 수 있었다. dispatch 시점 값을 직접 비교하면 지연 길이와 무관하게 정확하고, 집합이
	 * 바뀌었다가 같은 배치로 돌아온 경우는 오히려 정상 소비된다.
	 * 0 = 미설정(Flight 진입 직후 워밍업 — 안전하게 드롭).
	 */
	uint32 GpuAttribSig = 0;

	/**
	 * GPU 감지(G3) 프레임 산출: 서브시스템이 GetLatestContacts를 귀속해 Finalize 전에 채운다.
	 * bValid면 FinalizeSimFrame의 Flight 접촉 소스가 CPU 스윕 대신 이 후보들을 쓴다(GPU 경로).
	 */
	TArray<FRopeContactCandidate> GpuFlightCandidates;
	bool bGpuContactsThisFrame = false;
};
