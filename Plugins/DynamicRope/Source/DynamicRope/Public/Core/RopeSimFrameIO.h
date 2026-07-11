// Copyright Epic Games, Inc. All Rights Reserved.
//
// URopeComponent ↔ URopeSimSubsystem의 "프레임 계약" 묶음. 서브시스템이 friend 접근으로 쓰거나
// 읽는 프레임 단위 입출력을 한 타입으로 모아 경계를 명시한다 — 컴포넌트의 나머지 private 상태와
// 달리, 이 안의 값들은 프레임마다 서브시스템 주도로 채워지거나 소비된다.
// 필드 이름은 컴포넌트 낱개 멤버 시절 그대로다(접근 경로만 SimFrame.X로 변경 — CL 303).

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtrTemplates.h"
#include "Core/RopeTypes.h"

class IRopeCollider;
class USceneComponent;

/**
 * 로프 하나의 한 프레임 시뮬 입출력(서브시스템 프레임 계약).
 * 수명 규약 요약 — 자세한 흐름은 URopeComponent::PrepareSimFrame 3단계 계약 주석 참조:
 *  - 프레임 스코프(매 프레임 리셋/재작성): FrameColliders, OverrideFrame, bSolveThisFrame,
 *    bSolveCollisionsThisFrame, bGpuSteppedThisFrame, Gpu*Attribution, GpuFlightCandidates,
 *    bGpuContactsThisFrame.
 *  - 프레임을 넘어 유지: SimGeneration(진짜 시드에만 증가), AimRayColliderQueryBounds(에임 모드 동안 유지).
 */
struct FRopeSimFrameIO
{
	/**
	 * 한 프레임 collider 스냅샷. RopeSimSubsystem이 Tick에서 중앙 수집해 채운다(provider 레지스트리 → 로프 필터).
	 * Prepare/Solve/Finalize에서 read. provider 소유라 raw 포인터(해당 프레임 동안 유효).
	 */
	TArray<IRopeCollider*> FrameColliders;

	/** 현재 로프 위치와 떨어진 조준 대상 SDF도 수집하도록 로프 AABB에 합칠 추가 영역. */
	FBox AimRayColliderQueryBounds = FBox(ForceInit);

	/**
	 * 이번 프레임에 Solver.Step을 돌릴지. Free/Flight/Wrapped true(Wrapped는 latch 노드 InvMass=0),
	 * Contacting/Wrapping/Releasing은 로직 구동이라 false.
	 */
	bool bSolveThisFrame = false;

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
	 * 동안 파괴될 수 있어 weak. (콜라이더 집합이 프레임 간 바뀌면 인덱스가 어긋날 수 있으나 순서가
	 * 안정적이고 범위 밖은 무시 → stale 미러 감지와 동일한 관용도. 최악의 경우 한 프레임 오귀속, 자기수정.)
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
	 * GPU 감지(G3) 프레임 산출: 서브시스템이 GetLatestContacts를 귀속해 Finalize 전에 채운다.
	 * bValid면 FinalizeSimFrame의 Flight 접촉 소스가 CPU 스윕 대신 이 후보들을 쓴다(GPU 경로).
	 */
	TArray<FRopeContactCandidate> GpuFlightCandidates;
	bool bGpuContactsThisFrame = false;
};
