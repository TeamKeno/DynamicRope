// Copyright Epic Games, Inc. All Rights Reserved.

#include "Subsystem/RopeSimSubsystem.h"
#include "RopeComponent.h"
#include "DynamicRopeLog.h"
#include "Solver/RopeXPBDSolver.h" // RopeSolverSubsteps
#include "Collision/RopeCollider.h" // IRopeCollider::GetGPUCapsule
#include "Collision/RopeColliderProvider.h" // IRopeColliderProvider (중앙 collider gather)
#include "RopeGPUSolver.h"          // FRopeGPUSolver / FRopeGPUResidentStep / FRopeGPUCapsule (DynamicRopeShaders 모듈)
#include "Engine/World.h"
#include "GameFramework/Actor.h"    // AActor::GetOwner (provider 소스 필터링)
#include "Components/ActorComponent.h"
#include "Async/ParallelFor.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"
#include "RHI.h"        // GDynamicRHI
#include "Misc/App.h"   // FApp::CanEverRender

namespace
{
	// G4: GPU가 런타임 유일 경로. 렌더 가능한 RHI가 있으면 GPU 상주 솔브+감지, 없으면(쿡/-nullrhi/
	// 서버 빌드) 자동으로 CPU 솔브+감지로 폴백한다. 클라이언트 토글(CVar) 없음 — GPU가 THE 경로.
	// FRopeXPBDSolver는 이 폴백과 패리티 테스트를 위해 유지된다(런타임 클라이언트에선 사실상 미사용).
	bool RopeGpuRuntimeAvailable()
	{
		return GDynamicRHI != nullptr && FApp::CanEverRender();
	}
}

// FRopeNodeOverrideFrame(Core 모듈) 비트는 ERopeGPUOverride(Shaders 모듈)와 수치 1:1이어야 한다 —
// Core가 Shaders에 의존하지 않으려고 상수를 미러로 두었고, 여기(둘 다 보이는 곳)서 검증한다.
static_assert(RopeNodeOverride::Position == static_cast<uint8>(ERopeGPUOverride::Position)
	&& RopeNodeOverride::Prev == static_cast<uint8>(ERopeGPUOverride::Prev)
	&& RopeNodeOverride::PrevFromPosition == static_cast<uint8>(ERopeGPUOverride::PrevFromPosition)
	&& RopeNodeOverride::InvMass == static_cast<uint8>(ERopeGPUOverride::InvMass),
	"RopeNodeOverride bits must mirror ERopeGPUOverride");

void URopeSimSubsystem::RegisterRope(URopeComponent* Rope)
{
	if (Rope)
	{
		Ropes.AddUnique(Rope);
		UE_LOG(LogDynamicRope, Verbose, TEXT("RegisterRope: %s (%d total)"), *Rope->GetName(), Ropes.Num());
	}
}

void URopeSimSubsystem::UnregisterRope(URopeComponent* Rope)
{
	Ropes.RemoveSingleSwap(Rope);
	if (Rope)
	{
		// GPU 상주 버퍼/리드백 해제(렌더 스레드에서). 캐시에서도 제거.
		const uint32 RopeId = Rope->GetUniqueID();
		GpuSolver.ReleaseRope(RopeId);
		GpuLatest.Remove(RopeId);
	}
	UE_LOG(LogDynamicRope, Verbose, TEXT("UnregisterRope: %s (%d remaining)"),
		Rope ? *Rope->GetName() : TEXT("null"), Ropes.Num());
}

URopeSimSubsystem* URopeSimSubsystem::Get(const UWorld* World)
{
	return World ? World->GetSubsystem<URopeSimSubsystem>() : nullptr;
}

void URopeSimSubsystem::RegisterColliderProvider(UActorComponent* Provider)
{
	if (Provider)
	{
		ColliderProviders.AddUnique(Provider);
		UE_LOG(LogRopeCollision, Verbose, TEXT("RegisterColliderProvider: %s (%d total)"),
			*Provider->GetName(), ColliderProviders.Num());
	}
}

void URopeSimSubsystem::UnregisterColliderProvider(UActorComponent* Provider)
{
	ColliderProviders.RemoveSingleSwap(Provider);
}

void URopeSimSubsystem::BuildFrameColliders()
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_BuildColliders);
	FrameProviders.Reset();

	// 전 로프 bounds 합집합(provider broad-phase용). 현 provider들은 무시하지만 인터페이스 계약 유지 — 향후
	// bounds-aware provider는 활성 영역으로 컬할 수 있다. 솔버가 다시 per-rope로 좁힌다.
	FBox AllBounds(ForceInit);
	for (URopeComponent* Rope : Ropes)
	{
		if (!IsValid(Rope))
		{
			continue;
		}
		for (const FVector& P : Rope->Sim.Positions)
		{
			AllBounds += P;
		}
	}
	if (AllBounds.IsValid)
	{
		AllBounds = AllBounds.ExpandBy(50.0f); // contact reach 여유.
	}

	// 등록된 provider마다 1회 gather(프레임당 1회 — 로프 수와 무관). 죽은 provider는 정리.
	for (int32 i = ColliderProviders.Num() - 1; i >= 0; --i)
	{
		UActorComponent* Comp = ColliderProviders[i];
		if (!IsValid(Comp))
		{
			ColliderProviders.RemoveAtSwap(i);
			continue;
		}
		IRopeColliderProvider* Provider = Cast<IRopeColliderProvider>(Comp);
		if (!Provider)
		{
			continue;
		}
		FFrameProviderColliders FP;
		FP.Owner = Comp->GetOwner();
		Provider->GatherColliders(AllBounds, FP.Colliders);
		if (FP.Colliders.Num() > 0)
		{
			FrameProviders.Add(MoveTemp(FP));
		}
	}
}

void URopeSimSubsystem::GatherCollidersForRope(const URopeComponent& Rope, TArray<IRopeCollider*>& OutColliders) const
{
	OutColliders.Reset();

	// 기본: 월드의 모든 provider와 충돌하되 자기 owner(던진 본인) provider는 제외(throw 시 self-tangle 방지).
	// 다른 액터 body 잡기(cross-actor)는 그 액터가 "전체"에 포함되므로 자동. owner 충돌이 필요하면 옵트인.
	const AActor* OwnerToExclude = Rope.bIncludeOwnerColliders ? nullptr : Rope.GetOwner();

	for (const FFrameProviderColliders& FP : FrameProviders)
	{
		if (FP.Owner == OwnerToExclude && OwnerToExclude != nullptr)
		{
			continue; // 자기 owner provider 제외.
		}
		OutColliders.Append(FP.Colliders);
	}
}

void URopeSimSubsystem::BuildGpuFlightCandidates(URopeComponent& Rope)
{
	Rope.GpuFlightCandidates.Reset();

	const FRopeResidentContacts* Contacts = GpuLatestContacts.Find(Rope.GetUniqueID());
	if (!Contacts || Contacts->Generation != Rope.SimGeneration)
	{
		// 아직 회수분이 없거나 재시드 catch-up 중 — 이번 프레임은 GPU 후보 없음(캡처는 다음 프레임).
		Rope.bGpuContactsThisFrame = true; // 소스는 GPU(빈 후보) — CPU 스윕으로 되돌아가지 않는다.
		return;
	}

	// Contacts는 슬롯 순서(actual 먼저, predictive 뒤)라 actual이 우선 처리된다. CPU AddUniqueCandidate와
	// 동일하게 (node, bone, mesh) 중복은 병합한다(SourceMask OR + Source 우선순위 Guided>Actual>Free) —
	// 트래커의 노드 중복 카운트를 막고 판정을 CPU와 일치시킨다.
	for (const FRopeGPUContactResult& C : Contacts->Contacts)
	{
		// 콜라이더 인덱스 → (bone, mesh) 귀속. 범위 밖(콜라이더 집합 변화)은 건너뛴다(자기수정).
		const TArray<URopeComponent::FGpuColliderAttribution>& Attr =
			(C.ColliderType == 0) ? Rope.GpuCapsuleAttribution : Rope.GpuSdfAttribution;
		if (!Attr.IsValidIndex(C.ColliderIndex))
		{
			continue;
		}
		const URopeComponent::FGpuColliderAttribution& A = Attr[C.ColliderIndex];
		if (A.Bone.IsNone())
		{
			continue; // 귀속 불가(비-스켈레탈 collider) — 캡처 대상 아님.
		}
		const USkeletalMeshComponent* Mesh = A.Mesh.Get(); // weak — 지연 중 파괴됐으면 null(판정은 bone으로 진행).

		// 병합: 같은 (node, bone, mesh) 후보가 있으면 SourceMask OR + Source 우선순위 갱신, 새 후보는 추가 안 함.
		FRopeContactCandidate* Existing = nullptr;
		for (FRopeContactCandidate& E : Rope.GpuFlightCandidates)
		{
			if (E.NodeIndex == C.NodeIndex && E.Bone == A.Bone && E.Mesh == Mesh)
			{
				Existing = &E;
				break;
			}
		}
		if (Existing)
		{
			Existing->SourceMask |= C.Source;
			if (C.Source == static_cast<uint8>(ERopeContactCandidateSource::PredictiveGuided) ||
				(Existing->Source == ERopeContactCandidateSource::Actual &&
					C.Source == static_cast<uint8>(ERopeContactCandidateSource::PredictiveFree)))
			{
				Existing->Source = static_cast<ERopeContactCandidateSource>(C.Source);
			}
			continue;
		}

		FRopeContactCandidate Cand;
		Cand.bValid          = true;
		Cand.NodeIndex       = C.NodeIndex;
		Cand.Bone            = A.Bone;
		Cand.Mesh            = Mesh;
		Cand.Source          = static_cast<ERopeContactCandidateSource>(C.Source);
		Cand.SourceMask      = C.Source;
		Cand.WorldPoint      = C.WorldPoint;
		Cand.Normal          = C.Normal.GetSafeNormal();
		Cand.Penetration     = C.Penetration;
		Cand.SurfaceVelocity = C.SurfaceVelocity;
		Cand.WrapDirectionScore = 0.0f; // EvaluateRelativeMotion(GT)이 채운다.
		Rope.GpuFlightCandidates.Add(Cand);
	}
	Rope.bGpuContactsThisFrame = true;
}

bool URopeSimSubsystem::SyncGpuPositionsForHandoff(URopeComponent& Rope)
{
	if (!RopeGpuRuntimeAvailable())
	{
		return false; // CPU 폴백 — Sim이 이미 최신.
	}

	FRopeSimState& S = Rope.Sim;
	TArray<FVector> Pos;
	TArray<FVector> Prev;
	uint32 Generation = 0;
	if (!GpuSolver.ReadbackNow(Rope.GetUniqueID(), Pos, Prev, Generation))
	{
		return false; // 상주 버퍼 없음(GPU로 step된 적 없음) — 미러가 곧 진실.
	}
	if (Generation != Rope.SimGeneration || Pos.Num() != S.Num() || Prev.Num() != S.Num())
	{
		return false; // 재시드 catch-up 중이거나 노드 수 불일치 — stale 적용 방지.
	}

	S.Positions     = MoveTemp(Pos);
	S.PrevPositions = MoveTemp(Prev);
	// 잡은 끝(node 0)은 미러 규약과 동일하게 현재 핀으로 스냅.
	if (S.bStartPinned && S.Num() > 0)
	{
		S.Positions[0]     = S.StartPinTarget;
		S.PrevPositions[0] = S.StartPinPrev;
	}
	return true;
}

void URopeSimSubsystem::Tick(float DeltaTime)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_SubsystemTick);

	// 무효 항목 정리.
	for (int32 i = Ropes.Num() - 1; i >= 0; --i)
	{
		if (!IsValid(Ropes[i]))
		{
			Ropes.RemoveAtSwap(i);
		}
	}
	if (Ropes.Num() == 0)
	{
		return;
	}

	// G4: GPU가 유일 런타임 경로. 렌더 가능 RHI면 GPU, 아니면 CPU 폴백(자동). 감지도 GPU와 함께 켜진다.
	const bool bUseGPU = RopeGpuRuntimeAvailable();
	const bool bUseGPUContacts = bUseGPU; // GPU 솔브 시 감지도 GPU(별도 토글 없음).

	// GPU 상주(M5): RT 리드백이 채운 RopeId별 최신(약 1~2프레임 지연) 위치를 회수해 캐시. 아래 Phase 2에서
	// Free/Flight 로프의 Sim(렌더/충돌 미러)에 반영한다. 순차 의존성은 GPU 영속 버퍼 안에서 충족된다.
	if (bUseGPU)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_GPUGetLatest);
		GpuSolver.GetLatest(GpuLatest);
		if (bUseGPUContacts)
		{
			GpuSolver.GetLatestContacts(GpuLatestContacts); // G3: 접촉 감지 결과 회수(Finalize 전에 귀속).
		}
	}

	// Phase 1a (GT): collider 중앙 수집 — 등록된 provider에서 프레임당 1회 빌드 후 로프별 필터로 FrameColliders 채움.
	// (로프마다 월드를 스캔하던 것을 대체. collider 포인터는 provider 소유라 이번 프레임 solve/finalize 동안 유효.)
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_GatherColliders);
		BuildFrameColliders();
		for (URopeComponent* Rope : Ropes)
		{
			GatherCollidersForRope(*Rope, Rope->FrameColliders);
		}
	}

	// Phase 1b (GT): 준비 — init/pin + 로직 phase 처리(collider는 위에서 이미 채워짐).
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_Prepare);
		for (URopeComponent* Rope : Ropes)
		{
			Rope->PrepareSimFrame(DeltaTime);
		}
	}

	// Phase 2: Free/Flight의 solver step.
	if (bUseGPU)
	{
		// GPU 상주 경로(M5a). 로프별 영속 버퍼를 매 프레임 in-place로 전진(라운드트립 스톨/슬로모 없음).
		// whip(G1)과 로직 페이즈(G2 — Wrapping/Wrapped/Releasing)도 GPU 상주: 로직 산출물
		// (OverrideFrame)을 override 패스로 실어 재시드 없이 커널에서 적용한다. 적분이 없는
		// 로직 프레임은 NumSub=0 override-only dispatch. 접촉 감지는 Finalize가 지연 미러로 처리(G3).
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_SolveGPU);

		TArray<FRopeGPUResidentStep> Steps;
		Steps.Reserve(Ropes.Num());
		for (URopeComponent* Rope : Ropes)
		{
			FRopeSimState& S = Rope->Sim;
			// GPU 상주 대상: 솔브 프레임(Free/Flight/Wrapped) 또는 로직 산출물이 있는 프레임
			// (Wrapping/Releasing 등 — override-only). Contacting(산출물 없음)은 dispatch 자체가
			// 없어 GPU 버퍼가 동결 상태로 유지된다(CPU의 "솔브 없음"과 동일).
			const bool bSolvePhase = Rope->bSolveThisFrame &&
				(Rope->Phase == ERopePhase::Free || Rope->Phase == ERopePhase::Flight || Rope->Phase == ERopePhase::Wrapped);
			const bool bGpuRope = (bSolvePhase || Rope->OverrideFrame.HasAny())
				&& S.Num() >= 2 && S.Num() <= FRopeGPUSolver::MaxNodes;
			// M5b: 이 프레임에 GPU step되는 로프만 렌더가 resident PosBuf를 직접 읽는다(아니면 stale → CPU 미러).
			Rope->bGpuSteppedThisFrame = bGpuRope;
			if (!bGpuRope)
			{
				// 폴백(노드수 초과 등): CPU 솔브. logic phase는 bSolveThisFrame=false라 자동 스킵.
				if (Rope->bSolveThisFrame)
				{
					Rope->SolveSimFrame(DeltaTime);
				}
				continue;
			}

			const uint32 RopeId = Rope->GetUniqueID();

			// 직전 회수분을 Sim(미러)에 반영. generation이 현재와 일치할 때만(= GPU가 현재 시드를 따라잡음);
			// 재시드 직후 catch-up 중이면 CPU Sim을 그대로 둬 시드 소스를 보존한다.
			if (const FRopeResidentLatest* L = GpuLatest.Find(RopeId))
			{
				if (L->Generation == Rope->SimGeneration && L->NumNodes == S.Num()
					&& L->Positions.Num() == S.Num() && L->PrevPositions.Num() == S.Num())
				{
					S.Positions     = L->Positions;
					S.PrevPositions = L->PrevPositions;
				}
			}

			// 잡은 끝(node 0)을 현재 핀 위치로 정확히 맞춘다 — GPU 미러는 ~1~2프레임 지연이라 손과 어긋난다.
			// 렌더/접촉용 보정(GPU 솔브 자체는 PinTarget으로 매 스텝 핀을 처리하므로 시뮬레이션엔 영향 없음).
			if (S.bStartPinned && S.Num() > 0)
			{
				S.Positions[0]     = S.StartPinTarget;
				S.PrevPositions[0] = S.StartPinPrev;
			}

			// 미러가 Prepare의 로직 산출물(앵커 위치 등)을 덮었으면 재적용 — CPU Sim 미러를
			// "최신 본 기준 로직 쓰기 + 지연된 자유 구간"의 최선 조합으로 유지한다(G2).
			if (Rope->OverrideFrame.HasAny())
			{
				Rope->OverrideFrame.ApplyToSim(S);
			}

			// 고정-timestep 스케줄(CPU accumulator). 로직 프레임(bSolveThisFrame=false)은 적분 없이
			// override만 기록한다(NumSub=0) — CPU 경로의 "솔브 없음"과 동일한 시간 처리.
			FRopeSubstepSchedule Schedule;
			Schedule.NumSub = 0;
			Schedule.FixedDt = 0.0f;
			if (Rope->bSolveThisFrame)
			{
				Schedule = RopeSolverSubsteps(S, Rope->SolverConfig, DeltaTime);
			}

			// 상주 step 구성(self-contained). 시드 데이터는 매 프레임 제공(RT는 재시드 시에만 GPU 업로드).
			const FRopeSolverConfig& Cfg = Rope->SolverConfig;
			FRopeGPUResidentStep Step;
			Step.RopeId            = RopeId;
			Step.Generation        = Rope->SimGeneration;
			Step.NumNodes          = S.Num();
			Step.SeedPositions     = S.Positions;
			Step.SeedPrevPositions = S.PrevPositions;
			Step.InvMass           = S.InvMass;
			Step.SegmentLength     = S.SegmentLength;
			Step.bStartPinned      = S.bStartPinned;
			Step.StartPinPrev      = S.StartPinPrev;
			Step.StartPinTarget    = S.StartPinTarget;
			Step.StretchCompliance = Cfg.StretchCompliance;
			Step.BendCompliance    = Cfg.BendCompliance;
			Step.Damping           = Cfg.Damping;
			Step.Iterations        = Cfg.Iterations;
			Step.CollisionPasses   = Cfg.CollisionPassesPerSubstep;
			Step.Gravity           = Cfg.Gravity;
			Step.CollisionRadius   = Cfg.CollisionRadius;
			Step.Friction          = Cfg.Friction;
			Step.TipFrictionScale  = Cfg.TipFrictionScale;
			Step.SweepStep         = Cfg.SweepStep;
			Step.MaxSweepSamples   = Cfg.MaxSweepSamples;
			Step.NumSub            = Schedule.NumSub;
			Step.FixedDt           = Schedule.FixedDt;

			// G3: 접촉 감지 요청은 GPUContacts on + Flight 로프에만(캡처는 Flight에서만 일어난다).
			// 귀속 테이블(콜라이더 인덱스 → bone/mesh)을 Step.Capsules/SDFColliders와 같은 순서로 채운다.
			const bool bDetectThisRope = bUseGPUContacts && Rope->Phase == ERopePhase::Flight;
			if (bDetectThisRope)
			{
				Step.bDetectContacts = true;
				Step.ContactRadius = Rope->WrapConfig.ContactRadius;
				Step.PredictionFrames = Rope->WrapConfig.PredictiveContactFrames;
				Rope->GpuCapsuleAttribution.Reset();
				Rope->GpuSdfAttribution.Reset();

				// 예측 접촉(G3b): whip 활성 프레임엔 가이드 마스크/현재·직전·다음 타깃을 실어 GPU가
				// 가이드 노드를 외삽하게 한다(CPU AddPredictedContactCandidates와 동일 입력).
				const TArray<uint8>& WhipMask = Rope->WhipGuide.GetGuidedNodeMask();
				if (Step.PredictionFrames > KINDA_SMALL_NUMBER && WhipMask.Num() == S.Num())
				{
					TArray<FVector> NextTargets;
					Rope->WhipGuide.PreviewNextTargets(DeltaTime, S, Rope->MakeWhipGuideConfig(), NextTargets);
					Step.WhipGuidedMask     = WhipMask;
					Step.WhipCurrentTargets = Rope->WhipGuide.GetCurrentTargets();
					Step.WhipPrevTargets    = Rope->WhipGuide.GetPrevTargets();
					Step.WhipNextTargets    = MoveTemp(NextTargets);
				}
			}

			// 충돌: 이 로프의 collider를 capsule(M2)/SDF(M3)로 분류. FrameColliders는 Prepare에서 GT gather된 스냅샷.
			for (IRopeCollider* Collider : Rope->FrameColliders)
			{
				if (!Collider)
				{
					continue;
				}
				FRopeGPUCapsule Cap;
				if (Collider->GetGPUCapsule(Cap.A, Cap.B, Cap.Radius))
				{
					Step.Capsules.Add(Cap);
					if (bDetectThisRope)
					{
						URopeComponent::FGpuColliderAttribution Attr;
						const USkeletalMeshComponent* Mesh = nullptr;
						Collider->GetGPUAttribution(Attr.Bone, Mesh);
						Attr.Mesh = Mesh;
						Rope->GpuCapsuleAttribution.Add(Attr);
					}
					continue;
				}
				FRopeSDFColliderView View;
				if (Collider->GetGPUSDF(View))
				{
					FRopeGPUSDFCollider Sdf;
					Sdf.Distances   = View.Distances;   // 코드 바이트 블롭(업로드 평탄화 시 dequant)
					Sdf.BytesPerCode = View.BytesPerCode;
					Sdf.NarrowBandInner = View.NarrowBandInner;
					Sdf.NarrowBandOuter = View.NarrowBandOuter;
					Sdf.ResX        = View.ResX;
					Sdf.ResY        = View.ResY;
					Sdf.ResZ        = View.ResZ;
					Sdf.LocalMin    = View.LocalMin;
					Sdf.LocalSize   = View.LocalSize;
					Sdf.BoneToWorld = View.BoneToWorld;
					Sdf.PrevBoneToWorld = View.PrevBoneToWorld; // GPU CCD/표면속도 드래그.
					Sdf.InvDeltaTime    = View.InvDeltaTime;
					Sdf.VolumeKey   = View.VolumeKey;
					Step.SDFColliders.Add(Sdf);
					if (bDetectThisRope)
					{
						URopeComponent::FGpuColliderAttribution Attr;
						const USkeletalMeshComponent* Mesh = nullptr;
						Collider->GetGPUAttribution(Attr.Bone, Mesh);
						Attr.Mesh = Mesh;
						Rope->GpuSdfAttribution.Add(Attr);
					}
				}
			}

			// G2: 로직 페이즈 산출물(OverrideFrame)을 override로 주입 — 로직 페이즈 재시드 대체.
			// CPU Sim에 적용된 것과 완전히 같은 데이터(비트 미러는 위 static_assert로 보증).
			if (Rope->OverrideFrame.HasAny() && Rope->OverrideFrame.Flags.Num() == S.Num())
			{
				Step.OverrideFlags         = Rope->OverrideFrame.Flags;
				Step.OverridePositions     = Rope->OverrideFrame.Positions;
				Step.OverridePrevPositions = Rope->OverrideFrame.PrevPositions;
				Step.OverrideInvMass       = Rope->OverrideFrame.InvMass;
			}

			// G1: whip 가이드 타깃을 override로 주입 — 재시드/CPU 폴백 없이 상주 유지(적분 전 적용).
			// Prepare의 Advance가 계산한 산출물을 그대로 싣는다(CPU 경로의 ApplyToSim과 동일 데이터).
			// Flight 게이트: 다른 페이즈에 남은 stale 마스크가 적용되는 것을 막는다.
			// (Flight는 OverrideFrame을 채우지 않으므로 위 G2 패킹과 겹치지 않는다.)
			if (Rope->Phase == ERopePhase::Flight)
			{
				const TArray<uint8>& WhipMask = Rope->WhipGuide.GetGuidedNodeMask();
				if (WhipMask.Num() > 0)
				{
					const TArray<FVector>& WhipCur  = Rope->WhipGuide.GetCurrentTargets();
					const TArray<FVector>& WhipPrev = Rope->WhipGuide.GetPrevTargets();
					Step.OverrideFlags.SetNumZeroed(S.Num());
					Step.OverridePositions.SetNumZeroed(S.Num());
					Step.OverridePrevPositions.SetNumZeroed(S.Num());
					for (int32 k = 0; k < S.Num() && k < WhipMask.Num(); ++k)
					{
						if (WhipMask[k] == 0 || !WhipCur.IsValidIndex(k))
						{
							continue;
						}
						Step.OverrideFlags[k] = static_cast<uint8>(ERopeGPUOverride::Position | ERopeGPUOverride::Prev);
						Step.OverridePositions[k]     = WhipCur[k];
						// 직전 타깃이 없으면(엣지 케이스) 속도 0 — CPU 폴백("직전 위치")과 근사.
						Step.OverridePrevPositions[k] = WhipPrev.IsValidIndex(k) ? WhipPrev[k] : WhipCur[k];
					}
				}
			}

			Steps.Add(MoveTemp(Step));
		}
		if (Steps.Num() > 0)
		{
			GpuSolver.Step(MoveTemp(Steps));
		}
	}
	else
	{
		// CPU 경로(기본): 로프는 서로 독립 + collider 스냅샷 read-only → 스레드 안전.
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_SolveParallel);
		ParallelFor(Ropes.Num(), [this, DeltaTime](int32 Index)
		{
			Ropes[Index]->bGpuSteppedThisFrame = false; // CPU 경로 → resident 렌더 안 함(M5b).
			Ropes[Index]->SolveSimFrame(DeltaTime);
		});
	}

	// Phase 3 (GT): 마무리 — Flight 접촉 감지/캡처(UObject·이벤트) + 렌더 dirty.
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(RopeSim_Finalize);
		for (URopeComponent* Rope : Ropes)
		{
			// G3: GPU 감지 결과를 귀속해 Finalize의 Flight 접촉 소스를 GPU 후보로 채운다.
			// 게이트는 전역이 아니라 로프별 bGpuSteppedThisFrame — 이 프레임 실제로 GPU step된 로프만
			// GPU 감지를 쓴다. GPU step 못 한 로프(노드>256 등)는 CPU 솔브됐으므로 여기서도 GPU 후보를
			// 강제하지 않아, FinalizeSimFrame이 CPU 스윕 감지로 폴백한다(안 그러면 감지 자체가 누락돼 캡처 불가).
			Rope->bGpuContactsThisFrame = false;
			if (Rope->bGpuSteppedThisFrame && Rope->Phase == ERopePhase::Flight)
			{
				BuildGpuFlightCandidates(*Rope);
			}
			Rope->FinalizeSimFrame(DeltaTime);
		}
	}

	// TODO: LOD/sleep(멀거나 안정된 로프 스킵), 프레임당 총 솔브 비용 상한.
	// TODO: tick 순서 — 충돌은 애니메이션(본 트랜스폼) 이후가 필요. 정밀 정렬은 TG_PostPhysics tick function.
}

TStatId URopeSimSubsystem::GetStatId() const
{
	RETURN_QUICK_DECLARE_CYCLE_STAT(URopeSimSubsystem, STATGROUP_Tickables);
}

bool URopeSimSubsystem::DoesSupportWorldType(const EWorldType::Type WorldType) const
{
	// 게임/PIE에서만 시뮬레이션(에디터 프리뷰/인스펙터 월드 제외 → 컴포넌트도 그때만 BeginPlay 등록).
	return WorldType == EWorldType::Game || WorldType == EWorldType::PIE;
}
