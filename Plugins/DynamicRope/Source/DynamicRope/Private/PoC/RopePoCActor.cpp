// Copyright Epic Games, Inc. All Rights Reserved.
//
// PoC — 실험용이며 출시 대상 아님. Docs/PoC/01_PostWrapModel.md 참고.

#include "PoC/RopePoCActor.h"
#include "PoC/RopePoCCapsuleActor.h"

#include "Components/SplineMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInterface.h"
#include "UObject/ConstructorHelpers.h"
#include "DrawDebugHelpers.h"
#include "EngineUtils.h"
#include "Engine/Engine.h"
#include "HAL/PlatformTime.h"

namespace
{
	// /Engine/BasicShapes/Cylinder의 기본 단면 radius(cm).
	constexpr float EngineCylinderBaseRadius = 50.0f;
	// 큰 프레임 hitch가 solver를 폭주시키지 못하도록 시뮬레이션 timestep을 clamp한다.
	constexpr float MaxSimDeltaSeconds = 1.0f / 30.0f;
}

ARopePoCActor::ARopePoCActor()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true;

	RopeRoot = CreateDefaultSubobject<USceneComponent>(TEXT("RopeRoot"));
	SetRootComponent(RopeRoot);
}

void ARopePoCActor::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	// 액터가 배치/이동/편집될 때마다 새로운 직선 rope를 배치한다.
	bInitialized = false;
}

void ARopePoCActor::BeginPlay()
{
	Super::BeginPlay();
	bInitialized = false;
}

#if WITH_EDITOR
void ARopePoCActor::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	// 어떤 프로퍼티를 수정하든 다음 tick에서 rope를 재생성한다.
	bInitialized = false;
}
#endif

FVector ARopePoCActor::GetStartWorld() const
{
	return GetActorLocation();
}

FVector ARopePoCActor::GetEndWorld() const
{
	if (bPinEnd && EndAnchorActor)
	{
		return EndAnchorActor->GetActorLocation();
	}
	// Free end: 액터의 forward axis를 따라 rope를 펼친다.
	return GetActorLocation() + GetActorForwardVector() * RopeLength;
}

void ARopePoCActor::InitializeRope()
{
	NumParticles = FMath::Max(2, NumParticles);
	SegmentLength = RopeLength / static_cast<float>(NumParticles - 1);

	const FVector StartW = GetStartWorld();
	const FVector EndW = GetEndWorld();

	Positions.SetNum(NumParticles);
	OldPositions.SetNum(NumParticles);
	InvMasses.SetNum(NumParticles);

	for (int32 i = 0; i < NumParticles; ++i)
	{
		const float Alpha = static_cast<float>(i) / static_cast<float>(NumParticles - 1);
		Positions[i] = FMath::Lerp(StartW, EndW, Alpha);
		OldPositions[i] = Positions[i];
		InvMasses[i] = 1.0f;
	}

	// Wrap-latch 상태는 비어서 시작한다(아직 latch된 노드 없음).
	LatchCapsule.Init(-1, NumParticles);
	LatchAlong.Init(0.0f, NumParticles);
	LatchRadialDir.Init(FVector::UpVector, NumParticles);
	LatchRadialDist.Init(0.0f, NumParticles);
	LatchAxis.Init(FVector::UpVector, NumParticles);
	ContactDwell.Init(0.0f, NumParticles);
	bHasPrevPins = false;

	RebuildSegmentMeshes();
	GatherProviders();
	ApplyPinning();

	bInitialized = true;
}

void ARopePoCActor::GatherProviders()
{
	CapsuleProviders.Reset();

	// 명시적 provider(테스트용 capsule 액터).
	for (ARopePoCCapsuleActor* C : Colliders)
	{
		if (C)
		{
			CapsuleProviders.AddUnique(C);
		}
	}

	if (!bAutoFindColliders)
	{
		return;
	}

	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	// 레벨에서 IRopeCapsuleProvider를 구현한 모든 액터 또는 컴포넌트.
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		AActor* Actor = *It;
		if (!Actor)
		{
			continue;
		}

		if (Actor->Implements<URopeCapsuleProvider>())
		{
			CapsuleProviders.AddUnique(Actor);
		}

		TArray<UActorComponent*> Components;
		Actor->GetComponents(Components);
		for (UActorComponent* Comp : Components)
		{
			if (Comp && Comp->Implements<URopeCapsuleProvider>())
			{
				CapsuleProviders.AddUnique(Comp);
			}
		}
	}
}

void ARopePoCActor::BuildFrameCapsules()
{
	FrameCapsules.Reset();
	FrameCapsuleOwner.Reset();

	for (int32 ProviderIdx = 0; ProviderIdx < CapsuleProviders.Num(); ++ProviderIdx)
	{
		UObject* Obj = CapsuleProviders[ProviderIdx].Get();
		if (!Obj)
		{
			continue;
		}
		if (const IRopeCapsuleProvider* Provider = Cast<IRopeCapsuleProvider>(Obj))
		{
			const int32 Before = FrameCapsules.Num();
			Provider->GatherRopeCapsules(FrameCapsules);
			// 이 provider가 방금 추가한 모든 capsule에 자신의 provider 인덱스를 태깅한다.
			for (int32 c = Before; c < FrameCapsules.Num(); ++c)
			{
				FrameCapsuleOwner.Add(ProviderIdx);
			}
		}
	}

	// 이번 프레임의 pull-reaction 누산기를 capsule 집합에 맞춰 reset한다.
	CapsuleReaction.Init(FVector::ZeroVector, FrameCapsules.Num());
	CapsuleReactionPoint.Init(FVector::ZeroVector, FrameCapsules.Num());
	CapsuleReactionWeight.Init(0.0f, FrameCapsules.Num());
}

void ARopePoCActor::RebuildSegmentMeshes()
{
	// 별도 설정 없이도 rope가 보이도록 엔진 에셋으로 fallback한다.
	if (!RopeMesh)
	{
		RopeMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	}
	if (!RopeMaterial)
	{
		RopeMaterial = LoadObject<UMaterialInterface>(nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	}

	const int32 DesiredSegments = NumParticles - 1;
	const float RadiusScale = RopeRadius / EngineCylinderBaseRadius;

	// segment 개수가 그대로일 때(예: 액터를 단순히 드래그하는 중)는 기존 컴포넌트를
	// 재사용하고, segment별 시각 설정만 갱신한다.
	bool bCountMatches = (SegmentMeshes.Num() == DesiredSegments);
	for (USplineMeshComponent* SM : SegmentMeshes)
	{
		bCountMatches &= (SM != nullptr);
	}
	if (bCountMatches)
	{
		for (USplineMeshComponent* SM : SegmentMeshes)
		{
			if (RopeMesh) { SM->SetStaticMesh(RopeMesh); }
			if (RopeMaterial) { SM->SetMaterial(0, RopeMaterial); }
			SM->SetStartScale(FVector2D(RadiusScale, RadiusScale), false);
			SM->SetEndScale(FVector2D(RadiusScale, RadiusScale), false);
		}
		return;
	}

	// segment 개수가 바뀜: 낡은 컴포넌트를 버리고 처음부터 재생성한다.
	for (USplineMeshComponent* SM : SegmentMeshes)
	{
		if (SM)
		{
			SM->DestroyComponent();
		}
	}
	SegmentMeshes.Reset();
	SegmentMeshes.SetNum(DesiredSegments);

	for (int32 i = 0; i < DesiredSegments; ++i)
	{
		USplineMeshComponent* SM = NewObject<USplineMeshComponent>(this, NAME_None, RF_Transient);
		SM->SetMobility(EComponentMobility::Movable);
		SM->SetupAttachment(RopeRoot);
		SM->RegisterComponent();

		SM->SetForwardAxis(ESplineMeshAxis::Z, /*bUpdateMesh=*/false);
		if (RopeMesh)
		{
			SM->SetStaticMesh(RopeMesh);
		}
		if (RopeMaterial)
		{
			SM->SetMaterial(0, RopeMaterial);
		}
		SM->SetStartScale(FVector2D(RadiusScale, RadiusScale), false);
		SM->SetEndScale(FVector2D(RadiusScale, RadiusScale), false);
		SM->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		SM->SetCastShadow(true);

		SegmentMeshes[i] = SM;
	}
}

void ARopePoCActor::ApplyPinning()
{
	SetPinnedTargets(GetStartWorld(), GetEndWorld());
}

void ARopePoCActor::SetPinnedTargets(const FVector& StartW, const FVector& EndW)
{
	if (Positions.Num() == 0)
	{
		return;
	}

	if (bPinStart)
	{
		Positions[0] = StartW;
		OldPositions[0] = StartW;
		InvMasses[0] = 0.0f;
	}
	else
	{
		InvMasses[0] = 1.0f;
	}

	const int32 Last = Positions.Num() - 1;
	if (bPinEnd && EndAnchorActor)
	{
		Positions[Last] = EndW;
		OldPositions[Last] = EndW;
		InvMasses[Last] = 0.0f;
	}
	else
	{
		InvMasses[Last] = 1.0f;
	}
}

void ARopePoCActor::SimulateStep(float DeltaSeconds)
{
	const float FrameDt = FMath::Min(DeltaSeconds, MaxSimDeltaSeconds);
	const int32 Sub = FMath::Clamp(SimSubsteps, 1, 16);
	const float SubDt = FrameDt / static_cast<float>(Sub);
	const float SubDt2 = SubDt * SubDt;
	const float DampFactor = 1.0f - FMath::Clamp(Damping, 0.0f, 1.0f);
	const int32 ItersPerSub = FMath::Max(1, SolverIterations / Sub);

	// 이번 프레임의 capsule(현재 포즈) + 프레임별 반작용 누산기 reset.
	BuildFrameCapsules();

	// Pinned-end target: 직전 프레임 포즈(Prev)에서 이번 프레임 포즈(Target)로 sweep한다.
	const FVector StartTarget = GetStartWorld();
	const FVector EndTarget = GetEndWorld();
	if (!bHasPrevPins)
	{
		PrevStartWorld = StartTarget;
		PrevEndWorld = EndTarget;
		bHasPrevPins = true;
	}

	const bool bCanInterpCapsules = (PrevFrameCapsules.Num() == FrameCapsules.Num());

	for (int32 s = 1; s <= Sub; ++s)
	{
		const float Alpha = static_cast<float>(s) / static_cast<float>(Sub);
		const float AlphaPrev = static_cast<float>(s - 1) / static_cast<float>(Sub);

		// 이번 substep의 capsule 포즈와 직전 substep의 포즈(friction 표면 속도용).
		ActiveCapsules = FrameCapsules;
		PrevActiveCapsules = FrameCapsules;
		if (bCanInterpCapsules)
		{
			for (int32 c = 0; c < FrameCapsules.Num(); ++c)
			{
				const FRopeCapsule& P = PrevFrameCapsules[c];
				const FRopeCapsule& F = FrameCapsules[c];
				ActiveCapsules[c].A = FMath::Lerp(P.A, F.A, Alpha);
				ActiveCapsules[c].B = FMath::Lerp(P.B, F.B, Alpha);
				PrevActiveCapsules[c].A = FMath::Lerp(P.A, F.A, AlphaPrev);
				PrevActiveCapsules[c].B = FMath::Lerp(P.B, F.B, AlphaPrev);
			}
		}

		// (더 작은) substep timestep으로 free 파티클을 Verlet integration한다.
		for (int32 i = 0; i < Positions.Num(); ++i)
		{
			if (InvMasses[i] <= 0.0f)
			{
				continue;
			}
			const FVector Velocity = (Positions[i] - OldPositions[i]) * DampFactor;
			const FVector NewPos = Positions[i] + Velocity + Gravity * SubDt2;
			OldPositions[i] = Positions[i];
			Positions[i] = NewPos;
		}

		// pinned 끝을 보간된 target으로 sweep한다.
		SetPinnedTargets(FMath::Lerp(PrevStartWorld, StartTarget, Alpha),
		                 FMath::Lerp(PrevEndWorld, EndTarget, Alpha));

		// Hold: 이번 substep 동안 latch된 wrap 노드를 (움직이는) capsule 표면에 붙여둔다.
		UpdateLatchedPositions();

		for (int32 Iter = 0; Iter < ItersPerSub; ++Iter)
		{
			// 반복마다 solve 방향을 번갈아, 어느 끝도 Gauss-Seidel sweep을 "이기지" 않게
			// 한다 — 한쪽으로 처지는 현상 / 비대칭 wrap 편향을 제거한다.
			const bool bReverse = (Iter & 1) != 0;
			SolveConstraints(bReverse);
			SolveBendingConstraints(bReverse);
			SolveCollisions();
		}

		// Friction은 contact가 해결된 뒤에 적용하는 substep별 속도 보정이다.
		ApplyFriction();
	}

	// 해결된 프레임 상태를 이용해, 새로 성립된 wrap을 latch / 뜯겨나간 것은 해제한다.
	ManageWrapLatch(FrameDt);
	// latch된 노드는 push-out 반작용을 받지 않으므로(pin됨) 대신 그들의 tension을 되먹인다.
	AccumulateLatchReaction();

	// S4: 반작용은 모든 substep에 걸쳐 누적되었다 — provider에게 한 번에 전달한다.
	ApplyPullReaction();

	// 다음 프레임의 sweep을 위해 이번 프레임의 pin target과 capsule 포즈를 기억해둔다.
	PrevStartWorld = StartTarget;
	PrevEndWorld = EndTarget;
	PrevFrameCapsules = FrameCapsules;
}

void ARopePoCActor::SolveConstraints(bool bReverse)
{
	const int32 Count = Positions.Num() - 1; // i와 i+1 사이의 distance constraint
	for (int32 k = 0; k < Count; ++k)
	{
		const int32 i = bReverse ? (Count - 1 - k) : k;
		FVector& A = Positions[i];
		FVector& B = Positions[i + 1];
		const float WA = InvMasses[i];
		const float WB = InvMasses[i + 1];
		const float WSum = WA + WB;
		if (WSum <= 0.0f)
		{
			continue; // 양쪽 끝점 모두 pin됨
		}

		const FVector Delta = B - A;
		const float Dist = Delta.Size();
		if (Dist <= KINDA_SMALL_NUMBER)
		{
			continue;
		}

		const float Diff = (Dist - SegmentLength) / Dist;
		const FVector Correction = Delta * Diff;
		A += Correction * (WA / WSum);
		B -= Correction * (WB / WSum);
	}
}

void ARopePoCActor::SolveBendingConstraints(bool bReverse)
{
	if (BendStiffness <= 0.0f)
	{
		return;
	}

	// Jakobsen "support stick": i와 i+2 사이의 distance constraint로, rest length는
	// 직선 거리(2 segment)다. rope가 휘면 이 두 노드는 직선 거리보다 가까워지므로,
	// constraint가 둘을 밀어낸다 → 휨에 저항 = stiffness.
	// BendStiffness [0..1]가 보정량을 스케일한다(1 = 뻣뻣, 0 = 흐물거리는 체인).
	const float BendRest = SegmentLength * 2.0f;

	const int32 Count = Positions.Num() - 2; // i와 i+2 사이의 support stick
	for (int32 k = 0; k < Count; ++k)
	{
		const int32 i = bReverse ? (Count - 1 - k) : k;
		FVector& A = Positions[i];
		FVector& B = Positions[i + 2];
		const float WA = InvMasses[i];
		const float WB = InvMasses[i + 2];
		const float WSum = WA + WB;
		if (WSum <= 0.0f)
		{
			continue;
		}

		const FVector Delta = B - A;
		const float Dist = Delta.Size();
		if (Dist <= KINDA_SMALL_NUMBER)
		{
			continue;
		}

		const float Diff = (Dist - BendRest) / Dist;
		const FVector Correction = Delta * Diff * BendStiffness;
		A += Correction * (WA / WSum);
		B -= Correction * (WB / WSum);
	}
}

void ARopePoCActor::SolveCollisions()
{
	// 접촉이 degenerate할 때(rope가 정확히 axis 위) capsule-axis 기준 normal을 고른다.
	auto FallbackNormal = [](const FRopeCapsule& Cap) -> FVector
	{
		const FVector Axis = (Cap.B - Cap.A).GetSafeNormal(1e-4f, FVector::UpVector);
		return FVector::CrossProduct(Axis, FVector::ForwardVector).GetSafeNormal(1e-4f, FVector::RightVector);
	};

	for (int32 c = 0; c < ActiveCapsules.Num(); ++c)
	{
		const FRopeCapsule& Cap = ActiveCapsules[c];
		const float MinDist = Cap.Radius + RopeCollisionRadius;

		if (bUseSegmentCollision)
		{
			// --- Segment-vs-capsule: 각 rope segment를 swept sphere로 보아 capsule axis에
			//     대해 테스트하고, push-out을 양쪽 끝 노드로 나눈다. ---
			const int32 NumSegments = Positions.Num() - 1;
			for (int32 i = 0; i < NumSegments; ++i)
			{
				const float W0 = InvMasses[i];
				const float W1 = InvMasses[i + 1];
				if (W0 <= 0.0f && W1 <= 0.0f)
				{
					continue; // 양쪽 끝 모두 pin됨 — segment가 움직일 수 없음
				}

				// rope segment와 capsule의 내부 axis segment 사이의 최근접점.
				FVector OnRope, OnAxis;
				FMath::SegmentDistToSegmentSafe(Positions[i], Positions[i + 1], Cap.A, Cap.B, OnRope, OnAxis);

				const FVector Dir = OnRope - OnAxis;
				const float Dist = Dir.Size();
				if (Dist >= MinDist)
				{
					continue;
				}

				const FVector Normal = (Dist > KINDA_SMALL_NUMBER) ? (Dir / Dist) : FallbackNormal(Cap);
				const float Penetration = MinDist - Dist;

				// rope segment를 따라가는 접촉의 barycentric 위치 → 한 노드만이 아니라
				// segment 전체가 표면에서 떨어지도록 보정을 분배한다.
				const FVector Seg = Positions[i + 1] - Positions[i];
				const float SegLenSq = Seg.SizeSquared();
				const float S = (SegLenSq > KINDA_SMALL_NUMBER)
					? FMath::Clamp(FVector::DotProduct(OnRope - Positions[i], Seg) / SegLenSq, 0.0f, 1.0f)
					: 0.0f;
				const float C0 = 1.0f - S;
				const float C1 = S;

				// 접촉점에서의 가상 inverse mass: C0^2*W0 + C1^2*W1.
				const float WSum = C0 * C0 * W0 + C1 * C1 * W1;
				if (WSum <= 0.0f)
				{
					continue;
				}
				const float Lambda = Penetration / WSum;
				Positions[i]     += Normal * (W0 * C0 * Lambda);
				Positions[i + 1] += Normal * (W1 * C1 * Lambda);

				// S4: rope는 접촉점에서 push-out과 반대 방향으로 바디를 민다.
				if (bEnableTwoWayPull && CapsuleReaction.IsValidIndex(c))
				{
					CapsuleReaction[c] += -Normal * Penetration;
					CapsuleReactionPoint[c] += OnAxis;
					CapsuleReactionWeight[c] += 1.0f;
				}
			}
		}
		else
		{
			// --- Legacy 노드 전용 테스트(before/after 비교용으로 남겨둠). ---
			for (int32 i = 0; i < Positions.Num(); ++i)
			{
				if (InvMasses[i] <= 0.0f)
				{
					continue; // pin된 파티클은 밀지 않는다
				}

				const FVector Closest = FMath::ClosestPointOnSegment(Positions[i], Cap.A, Cap.B);
				const FVector ToParticle = Positions[i] - Closest;
				const float Dist = ToParticle.Size();
				if (Dist >= MinDist)
				{
					continue;
				}

				const FVector Normal = (Dist > KINDA_SMALL_NUMBER) ? (ToParticle / Dist) : FallbackNormal(Cap);
				const FVector Target = Closest + Normal * MinDist;
				const FVector PushOut = Target - Positions[i];
				Positions[i] = Target;

				if (bEnableTwoWayPull && CapsuleReaction.IsValidIndex(c))
				{
					CapsuleReaction[c] += -PushOut;
					CapsuleReactionPoint[c] += Closest;
					CapsuleReactionWeight[c] += 1.0f;
				}
			}
		}
	}
}

void ARopePoCActor::ApplyFriction()
{
	if (WrapFriction <= 0.0f)
	{
		return;
	}

	// 표면이 어떻게 움직였는지 추정하려면 같은 순서의 직전 substep capsule이 필요하다.
	if (PrevActiveCapsules.Num() != ActiveCapsules.Num())
	{
		return;
	}

	for (int32 c = 0; c < ActiveCapsules.Num(); ++c)
	{
		const FRopeCapsule& Cur = ActiveCapsules[c];
		const FRopeCapsule& Prev = PrevActiveCapsules[c];
		const float ContactDist = Cur.Radius + RopeCollisionRadius + FrictionContactBand;

		const FVector Seg = Cur.B - Cur.A;
		const float SegLenSq = Seg.SizeSquared();

		for (int32 i = 0; i < Positions.Num(); ++i)
		{
			if (InvMasses[i] <= 0.0f)
			{
				continue;
			}

			const FVector Closest = FMath::ClosestPointOnSegment(Positions[i], Cur.A, Cur.B);
			const FVector ToParticle = Positions[i] - Closest;
			const float Dist = ToParticle.Size();
			if (Dist > ContactDist)
			{
				continue; // 접촉 아님 — friction 없음
			}

			const FVector Normal = (Dist > KINDA_SMALL_NUMBER) ? (ToParticle / Dist) : FVector::UpVector;

			// 접촉점에서의 표면 속도를 샘플링한다(prev/cur segment에서 동일 파라미터).
			const float T = (SegLenSq > KINDA_SMALL_NUMBER)
				? FMath::Clamp(FVector::DotProduct(Positions[i] - Cur.A, Seg) / SegLenSq, 0.0f, 1.0f)
				: 0.0f;
			const FVector SurfaceVel = FMath::Lerp(Cur.A, Cur.B, T) - FMath::Lerp(Prev.A, Prev.B, T);

			// rope 파티클과 움직이는 표면 사이의 상대 tangential 운동.
			const FVector ParticleVel = Positions[i] - OldPositions[i];
			const FVector RelVel = ParticleVel - SurfaceVel;
			const FVector RelTangent = RelVel - FVector::DotProduct(RelVel, Normal) * Normal;

			// 그중 일부를 상쇄한다. Verlet에서는 OldPosition을 Position 쪽으로 옮기면 속도가 줄어든다.
			OldPositions[i] += RelTangent * WrapFriction;
		}
	}
}

void ARopePoCActor::ApplyPullReaction()
{
	if (!bEnableTwoWayPull || PullReactionGain <= 0.0f)
	{
		return;
	}

	for (int32 c = 0; c < FrameCapsules.Num(); ++c)
	{
		const float Weight = CapsuleReactionWeight.IsValidIndex(c) ? CapsuleReactionWeight[c] : 0.0f;
		if (Weight <= 0.0f)
		{
			continue; // 이번 프레임에 접촉 없음
		}

		// 이 capsule을 소유한 provider를 resolve한다.
		if (!FrameCapsuleOwner.IsValidIndex(c))
		{
			continue;
		}
		const int32 ProviderIdx = FrameCapsuleOwner[c];
		if (!CapsuleProviders.IsValidIndex(ProviderIdx))
		{
			continue;
		}
		UObject* Obj = CapsuleProviders[ProviderIdx].Get();
		IRopeCapsuleProvider* Provider = Obj ? Cast<IRopeCapsuleProvider>(Obj) : nullptr;
		if (!Provider)
		{
			continue;
		}

		const FVector Impulse = CapsuleReaction[c] * PullReactionGain;
		const FVector AppPoint = CapsuleReactionPoint[c] / Weight; // 평균낸 접촉점
		Provider->ApplyRopeReaction(Impulse, AppPoint);

		if (bDrawDebug)
		{
			if (const UWorld* World = GetWorld())
			{
				DrawDebugDirectionalArrow(World, AppPoint, AppPoint + Impulse * 20.0f,
					12.0f, FColor::Magenta, false, -1.0f, SDPG_World, 1.5f);
			}
		}
	}
}

void ARopePoCActor::UpdateLatchedPositions()
{
	if (!bEnableWrapLatch)
	{
		return;
	}

	for (int32 i = 0; i < Positions.Num(); ++i)
	{
		const int32 c = LatchCapsule[i];
		if (c < 0)
		{
			continue;
		}
		if (!ActiveCapsules.IsValidIndex(c))
		{
			// capsule 집합이 도중에 바뀜 — latch를 해제한다(내부 노드만 latch됨).
			LatchCapsule[i] = -1;
			InvMasses[i] = 1.0f;
			continue;
		}

		const FRopeCapsule& Cap = ActiveCapsules[c];
		const FVector AxisVec = Cap.B - Cap.A;
		const float AxisLen = AxisVec.Size();
		const FVector CurAxis = (AxisLen > KINDA_SMALL_NUMBER) ? (AxisVec / AxisLen) : LatchAxis[i];

		// 저장된 radial offset을 직전 step 이후 capsule axis가 회전한 만큼 회전시켜,
		// wrap이 swing하는 limb를 따라가게 한다(capsule은 axis 기준 대칭).
		const FQuat Turn = FQuat::FindBetweenNormals(LatchAxis[i], CurAxis);
		const FVector NewRadial = Turn.RotateVector(LatchRadialDir[i]).GetSafeNormal(1e-4f, CurAxis);
		LatchRadialDir[i] = NewRadial;
		LatchAxis[i] = CurAxis;

		// 표면점 = axis를 따라가는 점 + radial offset.
		const float Along = FMath::Clamp(LatchAlong[i], 0.0f, AxisLen);
		const FVector Surface = Cap.A + CurAxis * Along + NewRadial * LatchRadialDist[i];

		Positions[i] = Surface;
		OldPositions[i] = Surface; // pin됨: hold 중에는 속도를 이어가지 않는다
		InvMasses[i] = 0.0f;
	}
}

void ARopePoCActor::ManageWrapLatch(float FrameDt)
{
	if (!bEnableWrapLatch)
	{
		// 비활성화: 모든 latch를 해제해 rope를 다시 완전히 동적으로 만든다.
		for (int32 i = 0; i < Positions.Num(); ++i)
		{
			if (LatchCapsule[i] >= 0)
			{
				LatchCapsule[i] = -1;
				InvMasses[i] = 1.0f;
			}
			ContactDwell[i] = 0.0f;
		}
		return;
	}

	const float MaxLen = SegmentLength * LatchReleaseStrain;

	// (사용자가 제어하는) 끝점은 절대 latch하지 않는다.
	for (int32 i = 1; i < Positions.Num() - 1; ++i)
	{
		// 이미 latch됨 → 인접 segment가 한계를 넘어 늘어나면 해제한다.
		if (LatchCapsule[i] >= 0)
		{
			const bool bYanked =
				(Positions[i] - Positions[i - 1]).Size() > MaxLen ||
				(Positions[i + 1] - Positions[i]).Size() > MaxLen;
			if (bYanked)
			{
				LatchCapsule[i] = -1;
				InvMasses[i] = 1.0f;
				ContactDwell[i] = 0.0f;
			}
			continue;
		}

		// latch 안 됨 → 현재 접촉 중인 가장 가까운 capsule을 찾는다.
		int32 BestCap = -1;
		float BestDist = TNumericLimits<float>::Max();
		FVector BestClosest = FVector::ZeroVector;
		FVector BestAxis = FVector::UpVector;
		float BestAxisLen = 0.0f;

		for (int32 c = 0; c < ActiveCapsules.Num(); ++c)
		{
			const FRopeCapsule& Cap = ActiveCapsules[c];
			const float ContactDist = Cap.Radius + RopeCollisionRadius + LatchContactBand;
			const FVector Closest = FMath::ClosestPointOnSegment(Positions[i], Cap.A, Cap.B);
			const float Dist = (Positions[i] - Closest).Size();
			if (Dist <= ContactDist && Dist < BestDist)
			{
				const FVector AxisVec = Cap.B - Cap.A;
				const float AxisLen = AxisVec.Size();
				BestDist = Dist;
				BestCap = c;
				BestClosest = Closest;
				BestAxisLen = AxisLen;
				BestAxis = (AxisLen > KINDA_SMALL_NUMBER) ? (AxisVec / AxisLen) : FVector::UpVector;
			}
		}

		if (BestCap < 0)
		{
			ContactDwell[i] = 0.0f; // 아무것에도 닿지 않음 — dwell reset
			continue;
		}

		// 충분히 오래 dwell → latch를 성립시키고, 접촉을 capsule 기준으로 저장한다.
		ContactDwell[i] += FrameDt;
		if (ContactDwell[i] >= LatchContactTime)
		{
			const FVector Radial = Positions[i] - BestClosest;
			const float RadialDist = Radial.Size();

			LatchCapsule[i] = BestCap;
			// Along = 최근접점을 axis에 투영했을 때 capsule의 A 끝으로부터의 거리.
			LatchAlong[i] = FMath::Clamp(FVector::DotProduct(BestClosest - ActiveCapsules[BestCap].A, BestAxis), 0.0f, BestAxisLen);
			LatchRadialDir[i] = (RadialDist > KINDA_SMALL_NUMBER)
				? (Radial / RadialDist)
				: FVector::CrossProduct(BestAxis, FVector::ForwardVector).GetSafeNormal(1e-4f, FVector::RightVector);
			LatchRadialDist[i] = FMath::Max(RadialDist, 1e-3f);
			LatchAxis[i] = BestAxis;
			InvMasses[i] = 0.0f;
		}
	}
}

void ARopePoCActor::AccumulateLatchReaction()
{
	if (!bEnableTwoWayPull || !bEnableWrapLatch)
	{
		return;
	}

	for (int32 i = 1; i < Positions.Num() - 1; ++i)
	{
		const int32 c = LatchCapsule[i];
		if (c < 0 || !CapsuleReaction.IsValidIndex(c))
		{
			continue;
		}

		// latch된 노드는 limb에 pin되어 있어 push-out 반작용을 받지 않는다. 대신 이 노드의
		// rope tension(이웃 쪽으로 늘어난 segment)이 rope가 limb에 가하는 힘이다 —
		// capsule을 그 방향으로 당긴다.
		FVector Pull = FVector::ZeroVector;
		const int32 Neighbours[2] = { i - 1, i + 1 };
		for (int32 j : Neighbours)
		{
			const FVector D = Positions[j] - Positions[i];
			const float Len = D.Size();
			const float Stretch = Len - SegmentLength;
			if (Stretch > 0.0f && Len > KINDA_SMALL_NUMBER)
			{
				Pull += (D / Len) * Stretch;
			}
		}

		CapsuleReaction[c] += Pull;
		CapsuleReactionPoint[c] += Positions[i];
		CapsuleReactionWeight[c] += 1.0f;
	}
}

void ARopePoCActor::ReleaseAllWraps()
{
	for (int32 i = 0; i < LatchCapsule.Num(); ++i)
	{
		if (LatchCapsule[i] >= 0)
		{
			LatchCapsule[i] = -1;
			InvMasses[i] = 1.0f;
		}
		if (ContactDwell.IsValidIndex(i))
		{
			ContactDwell[i] = 0.0f;
		}
	}
}

void ARopePoCActor::UpdateSegmentMeshes()
{
	if (SegmentMeshes.Num() != Positions.Num() - 1)
	{
		return;
	}

	const FTransform ActorXf = GetActorTransform();

	// 부드러운 segment를 위한, 로컬 공간에서의 파티클별 tangent(Catmull 방식).
	const int32 N = Positions.Num();
	TArray<FVector, TInlineAllocator<64>> Local;
	Local.SetNum(N);
	for (int32 i = 0; i < N; ++i)
	{
		Local[i] = ActorXf.InverseTransformPosition(Positions[i]);
	}

	for (int32 i = 0; i < SegmentMeshes.Num(); ++i)
	{
		USplineMeshComponent* SM = SegmentMeshes[i];
		if (!SM)
		{
			continue;
		}

		const FVector StartPos = Local[i];
		const FVector EndPos = Local[i + 1];

		const FVector PrevA = Local[FMath::Max(i - 1, 0)];
		const FVector NextA = Local[FMath::Min(i + 1, N - 1)];
		const FVector PrevB = Local[FMath::Max(i, 0)];
		const FVector NextB = Local[FMath::Min(i + 2, N - 1)];

		FVector StartTangent = (NextA - PrevA) * 0.5f;
		FVector EndTangent = (NextB - PrevB) * 0.5f;
		if (StartTangent.IsNearlyZero()) { StartTangent = EndPos - StartPos; }
		if (EndTangent.IsNearlyZero()) { EndTangent = EndPos - StartPos; }

		SM->SetStartAndEnd(StartPos, StartTangent, EndPos, EndTangent, /*bUpdateMesh=*/true);
	}
}

void ARopePoCActor::DrawDebugRope() const
{
	const UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	for (int32 i = 0; i < Positions.Num(); ++i)
	{
		// Orange = latch된 wrap 노드, Red = pin된 끝점, Yellow = free 파티클.
		const bool bLatched = LatchCapsule.IsValidIndex(i) && LatchCapsule[i] >= 0;
		const FColor PointColor = bLatched ? FColor(255, 128, 0)
			: (InvMasses[i] <= 0.0f) ? FColor::Red : FColor::Yellow;
		DrawDebugPoint(World, Positions[i], 6.0f, PointColor, false, -1.0f, SDPG_World);
		if (i < Positions.Num() - 1)
		{
			DrawDebugLine(World, Positions[i], Positions[i + 1], FColor::Cyan, false, -1.0f, SDPG_World, 0.5f);
		}
	}
}

void ARopePoCActor::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (!bInitialized)
	{
		InitializeRope();
	}

	const double SolveStart = FPlatformTime::Seconds();
	SimulateStep(DeltaSeconds);
	const double SolveEnd = FPlatformTime::Seconds();

	LastSolveMs = static_cast<float>((SolveEnd - SolveStart) * 1000.0);
	AvgSolveMs = (AvgSolveMs <= 0.0f) ? LastSolveMs : FMath::Lerp(AvgSolveMs, LastSolveMs, 0.05f);

	UpdateSegmentMeshes();

	if (bDrawDebug)
	{
		DrawDebugRope();

		if (GEngine)
		{
			const FColor BudgetColor = (AvgSolveMs < 0.3f) ? FColor::Green : FColor::Orange;
			int32 LatchedCount = 0;
			for (int32 LC : LatchCapsule) { if (LC >= 0) { ++LatchedCount; } }
			GEngine->AddOnScreenDebugMessage(
				reinterpret_cast<uint64>(this), 0.0f, BudgetColor,
				FString::Printf(TEXT("[Rope] solve %.3f ms (avg) | particles %d | iters %d | sub %d | capsules %d | latched %d"),
					AvgSolveMs, NumParticles, SolverIterations, FMath::Clamp(SimSubsteps, 1, 16), FrameCapsules.Num(), LatchedCount));
		}
	}
}
