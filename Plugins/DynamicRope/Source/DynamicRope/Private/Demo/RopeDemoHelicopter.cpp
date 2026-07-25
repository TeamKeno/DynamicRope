// Copyright Epic Games, Inc. All Rights Reserved.

#include "Demo/RopeDemoHelicopter.h"
#include "RopeComponent.h"
#include "Logic/RopeAimTargeting.h"
#include "Core/RopeLifecycleTypes.h"
#include "Core/RopeThrowTypes.h"
#include "DynamicRopeLog.h"

#include "Components/SkeletalMeshComponent.h"
#include "Components/SphereComponent.h"
#include "Components/StaticMeshComponent.h"

ARopeDemoHelicopter::ARopeDemoHelicopter()
{
	// 호버/비행/로터/잡기 재시도에 틱이 필요하다.
	PrimaryActorTick.bCanEverTick = true;

	Base = CreateDefaultSubobject<USceneComponent>(TEXT("Base"));
	SetRootComponent(Base);

	// 메시는 레벨/BP에서 지정한다(플러그인 → /Game 참조 금지). 로터는 선택 — 없으면 그냥 빈 컴포넌트.
	BodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BodyMesh"));
	BodyMesh->SetupAttachment(Base);
	RotorMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RotorMesh"));
	RotorMesh->SetupAttachment(BodyMesh);

	// 케이블은 기체 하단 지점에 매단다(위치는 디테일에서 조정).
	RopeAttach = CreateDefaultSubobject<USceneComponent>(TEXT("RopeAttach"));
	RopeAttach->SetupAttachment(Base);
	RopeAttach->SetRelativeLocation(FVector(0.0f, 0.0f, -100.0f));

	Rope = CreateDefaultSubobject<URopeComponent>(TEXT("Rope"));
	Rope->SetupAttachment(RopeAttach);
	// 지정한 승객 본을 실패 없이 감아야 하는 데모라 ③ GuaranteedWrap 고정(BeginPlay에서 Loaded 대기).
	Rope->ResolveMode = ERopeWrapResolveMode::GuaranteedWrap;
	Rope->RopeLength = 700.0f;

	// 잡기 구역: 감지 전용(QueryOnly + Pawn 오버랩만) — CL 746 자격 필터(물리 충돌 ∧ PhysicsBody Block)에
	// 걸리지 않아 로프 정적 콜라이더로 수집되지 않는다(보이지 않는 구에 로프가 밀리는 사고 방지).
	GrabVolume = CreateDefaultSubobject<USphereComponent>(TEXT("GrabVolume"));
	GrabVolume->SetupAttachment(Base);
	GrabVolume->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
	GrabVolume->SetCollisionObjectType(ECC_WorldDynamic);
	GrabVolume->SetCollisionResponseToAllChannels(ECR_Ignore);
	GrabVolume->SetCollisionResponseToChannel(ECC_Pawn, ECR_Overlap);
	GrabVolume->SetGenerateOverlapEvents(true);
}

void ARopeDemoHelicopter::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	// 잡기 구역을 노브에 맞춰 재배치(에디터에서 수치 조정 즉시 반영).
	if (GrabVolume)
	{
		GrabVolume->SetRelativeLocation(FVector(0.0f, 0.0f, -GrabZoneDrop));
		GrabVolume->SetSphereRadius(GrabZoneRadius);
	}
}

void ARopeDemoHelicopter::BeginPlay()
{
	Super::BeginPlay();

	IdleAnchor = GetActorLocation();
	NavPos = IdleAnchor;
	NavYaw = static_cast<float>(GetActorRotation().Yaw);

	if (GrabVolume)
	{
		GrabVolume->OnComponentBeginOverlap.AddDynamic(this, &ARopeDemoHelicopter::HandleGrabZoneBeginOverlap);
	}
	if (!Rope)
	{
		UE_LOG(LogDynamicRope, Warning, TEXT("[%s] demo helicopter has no rope component — it will never grab."),
			*GetName());
	}
}

void ARopeDemoHelicopter::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	// 로터는 상태와 무관하게 항상 돈다(RPM → 도/초 = ×6).
	if (RotorMesh && RotorRPM > 0.0f)
	{
		RotorMesh->AddLocalRotation(FRotator(0.0f, RotorRPM * 6.0f * DeltaSeconds, 0.0f));
	}
	BobTime += DeltaSeconds;

	switch (State)
	{
	case EState::Idle:
		// 홈(또는 마지막 정지 지점)에서 호버.
		MoveTowards(IdleAnchor, DeltaSeconds);
		break;

	case EState::Grabbing:
	{
		if (!CarryTarget.IsValid() || !ResolveTargetMesh())
		{
			UE_LOG(LogDynamicRope, Log, TEXT("[%s] grab target lost — cancelling."), *GetName());
			CancelGrab();
			break;
		}
		if (GrabTimeout > 0.0f)
		{
			GrabElapsed += DeltaSeconds;
			if (GrabElapsed >= GrabTimeout)
			{
				UE_LOG(LogDynamicRope, Log, TEXT("[%s] grab timed out (%.1fs) — cancelling."), *GetName(), GrabTimeout);
				CancelGrab();
				break;
			}
		}
		if (Rope && Rope->GetPhase() == ERopePhase::Wrapped)
		{
			// 감김 성립 — 수송 시작.
			State = EState::Carrying;
			WaypointIndex = 0;
			bCarryBroadcast = true;
			UE_LOG(LogDynamicRope, Log, TEXT("[%s] passenger grabbed: %s (%s -> %s)."), *GetName(),
				*GetNameSafe(CarryTarget.Get()), *GrabBone.ToString(), *Rope->GetWrappedBoneName().ToString());
			OnCarryStateChanged.Broadcast(this, true);
			break;
		}
		// 미성립 — 주기 재발사(장전 에지 포함, 스네어와 동일 리듬).
		GrabRetryRemaining -= DeltaSeconds;
		if (GrabRetryRemaining <= 0.0f)
		{
			FireRopeAtTarget();
			GrabRetryRemaining = GrabRetryPeriod;
		}
		MoveTowards(IdleAnchor, DeltaSeconds);
		break;
	}

	case EState::Carrying:
	{
		if (!Rope || Rope->GetPhase() != ERopePhase::Wrapped)
		{
			// 외부 release/절단/대상 파괴 — 수송 종료 후 복귀.
			UE_LOG(LogDynamicRope, Log, TEXT("[%s] carried passenger lost mid-flight."), *GetName());
			RecallRope();
			if (bCarryBroadcast)
			{
				bCarryBroadcast = false;
				OnCarryStateChanged.Broadcast(this, false);
			}
			CarryTarget.Reset();
			State = bReturnHomeAfterRelease ? EState::Returning : EState::Idle;
			if (!bReturnHomeAfterRelease)
			{
				IdleAnchor = NavPos;
			}
			break;
		}

		// 릴-인으로 목표 길이까지 끌어올린다(+선택 능동 Pull).
		const float MaxLength = FMath::Max(Rope->RopeLength, Rope->MinRopeLength);
		const float TargetLength = FMath::Clamp(CarryRopeLength, Rope->MinRopeLength, MaxLength);
		const float CurrentLength = Rope->GetCurrentRopeLength();
		Rope->SetReelRate(CurrentLength > TargetLength + LiftTolerance ? LiftReelSpeed : 0.0f);
		Rope->SetActivePull(CarryPullForce);

		// 끌어올리는 동안은 제자리 호버 — 다 올라오면 경유지 순회.
		const bool bLifted = CurrentLength <= TargetLength + LiftTolerance;
		if (!bLifted || !Waypoints.IsValidIndex(WaypointIndex) || !Waypoints[WaypointIndex])
		{
			// 경유지가 없거나(호버 크레인) 아직 끌어올리는 중 — 제자리 유지.
			MoveTowards(NavPos, DeltaSeconds);
			break;
		}
		const float Remaining = MoveTowards(Waypoints[WaypointIndex]->GetActorLocation(), DeltaSeconds);
		if (Remaining <= WaypointTolerance)
		{
			++WaypointIndex;
			if (WaypointIndex >= Waypoints.Num() && bReleaseAtLastWaypoint)
			{
				ReleaseCarried();
			}
		}
		break;
	}

	case EState::Returning:
		if (MoveTowards(IdleAnchor, DeltaSeconds) <= WaypointTolerance)
		{
			State = EState::Idle;
		}
		break;
	}
}

bool ARopeDemoHelicopter::Grab(AActor* Passenger)
{
	if (State != EState::Idle || !Rope)
	{
		return false;
	}
	if (!Passenger || Passenger == this
		|| !Passenger->FindComponentByClass<USkeletalMeshComponent>())
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] grab rejected: '%s' has no skeletal mesh."),
			*GetName(), *GetNameSafe(Passenger));
		return false;
	}
	CarryTarget = Passenger;
	State = EState::Grabbing;
	GrabRetryRemaining = 0.0f; // 다음 틱에 곧바로 첫 발사.
	GrabElapsed = 0.0f;
	return true;
}

void ARopeDemoHelicopter::CancelGrab()
{
	if (State != EState::Grabbing)
	{
		return;
	}
	RecallRope();
	CarryTarget.Reset();
	State = EState::Idle;
}

void ARopeDemoHelicopter::ReleaseCarried()
{
	if (State != EState::Carrying)
	{
		return;
	}
	RecallRope();
	if (bCarryBroadcast)
	{
		bCarryBroadcast = false;
		OnCarryStateChanged.Broadcast(this, false);
	}
	UE_LOG(LogDynamicRope, Log, TEXT("[%s] passenger released: %s."), *GetName(), *GetNameSafe(CarryTarget.Get()));
	CarryTarget.Reset();
	State = bReturnHomeAfterRelease ? EState::Returning : EState::Idle;
	if (!bReturnHomeAfterRelease)
	{
		// 이 자리가 새 대기 지점이 된다(다음 승객을 여기서 기다림).
		IdleAnchor = NavPos;
	}
}

bool ARopeDemoHelicopter::IsCarrying() const
{
	return State == EState::Carrying;
}

void ARopeDemoHelicopter::RecallRope()
{
	if (!Rope)
	{
		return;
	}
	Rope->SetReelRate(0.0f);
	Rope->SetActivePull(0.0f);
	// 아직 안 나간 발사 큐 폐기 + 전 페이즈 회수(비행 중/잡는 중 포함 — ReleaseWrapAs가 게이트, 그 외 no-op).
	// 스네어의 빠른 해제 구멍(CL 768)과 동일한 위생: 안 걷으면 취소 *후에* 케이블이 마저 날아가 감긴다.
	Rope->CancelQueuedGuaranteedAimThrow();
	Rope->ReleaseWrap();
	// 다음 잡기가 같은 길이에서 시작하도록 감았던 만큼 되돌린다.
	Rope->SetRopeLength(Rope->RopeLength);
}

USkeletalMeshComponent* ARopeDemoHelicopter::ResolveTargetMesh() const
{
	AActor* Target = CarryTarget.Get();
	return Target ? Target->FindComponentByClass<USkeletalMeshComponent>() : nullptr;
}

bool ARopeDemoHelicopter::FireRopeAtTarget()
{
	USkeletalMeshComponent* Mesh = ResolveTargetMesh();
	if (!Rope || !Mesh)
	{
		return false;
	}
	if (Mesh->GetBoneIndex(GrabBone) == INDEX_NONE)
	{
		UE_LOG(LogDynamicRope, Warning, TEXT("[%s] grab bone '%s' not found on '%s' — cancelling."),
			*GetName(), *GrabBone.ToString(), *Mesh->GetName());
		return false;
	}

	// ③은 Loaded(장전)에서만 던질 수 있다. 아니면 장전만 하고 다음 재시도에서 발사한다(장전 에지 안정화).
	if (Rope->GetPhase() != ERopePhase::Loaded)
	{
		Rope->EnterLoaded();
		return false;
	}

	const FVector Origin = Rope->GetComponentLocation();
	const FVector BoneWorld = Mesh->GetSocketLocation(GrabBone);
	const FVector ToBone = BoneWorld - Origin;
	const float Dist = static_cast<float>(ToBone.Size());
	if (Dist < KINDA_SMALL_NUMBER)
	{
		return false;
	}
	const FVector AimDir = ToBone / Dist;

	// Wielder의 BuildAimRayThrowRequest 최소 복제(입력 없이 본 고정 조준) — 스네어 FireSnareRopeFor와 동일 계약.
	FRopeAimRayThrowRequest Request;
	FRopeThrowContext& Ctx = Request.BaseContext;
	Ctx.Origin = Origin;
	Ctx.FrameForward = AimDir;
	Ctx.FrameUp = FVector::UpVector;
	Ctx.FrameRight = FVector::CrossProduct(AimDir, FVector::UpVector).GetSafeNormal();
	if (Ctx.FrameRight.IsNearlyZero())
	{
		// 수직 낙하 조준(AimDir∥Up) — 헬기는 바로 아래를 쏘는 게 기본이라 이 축퇴가 상시 경로다.
		Ctx.FrameRight = FVector::RightVector;
	}
	Ctx.FrameMode = ERopeThrowFrameMode::Custom;
	Ctx.bAimRayEvaluated = true;

	Request.RayOrigin = Origin;
	Request.RayDirection = AimDir;
	Request.ReachOrigin = Origin;
	Request.ReachLength = Dist + 200.0f; // 도달 여유(본을 확실히 포함).
	Request.RayLength = FRopeAimTargeting::ResolveRayLengthForReach(
		Request.RayOrigin, Request.RayDirection, Request.ReachOrigin, Request.ReachLength);
	Request.QueryRadius = 0.0f;
	Request.SweepStep = 2.0f;

	return Rope->QueueGuaranteedAimThrow(Request, /*bExecuteWhenReady*/ true);
}

float ARopeDemoHelicopter::MoveTowards(const FVector& Dest, float DeltaSeconds)
{
	const FVector Delta = Dest - NavPos;
	const float Remaining = static_cast<float>(Delta.Size());
	if (Remaining > KINDA_SMALL_NUMBER)
	{
		NavPos = FMath::VInterpConstantTo(NavPos, Dest, DeltaSeconds, FlySpeed);
		if (bFaceTravelDirection && Remaining > WaypointTolerance)
		{
			const float DesiredYaw = static_cast<float>(Delta.Rotation().Yaw);
			NavYaw = FMath::FixedTurn(NavYaw, DesiredYaw, TurnRateDeg * DeltaSeconds);
		}
	}
	ApplyPose();
	return Remaining;
}

void ARopeDemoHelicopter::ApplyPose()
{
	// 흔들림은 표시에만 얹는다 — 이동 판정(NavPos)에는 섞지 않아 경유지 도착이 흔들림에 출렁이지 않는다.
	FVector Shown = NavPos;
	if (HoverBobAmplitude > 0.0f && HoverBobPeriod > KINDA_SMALL_NUMBER)
	{
		Shown.Z += HoverBobAmplitude * FMath::Sin(BobTime * (2.0f * UE_PI / HoverBobPeriod));
	}
	SetActorLocationAndRotation(Shown, FRotator(0.0f, NavYaw, 0.0f));
}

void ARopeDemoHelicopter::HandleGrabZoneBeginOverlap(UPrimitiveComponent* /*OverlappedComponent*/,
	AActor* OtherActor, UPrimitiveComponent* /*OtherComp*/, int32 /*OtherBodyIndex*/, bool /*bFromSweep*/,
	const FHitResult& /*SweepResult*/)
{
	if (!bAutoGrab || State != EState::Idle)
	{
		return;
	}
	Grab(OtherActor);
}
