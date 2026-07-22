// Copyright Epic Games, Inc. All Rights Reserved.

#include "Demo/RopeDemoElevator.h"
#include "Demo/RopeDemoPressurePlate.h"
#include "RopeComponent.h"
#include "Logic/RopeAimTargeting.h"
#include "Core/RopeLifecycleTypes.h"
#include "Core/RopeThrowTypes.h"
#include "DynamicRopeLog.h"

#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "UObject/ConstructorHelpers.h"

ARopeDemoElevator::ARopeDemoElevator()
{
	// 승강 구동 + 그래플 확립 폴링에 틱이 필요하다.
	PrimaryActorTick.bCanEverTick = true;

	Platform = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Platform"));
	SetRootComponent(Platform);
	// 200x200x20cm 판(엔진 큐브 100cm 기준). 물리 바디 = climb-in 견인 수신자(SimBody).
	Platform->SetRelativeScale3D(FVector(2.0f, 2.0f, 0.2f));
	Platform->SetMobility(EComponentMobility::Movable);
	Platform->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	Platform->SetCollisionObjectType(ECC_PhysicsBody);
	Platform->SetSimulatePhysics(true);

	// 콘텐츠 의존을 만들지 않으려고 엔진 기본 셰이프만 쓴다(플러그인 → /Game 참조 금지).
	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeMesh(TEXT("/Engine/BasicShapes/Cube.Cube"));
	if (CubeMesh.Succeeded())
	{
		Platform->SetStaticMesh(CubeMesh.Object);
	}

	Rope = CreateDefaultSubobject<URopeComponent>(TEXT("Rope"));
	Rope->SetupAttachment(Platform);
	// 플랫폼 윗면에서 로프가 나간다(판 절반 두께 ≈ 10cm 위).
	Rope->SetRelativeLocation(FVector(0.0f, 0.0f, 10.0f));
	// 천장 앵커를 실패 없이 감아야 하는 데모라 ③ GuaranteedWrap 고정.
	Rope->ResolveMode = ERopeWrapResolveMode::GuaranteedWrap;
}

void ARopeDemoElevator::BeginPlay()
{
	Super::BeginPlay();

	if (CallPlate)
	{
		CallPlate->OnPlatePressedChanged.AddDynamic(this, &ARopeDemoElevator::HandleCallPlateChanged);
	}

	if (!AnchorTarget)
	{
		UE_LOG(LogDynamicRope, Warning,
			TEXT("[%s] demo elevator has no AnchorTarget — set a wrappable ceiling anchor (skeletal bone collider / SDF) or it will never move."),
			*GetName());
	}

	// 시작은 아래층(바닥에서 대기). 그래플은 첫 틱부터 확립을 시도한다.
	bTargetTop = false;
	bArrivedBroadcast = false;
}

void ARopeDemoElevator::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (CallPlate)
	{
		CallPlate->OnPlatePressedChanged.RemoveDynamic(this, &ARopeDemoElevator::HandleCallPlateChanged);
	}

	Super::EndPlay(EndPlayReason);
}

void ARopeDemoElevator::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (!Rope)
	{
		return;
	}

	//~ 1) 그래플 확립 단계 — 천장 앵커를 감을 때까지 주기적으로 재발사한다.
	if (!bGrappleReady)
	{
		if (Rope->GetPhase() == ERopePhase::Wrapped)
		{
			bGrappleReady = true;
			bArrivedBroadcast = false;
			UE_LOG(LogDynamicRope, Log, TEXT("[%s] elevator grapple established (bone='%s')."),
				*GetName(), *Rope->GetWrappedBoneName().ToString());
			return;
		}

		if (AnchorTarget)
		{
			EstablishRetryRemaining -= DeltaSeconds;
			if (EstablishRetryRemaining <= 0.0f)
			{
				FireGrapple();
				EstablishRetryRemaining = 1.0f; // 1초 간격 재시도(gather/장전 에지 안정화 여유).
			}
		}
		return;
	}

	//~ 2) 그래플이 풀렸으면(대상 소실 등) 확립 단계로 되돌아간다.
	if (Rope->GetPhase() != ERopePhase::Wrapped)
	{
		bGrappleReady = false;
		EstablishRetryRemaining = 0.5f;
		Rope->SetReelRate(0.0f);
		Rope->SetActivePull(0.0f);
		return;
	}

	//~ 3) 목표 층으로 릴 구동. 상승=릴-인(+climb-in), 하강=릴-아웃(중력).
	const float Length = Rope->GetCurrentRopeLength();
	const float MinLength = Rope->MinRopeLength;
	const float MaxLength = FMath::Max(Rope->RopeLength, MinLength);

	if (bTargetTop)
	{
		if (Length > MinLength + ArrivalTolerance)
		{
			Rope->SetReelRate(AscendReelSpeed);
			if (ClimbForce > 0.0f)
			{
				Rope->SetActivePull(ClimbForce);
			}
		}
		else
		{
			Rope->SetReelRate(0.0f);
			Rope->SetActivePull(0.0f);
			if (!bArrivedBroadcast)
			{
				bArrivedBroadcast = true;
				UE_LOG(LogDynamicRope, Log, TEXT("[%s] elevator arrived at top."), *GetName());
				OnElevatorArrived.Broadcast(this, /*bAtTop*/ true);
			}
		}
	}
	else
	{
		if (Length < MaxLength - ArrivalTolerance)
		{
			Rope->SetReelRate(-DescendReelSpeed);
			Rope->SetActivePull(0.0f);
		}
		else
		{
			Rope->SetReelRate(0.0f);
			Rope->SetActivePull(0.0f);
			if (!bArrivedBroadcast)
			{
				bArrivedBroadcast = true;
				UE_LOG(LogDynamicRope, Log, TEXT("[%s] elevator arrived at bottom."), *GetName());
				OnElevatorArrived.Broadcast(this, /*bAtTop*/ false);
			}
		}
	}
}

void ARopeDemoElevator::SetTargetTop(bool bNewTargetTop)
{
	if (bTargetTop == bNewTargetTop)
	{
		return;
	}
	bTargetTop = bNewTargetTop;
	bArrivedBroadcast = false; // 새 목표 → 도착 판정 재개.
}

void ARopeDemoElevator::HandleCallPlateChanged(ARopeDemoPressurePlate* /*Plate*/, bool bPressed)
{
	// 호출 버튼: 누르면 위층, 풀면 아래층.
	SetTargetTop(bPressed);
}

FVector ARopeDemoElevator::ResolveAnchorAimWorld() const
{
	// 앵커 액터 위치를 조준한다 — swept aim ray가 그 방향의 wrappable 본을 sweep해 잠근다.
	return AnchorTarget ? AnchorTarget->GetActorLocation() : GetActorLocation();
}

bool ARopeDemoElevator::FireGrapple()
{
	if (!Rope || !AnchorTarget)
	{
		return false;
	}

	// ③은 Loaded(장전)에서만 던질 수 있다. 아니면 장전만 하고 다음 시도에서 발사한다(장전 에지 안정화).
	if (Rope->GetPhase() != ERopePhase::Loaded)
	{
		Rope->EnterLoaded();
		return false;
	}

	const FVector Origin = Rope->GetComponentLocation();
	const FVector AnchorWorld = ResolveAnchorAimWorld();
	const FVector ToAnchor = AnchorWorld - Origin;
	const float Dist = ToAnchor.Size();
	if (Dist < KINDA_SMALL_NUMBER)
	{
		return false;
	}
	const FVector AimDir = ToAnchor / Dist;

	// Wielder의 BuildAimRayThrowRequest를 최소 복제한다(입력 없이 앵커를 고정 조준).
	FRopeAimRayThrowRequest Request;
	FRopeThrowContext& Ctx = Request.BaseContext;
	Ctx.Origin = Origin;
	Ctx.FrameForward = AimDir;
	Ctx.FrameUp = FVector::UpVector;
	Ctx.FrameRight = FVector::CrossProduct(AimDir, FVector::UpVector).GetSafeNormal();
	if (Ctx.FrameRight.IsNearlyZero())
	{
		Ctx.FrameRight = FVector::RightVector; // 수직 조준(AimDir∥Up) 축퇴 폴백.
	}
	Ctx.FrameMode = ERopeThrowFrameMode::Custom;
	Ctx.bAimRayEvaluated = true; // 조준 ray가 만든 컨텍스트임을 표시(preview 빌더 계약).

	Request.RayOrigin = Origin;
	Request.RayDirection = AimDir;
	Request.ReachOrigin = Origin;
	Request.ReachLength = Dist + 200.0f; // 도달 여유(앵커를 확실히 포함).
	Request.RayLength = FRopeAimTargeting::ResolveRayLengthForReach(
		Request.RayOrigin, Request.RayDirection, Request.ReachOrigin, Request.ReachLength);
	Request.QueryRadius = 0.0f;
	Request.SweepStep = 2.0f;

	// 몽타주 없이 정상 gather 직후 즉시 실행한다.
	return Rope->QueueGuaranteedAimThrow(Request, /*bExecuteWhenReady*/ true);
}
