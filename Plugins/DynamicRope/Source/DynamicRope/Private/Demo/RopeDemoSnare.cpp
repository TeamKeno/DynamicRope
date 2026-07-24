// Copyright Epic Games, Inc. All Rights Reserved.

#include "Demo/RopeDemoSnare.h"
#include "Demo/RopeDemoPressurePlate.h"
#include "Gameplay/RopeRagdollResponseComponent.h"
#include "RopeComponent.h"
#include "Logic/RopeAimTargeting.h"
#include "Core/RopeLifecycleTypes.h"
#include "Core/RopeThrowTypes.h"
#include "DynamicRopeLog.h"

#include "Components/SkeletalMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "UObject/ConstructorHelpers.h"

ARopeDemoSnare::ARopeDemoSnare()
{
	// 결박 확립 폴링 + 릴 구동에 틱이 필요하다.
	PrimaryActorTick.bCanEverTick = true;

	Base = CreateDefaultSubobject<USceneComponent>(TEXT("Base"));
	SetRootComponent(Base);

	// 콘텐츠 의존을 만들지 않으려고 엔진 기본 셰이프만 쓴다(플러그인 → /Game 참조 금지).
	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeMesh(TEXT("/Engine/BasicShapes/Cube.Cube"));

	AnchorMarkers.Reserve(MaxBindings);
	Ropes.Reserve(MaxBindings);
	for (int32 Index = 0; Index < MaxBindings; ++Index)
	{
		UStaticMeshComponent* Marker = CreateDefaultSubobject<UStaticMeshComponent>(*FString::Printf(TEXT("Anchor%d"), Index));
		if (Marker)
		{
			Marker->SetupAttachment(Base);
			// 15cm 표식 — 앵커 위치만 보이면 되므로 충돌은 끈다(로프/캐릭터와 간섭 금지).
			Marker->SetRelativeScale3D(FVector(0.15f));
			Marker->SetCollisionEnabled(ECollisionEnabled::NoCollision);
			if (CubeMesh.Succeeded())
			{
				Marker->SetStaticMesh(CubeMesh.Object);
			}
			AnchorMarkers.Add(Marker);
		}

		URopeComponent* Cable = CreateDefaultSubobject<URopeComponent>(*FString::Printf(TEXT("Rope%d"), Index));
		if (Cable)
		{
			Cable->SetupAttachment(Marker ? static_cast<USceneComponent*>(Marker) : static_cast<USceneComponent*>(Base));
			// 지정한 팔다리를 실패 없이 감아야 하는 데모라 ③ GuaranteedWrap 고정.
			Cable->ResolveMode = ERopeWrapResolveMode::GuaranteedWrap;
			Ropes.Add(Cable);
		}
	}

	// 기본 = 양팔 2슬롯(2로프 스파이크). 대상이 +X를 보고 원점에 선 배치를 가정한다(왼쪽 = -Y).
	// 다리 2슬롯(foot_l / foot_r)을 추가하면 그대로 완전한 대자가 된다.
	Bindings.Reserve(MaxBindings);
	{
		FRopeDemoSnareBinding LeftHand;
		LeftHand.Bone = TEXT("hand_l");
		LeftHand.AnchorOffset = FVector(0.0f, -250.0f, 200.0f);
		Bindings.Add(LeftHand);

		FRopeDemoSnareBinding RightHand;
		RightHand.Bone = TEXT("hand_r");
		RightHand.AnchorOffset = FVector(0.0f, 250.0f, 200.0f);
		Bindings.Add(RightHand);
	}
}

void ARopeDemoSnare::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	// 에디터에서 AnchorOffset을 만지는 즉시 표식/로프가 따라오도록 배치를 재적용한다.
	ApplyBindingLayout();
}

void ARopeDemoSnare::BeginPlay()
{
	Super::BeginPlay();

	ApplyBindingLayout();

	if (TriggerPlate)
	{
		TriggerPlate->OnPlatePressedChanged.AddDynamic(this, &ARopeDemoSnare::HandleTriggerPlateChanged);
	}

	if (!TargetActor)
	{
		UE_LOG(LogDynamicRope, Warning,
			TEXT("[%s] demo snare has no TargetActor — set the character/ragdoll to bind or it will never fire."),
			*GetName());
	}
	else if (!ResolveTargetMesh())
	{
		UE_LOG(LogDynamicRope, Warning,
			TEXT("[%s] demo snare target '%s' has no skeletal mesh — nothing to wrap."),
			*GetName(), *TargetActor->GetName());
	}

	if (GetActiveBindingCount() == 0)
	{
		UE_LOG(LogDynamicRope, Warning,
			TEXT("[%s] demo snare has no active bindings — fill Bindings with limb bone names."), *GetName());
	}

	if (bSnareOnBeginPlay)
	{
		TriggerSnare();
	}
}

void ARopeDemoSnare::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (TriggerPlate)
	{
		TriggerPlate->OnPlatePressedChanged.RemoveDynamic(this, &ARopeDemoSnare::HandleTriggerPlateChanged);
	}

	Super::EndPlay(EndPlayReason);
}

void ARopeDemoSnare::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (!bTriggered || GetActiveBindingCount() == 0)
	{
		return;
	}

	//~ 1) 결박 확립 단계 — 활성 슬롯이 모두 감길 때까지 주기적으로 재발사한다.
	if (!bSnared)
	{
		if (AreAllBoundRopesWrapped())
		{
			SetSnared(true);
			return;
		}

		FireRetryRemaining -= DeltaSeconds;
		if (FireRetryRemaining <= 0.0f)
		{
			FireSnareRopes();
			FireRetryRemaining = 1.0f; // 1초 간격 재시도(gather/장전 에지 안정화 여유).
		}
		return;
	}

	//~ 2) 하나라도 풀렸으면(대상 소실/거리 해제 등) 확립 단계로 되돌아간다.
	if (!AreAllBoundRopesWrapped())
	{
		for (URopeComponent* Cable : Ropes)
		{
			if (Cable)
			{
				Cable->SetReelRate(0.0f);
				Cable->SetActivePull(0.0f);
			}
		}
		SetSnared(false);
		FireRetryRemaining = 0.5f;
		return;
	}

	//~ 3) 사지를 벌린다 — 목표 길이까지 릴-인(+슬롯당 능동 Pull). 테더가 본을 앵커 쪽으로 끌어당긴다.
	for (int32 Index = 0; Index < Ropes.Num(); ++Index)
	{
		if (!IsSlotActive(Index))
		{
			continue;
		}
		URopeComponent* Cable = Ropes[Index];
		const float MaxLength = FMath::Max(Cable->RopeLength, Cable->MinRopeLength);
		const float TargetLength = (SnareLength > 0.0f)
			? FMath::Clamp(SnareLength, Cable->MinRopeLength, MaxLength)
			: Cable->MinRopeLength;

		Cable->SetReelRate(Cable->GetCurrentRopeLength() > TargetLength + ArrivalTolerance ? SnareReelSpeed : 0.0f);
		Cable->SetActivePull(LimbPullForce);
	}
}

void ARopeDemoSnare::TriggerSnare()
{
	if (bTriggered)
	{
		return;
	}
	bTriggered = true;
	FireRetryRemaining = 0.0f; // 다음 틱에 곧바로 첫 발사.

	ForceTargetRagdoll();
}

void ARopeDemoSnare::ReleaseSnare()
{
	if (!bTriggered)
	{
		return;
	}
	bTriggered = false;

	for (URopeComponent* Cable : Ropes)
	{
		if (!Cable)
		{
			continue;
		}
		Cable->SetReelRate(0.0f);
		Cable->SetActivePull(0.0f);
		if (Cable->GetPhase() == ERopePhase::Wrapped)
		{
			Cable->ReleaseWrap();
		}
		// 다음 결박이 같은 길이에서 시작하도록 감았던 만큼 되돌린다.
		Cable->SetRopeLength(Cable->RopeLength);
	}

	SetSnared(false);
}

void ARopeDemoSnare::ToggleSnare()
{
	if (bTriggered)
	{
		ReleaseSnare();
	}
	else
	{
		TriggerSnare();
	}
}

int32 ARopeDemoSnare::GetActiveBindingCount() const
{
	int32 Count = 0;
	for (int32 Index = 0; Index < Ropes.Num(); ++Index)
	{
		if (IsSlotActive(Index))
		{
			++Count;
		}
	}
	return Count;
}

int32 ARopeDemoSnare::GetBoundRopeCount() const
{
	int32 Count = 0;
	for (int32 Index = 0; Index < Ropes.Num(); ++Index)
	{
		if (IsSlotActive(Index) && Ropes[Index]->GetPhase() == ERopePhase::Wrapped)
		{
			++Count;
		}
	}
	return Count;
}

bool ARopeDemoSnare::IsSlotActive(int32 SlotIndex) const
{
	return Ropes.IsValidIndex(SlotIndex)
		&& Ropes[SlotIndex] != nullptr
		&& Bindings.IsValidIndex(SlotIndex)
		&& !Bindings[SlotIndex].Bone.IsNone();
}

void ARopeDemoSnare::ApplyBindingLayout()
{
	for (int32 Index = 0; Index < MaxBindings; ++Index)
	{
		const bool bActive = IsSlotActive(Index);
		const FVector Offset = Bindings.IsValidIndex(Index) ? Bindings[Index].AnchorOffset : FVector::ZeroVector;

		if (AnchorMarkers.IsValidIndex(Index) && AnchorMarkers[Index])
		{
			AnchorMarkers[Index]->SetRelativeLocation(Offset);
			AnchorMarkers[Index]->SetVisibility(bActive);
		}
		if (Ropes.IsValidIndex(Index) && Ropes[Index])
		{
			// 로프는 표식에 붙어 있으므로 상대 위치는 0 — 미사용 슬롯만 숨긴다.
			Ropes[Index]->SetVisibility(bActive);
		}
	}
}

bool ARopeDemoSnare::AreAllBoundRopesWrapped() const
{
	const int32 ActiveCount = GetActiveBindingCount();
	return ActiveCount > 0 && GetBoundRopeCount() == ActiveCount;
}

USkeletalMeshComponent* ARopeDemoSnare::ResolveTargetMesh() const
{
	return TargetActor ? TargetActor->FindComponentByClass<USkeletalMeshComponent>() : nullptr;
}

void ARopeDemoSnare::ForceTargetRagdoll()
{
	if (!bForceRagdollOnSnare || !TargetActor)
	{
		return;
	}
	// 사지가 순순히 벌어지려면 **감기 전에** 물리로 넘어가 있어야 한다. 응답 컴포넌트가 없으면
	// 감김 이벤트 기반 자동 전환도 없다는 뜻이라 그대로 둔다(정적 메시 대상 등).
	if (URopeRagdollResponseComponent* Response = TargetActor->FindComponentByClass<URopeRagdollResponseComponent>())
	{
		if (!Response->IsRagdolled())
		{
			Response->EnterRagdoll();
		}
	}
}

void ARopeDemoSnare::SetSnared(bool bNewSnared)
{
	if (bSnared == bNewSnared)
	{
		return;
	}
	bSnared = bNewSnared;

	if (bSnared)
	{
		// 실제로 어떤 본이 잡혔는지는 조준한 본과 다를 수 있다(가로막은 상위 본) — 실측 판독용으로 찍는다.
		FString BoundBones;
		for (int32 Index = 0; Index < Ropes.Num(); ++Index)
		{
			if (IsSlotActive(Index))
			{
				BoundBones += FString::Printf(TEXT("%s%s->%s"), BoundBones.IsEmpty() ? TEXT("") : TEXT(", "),
					*Bindings[Index].Bone.ToString(), *Ropes[Index]->GetWrappedBoneName().ToString());
			}
		}
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] snare bound (%d cables): %s"), *GetName(), GetBoundRopeCount(), *BoundBones);
	}
	else
	{
		UE_LOG(LogDynamicRope, Log, TEXT("[%s] snare released."), *GetName());
	}

	OnSnareStateChanged.Broadcast(this, bSnared);
}

void ARopeDemoSnare::HandleTriggerPlateChanged(ARopeDemoPressurePlate* /*Plate*/, bool bPressed)
{
	// 함정 트리거: 밟으면 결박, 벗어나면 해제.
	if (bPressed)
	{
		TriggerSnare();
	}
	else
	{
		ReleaseSnare();
	}
}

void ARopeDemoSnare::FireSnareRopes()
{
	for (int32 Index = 0; Index < Ropes.Num(); ++Index)
	{
		if (IsSlotActive(Index) && Ropes[Index]->GetPhase() != ERopePhase::Wrapped)
		{
			FireSnareRopeFor(Index);
		}
	}
}

bool ARopeDemoSnare::FireSnareRopeFor(int32 SlotIndex)
{
	if (!IsSlotActive(SlotIndex))
	{
		return false;
	}
	USkeletalMeshComponent* Mesh = ResolveTargetMesh();
	if (!Mesh)
	{
		return false;
	}

	URopeComponent* Cable = Ropes[SlotIndex];
	const FName Bone = Bindings[SlotIndex].Bone;
	if (Mesh->GetBoneIndex(Bone) == INDEX_NONE)
	{
		UE_LOG(LogDynamicRope, Warning, TEXT("[%s] snare slot %d bone '%s' not found on '%s'."),
			*GetName(), SlotIndex, *Bone.ToString(), *Mesh->GetName());
		return false;
	}

	// ③은 Loaded(장전)에서만 던질 수 있다. 아니면 장전만 하고 다음 시도에서 발사한다(장전 에지 안정화).
	if (Cable->GetPhase() != ERopePhase::Loaded)
	{
		Cable->EnterLoaded();
		return false;
	}

	const FVector Origin = Cable->GetComponentLocation();
	const FVector BoneWorld = Mesh->GetSocketLocation(Bone);
	const FVector ToBone = BoneWorld - Origin;
	const float Dist = ToBone.Size();
	if (Dist < KINDA_SMALL_NUMBER)
	{
		return false;
	}
	const FVector AimDir = ToBone / Dist;

	// Wielder의 BuildAimRayThrowRequest를 최소 복제한다(입력 없이 본을 고정 조준).
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
	Request.ReachLength = Dist + 200.0f; // 도달 여유(본을 확실히 포함).
	Request.RayLength = FRopeAimTargeting::ResolveRayLengthForReach(
		Request.RayOrigin, Request.RayDirection, Request.ReachOrigin, Request.ReachLength);
	Request.QueryRadius = 0.0f;
	Request.SweepStep = 2.0f;

	// 몽타주 없이 정상 gather 직후 즉시 실행한다.
	return Cable->QueueGuaranteedAimThrow(Request, /*bExecuteWhenReady*/ true);
}
