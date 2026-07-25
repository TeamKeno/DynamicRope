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

	// 기본 = 사지 4슬롯(완전한 대자). 대상이 +X를 보고 원점에 선 배치를 가정한다(왼쪽 = -Y).
	// 손은 위쪽 대각, 발은 아래쪽 대각 앵커 — 레벨에서 슬롯을 비우거나(Bone 비움/항목 삭제)
	// AnchorOffset을 조정해 자유롭게 줄인다(2개만 남기면 종전 양팔 결박).
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

		FRopeDemoSnareBinding LeftFoot;
		LeftFoot.Bone = TEXT("foot_l");
		LeftFoot.AnchorOffset = FVector(0.0f, -250.0f, 30.0f);
		Bindings.Add(LeftFoot);

		FRopeDemoSnareBinding RightFoot;
		RightFoot.Bone = TEXT("foot_r");
		RightFoot.AnchorOffset = FVector(0.0f, 250.0f, 30.0f);
		Bindings.Add(RightFoot);
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
		// 판이 있으면 덫 모드(눌림 순간 점유 액터를 잡는다)라 무대상이 정상이다.
		if (!TriggerPlate)
		{
			UE_LOG(LogDynamicRope, Warning,
				TEXT("[%s] demo snare has no TargetActor and no TriggerPlate — set a target, or wire a plate for trap mode, or it will never fire."),
				*GetName());
		}
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

	//~ 2.5) 자동 놓기: 결박 성립 후 AutoReleaseDelay가 지나면 놓는다(0 = 끔). 판이 계속 눌려 있어도
	// 재결박하지 않는다 — 재무장은 판의 다음 눌림 에지나 수동 TriggerSnare 몫이다.
	if (AutoReleaseDelay > 0.0f)
	{
		AutoReleaseRemaining -= DeltaSeconds;
		if (AutoReleaseRemaining <= 0.0f)
		{
			UE_LOG(LogDynamicRope, Log, TEXT("[%s] snare auto-released after %.1fs."),
				*GetName(), AutoReleaseDelay);
			ReleaseSnare();
			return;
		}
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
	// 덫 모드(지정 대상 없음): 수동/BP 발동이어도 판이 이미 눌려 있으면 점유 액터를 잡아본다
	// (판 경로로 온 호출은 HandleTriggerPlateChanged가 이미 해석해 두어 no-op).
	if (!TargetActor && !AutoTargetActor.IsValid())
	{
		ResolveAutoTargetFromPlate();
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
		// 아직 안 나간 발사 큐 폐기 — 안 걷으면 해제 *후에* 큐가 실행돼 대상 없는 결박이 마저 날아간다.
		Cable->CancelQueuedGuaranteedAimThrow();
		// Wrapped만 풀던 종전 게이트가 구멍이었다: 비행 중(GuidedThrow)/잡는 중(Contacting/Wrapping) 로프가
		// 해제 *후에* 마저 감기면 bTriggered=false라 Tick이 관리하지 않고 ③은 자동 release도 없어 영구
		// 결박이 됐다(밟자마자 이탈하는 빠른 에지에서 재현). ReleaseWrap(ReleaseWrapAs)이 네 페이즈를 전부
		// 게이트하므로 무조건 호출한다(그 외 페이즈는 내부에서 no-op).
		Cable->ReleaseWrap();
		// 다음 결박이 같은 길이에서 시작하도록 감았던 만큼 되돌린다.
		Cable->SetRopeLength(Cable->RopeLength);
	}

	// 강제 랙돌 원복(감기기 전 해제 경로): 자동 복귀는 "감았던 로프의 release 이벤트"로만 발화하므로,
	// 로프가 하나도 안 감긴 채 해제되면(밟자마자 이탈 등) 영구 랙돌이 됐다. 감긴 로프는 위 ReleaseWrap의
	// release 이벤트 → 컴포넌트 자동 복귀가 처리하고, 여기서는 "아무도 안 감은" 잔여 케이스만 명시
	// 복귀한다(다른 로프가 잡고 있으면 IfUnheld 내부 게이트가 보류).
	if (AActor* Target = GetEffectiveTargetActor())
	{
		if (URopeRagdollResponseComponent* Response = Target->FindComponentByClass<URopeRagdollResponseComponent>())
		{
			Response->RecoverFromRagdollIfUnheld();
		}
	}

	SetSnared(false);
	// 덫 모드 대상은 결박 생명주기와 같다 — 해제하면 비워 다음 눌림에서 새 대상을 획득한다.
	AutoTargetActor.Reset();
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

AActor* ARopeDemoSnare::GetEffectiveTargetActor() const
{
	return TargetActor ? TargetActor.Get() : AutoTargetActor.Get();
}

USkeletalMeshComponent* ARopeDemoSnare::ResolveTargetMesh() const
{
	AActor* Target = GetEffectiveTargetActor();
	return Target ? Target->FindComponentByClass<USkeletalMeshComponent>() : nullptr;
}

void ARopeDemoSnare::ResolveAutoTargetFromPlate()
{
	AutoTargetActor.Reset();
	if (!TriggerPlate)
	{
		return;
	}
	// 본을 감는 데모라 스켈레탈 메시 보유자만 대상이 된다(물리 상자 등은 건너뜀). 점유가 여러 개면
	// 첫 번째 해당자 — 점유 자격(태그/물리 시뮬)은 판 쪽 설정이 정한다.
	for (AActor* Occupant : TriggerPlate->GetQualifyingOccupants())
	{
		if (Occupant && Occupant != this && Occupant->FindComponentByClass<USkeletalMeshComponent>())
		{
			AutoTargetActor = Occupant;
			UE_LOG(LogDynamicRope, Log, TEXT("[%s] trap target acquired from plate: %s"),
				*GetName(), *Occupant->GetName());
			return;
		}
	}
}

void ARopeDemoSnare::ForceTargetRagdoll()
{
	AActor* Target = GetEffectiveTargetActor();
	if (!bForceRagdollOnSnare || !Target)
	{
		return;
	}
	// 사지가 순순히 벌어지려면 **감기 전에** 물리로 넘어가 있어야 한다. 응답 컴포넌트가 없으면
	// 감김 이벤트 기반 자동 전환도 없다는 뜻이라 그대로 둔다(정적 메시 대상 등).
	if (URopeRagdollResponseComponent* Response = Target->FindComponentByClass<URopeRagdollResponseComponent>())
	{
		if (!Response->IsRagdolled())
		{
			// 로프 구동 진입으로 분류(true) — 스네어가 놓으면(마지막 로프 release) 자동 복귀 대상이 된다.
			// 종전엔 수동 분류라 자동 복귀 게이트(bRagdollWasAutoTriggered)에서 걸러져 영구 랙돌이었다.
			Response->EnterRagdoll(/*bAutoRecoverOnRelease*/ true);
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
		// 자동 놓기 카운트다운은 결박 **성립** 순간부터 잰다(발사/재시도 시간은 미포함).
		AutoReleaseRemaining = AutoReleaseDelay;
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
		// 덫 모드: 지정 대상이 없으면 "판을 밟은 그 액터"를 눌림 순간 1회 확정한다.
		if (!TargetActor)
		{
			ResolveAutoTargetFromPlate();
			if (!AutoTargetActor.IsValid())
			{
				UE_LOG(LogDynamicRope, Warning,
					TEXT("[%s] plate pressed but no occupant has a skeletal mesh — snare not triggered."),
					*GetName());
				return;
			}
		}
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
