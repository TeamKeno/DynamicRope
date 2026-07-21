// Fill out your copyright notice in the Description page of Project Settings.

#include "RopeDragonFlightDemoComponent.h"

#include "Animation/AnimSequenceBase.h"
#include "Components/SkeletalMeshComponent.h"
#include "Core/RopeTypes.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Subsystem/RopeSimSubsystem.h"
#include "UObject/UObjectIterator.h"

// 데모 전용 로그 카테고리 — 플러그인의 LogDynamicRope는 export되지 않아 게임 모듈에서 못 쓴다.
DEFINE_LOG_CATEGORY_STATIC(LogRopeDragonDemo, Log, All);

namespace RopeDragonDemo
{
	/** HowlAnim이 비어 있을 때 쓰는 하울링 길이(초). */
	static constexpr float FallbackHowlDuration = 1.5f;

	/** 0→1 구간을 부드럽게(가감속) 만든다 — 상승/하강의 팝 방지. */
	static float Ease(float Alpha)
	{
		const float T = FMath::Clamp(Alpha, 0.0f, 1.0f);
		return T * T * (3.0f - 2.0f * T);
	}
}

URopeDragonFlightDemoComponent::URopeDragonFlightDemoComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void URopeDragonFlightDemoComponent::BeginPlay()
{
	Super::BeginPlay();

	if (const AActor* Owner = GetOwner())
	{
		AnchorLocation = Owner->GetActorLocation();
		AnchorYawDeg = static_cast<float>(Owner->GetActorRotation().Yaw);
	}

	if (!ResolveMesh())
	{
		UE_LOG(LogRopeDragonDemo, Warning, TEXT("[%s] DragonDemo: 스켈레탈 메시를 찾지 못했다 — 애니메이션 전환 없이 이동만 한다."),
			*GetNameSafe(GetOwner()));
	}

	PlayAnim(IdleAnim, /*bLoop*/ true);

	// 자기를 감을 로프를 미리 몰라도 되도록 월드 중앙 신호를 구독한다(랙돌 반응 컴포넌트와 같은 패턴).
	if (URopeSimSubsystem* Sim = URopeSimSubsystem::Get(GetWorld()))
	{
		WrappedHandle = Sim->OnAnyRopeWrapped.AddUObject(this, &URopeDragonFlightDemoComponent::HandleAnyRopeWrapped);
	}
}

void URopeDragonFlightDemoComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (URopeSimSubsystem* Sim = URopeSimSubsystem::Get(GetWorld()))
	{
		Sim->OnAnyRopeWrapped.Remove(WrappedHandle);
	}
	Super::EndPlay(EndPlayReason);
}

void URopeDragonFlightDemoComponent::HandleAnyRopeWrapped(const FRopeWrappedEventInfo& Info)
{
	const USkeletalMeshComponent* MyMesh = ResolveMesh();
	if (!bStartOnWrapped || !MyMesh || Info.Mesh.Get() != MyMesh)
	{
		return;
	}
	if (State != ERopeDragonDemoState::Idle || PendingTriggerTime >= 0.0f)
	{
		// 이미 발동했거나 대기 중 — 두 번째 로프가 걸려도 연출을 다시 시작하지 않는다.
		return;
	}

	UE_LOG(LogRopeDragonDemo, Log, TEXT("[%s] DragonDemo: 로프 감김(본 %s) → %.2f초 뒤 하울링 시작."),
		*GetNameSafe(GetOwner()), *Info.Bone.ToString(), TriggerDelay);
	PendingTriggerTime = FMath::Max(TriggerDelay, 0.0f);
}

void URopeDragonFlightDemoComponent::StartSequence()
{
	if (State != ERopeDragonDemoState::Idle)
	{
		return;
	}

	// 발동 시점 트랜스폼이 연출 전체의 원점/기저다. Yaw만 쓰므로 8자는 항상 지면과 수평이다.
	if (const AActor* Owner = GetOwner())
	{
		AnchorLocation = Owner->GetActorLocation();
		AnchorYawDeg = static_cast<float>(Owner->GetActorRotation().Yaw);
	}
	PendingTriggerTime = -1.0f;
	AscendEndOffset = FVector::ZeroVector;

	EnterState(ERopeDragonDemoState::Howl);
}

void URopeDragonFlightDemoComponent::ResetSequence()
{
	PendingTriggerTime = -1.0f;
	AscendEndOffset = FVector::ZeroVector;
	State = ERopeDragonDemoState::Idle;
	TimeInState = 0.0f;
	ApplyPose(FVector::ZeroVector, 0.0f, 0.0f, 0.0f);
	PlayAnim(IdleAnim, /*bLoop*/ true);
	UE_LOG(LogRopeDragonDemo, Log, TEXT("[%s] DragonDemo: 리셋 — 발동 지점으로 복귀."), *GetNameSafe(GetOwner()));
}

void URopeDragonFlightDemoComponent::EnterState(ERopeDragonDemoState NewState)
{
	State = NewState;
	TimeInState = 0.0f;

	switch (NewState)
	{
	case ERopeDragonDemoState::Howl:
		PlayAnim(HowlAnim, /*bLoop*/ false);
		break;
	case ERopeDragonDemoState::Thrash:
		PlayAnim(FlyAnim, /*bLoop*/ true);
		break;
	case ERopeDragonDemoState::Finished:
		PlayAnim(IdleAnim, /*bLoop*/ true);
		break;
	default:
		// Ascend/Descend는 Thrash에서 켠 비행 루프를 그대로 이어 쓴다(재생 위치 리셋 방지).
		break;
	}

	UE_LOG(LogRopeDragonDemo, Log, TEXT("[%s] DragonDemo: 단계 → %s"),
		*GetNameSafe(GetOwner()), *UEnum::GetValueAsString(NewState));
}

void URopeDragonFlightDemoComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!GetOwner() || DeltaTime <= 0.0f)
	{
		return;
	}

	if (PendingTriggerTime >= 0.0f)
	{
		PendingTriggerTime -= DeltaTime;
		if (PendingTriggerTime <= 0.0f)
		{
			StartSequence();
		}
		return;
	}

	if (State == ERopeDragonDemoState::Idle || State == ERopeDragonDemoState::Finished)
	{
		return;
	}

	TimeInState += DeltaTime;

	switch (State)
	{
	case ERopeDragonDemoState::Howl:
	{
		// 제자리에서 하울링만 — 위치는 발동 지점 그대로 고정한다.
		ApplyPose(FVector::ZeroVector, 0.0f, 0.0f, 0.0f);
		if (TimeInState >= ResolveHowlDuration())
		{
			EnterState(ERopeDragonDemoState::Thrash);
		}
		break;
	}

	case ERopeDragonDemoState::Thrash:
	{
		// 8자 = 리사주 1:2 곡선 (sin u, sin 2u). 긴 축 = 발동 시점 전방, 교차점 = 발동 지점.
		// ThrashLoops가 정수라 u가 2π의 배수로 끝나며, 시작/끝이 모두 원점이라 앞뒤 단계와 매끄럽게 이어진다.
		const float Alpha = FMath::Clamp(TimeInState / FMath::Max(ThrashDuration, KINDA_SMALL_NUMBER), 0.0f, 1.0f);
		const float U = Alpha * 2.0f * PI * FMath::Max(ThrashLoops, 1);

		const float SinU = FMath::Sin(U);
		const float CosU = FMath::Cos(U);
		const float Sin2U = FMath::Sin(2.0f * U);
		const float Cos2U = FMath::Cos(2.0f * U);

		FVector Offset;
		Offset.X = ThrashLength * SinU;						// 전방 축
		Offset.Y = ThrashWidth * Sin2U;						// 좌우 축(2배 주파수 = 8자)
		Offset.Z = ThrashBobHeight * (0.5f - 0.5f * Cos2U);	// 0에서 시작/끝나는 상하 흔들림

		// 진행 방향 = 궤적의 접선. 몸이 항상 가는 쪽을 본다.
		const float TangentX = ThrashLength * CosU;
		const float TangentY = 2.0f * ThrashWidth * Cos2U;
		const float LocalYaw = FMath::RadiansToDegrees(FMath::Atan2(TangentY, TangentX));

		// 뱅크: 교차 구간에서 최대로 눕는다(선회 방향과 부호가 맞는다 — sin2U가 우선회 구간에서 양수).
		const float Roll = ThrashBankDeg * Sin2U;

		ApplyPose(Offset, LocalYaw, 0.0f, Roll);

		if (Alpha >= 1.0f)
		{
			EnterState(ERopeDragonDemoState::Ascend);
		}
		break;
	}

	case ERopeDragonDemoState::Ascend:
	{
		const float Alpha = FMath::Clamp(TimeInState / FMath::Max(AscendDuration, KINDA_SMALL_NUMBER), 0.0f, 1.0f);
		const float E = RopeDragonDemo::Ease(Alpha);

		const FVector Offset(AscendForward * E, 0.0f, AscendHeight * E);
		// 피치는 가운데서 최대, 끝에서 0 — 다음 단계(하강)와 이어질 때 각도가 튀지 않는다.
		const float Pitch = ClimbPitchDeg * FMath::Sin(PI * Alpha);
		ApplyPose(Offset, 0.0f, Pitch, 0.0f);

		if (Alpha >= 1.0f)
		{
			AscendEndOffset = Offset;
			EnterState(ERopeDragonDemoState::Descend);
		}
		break;
	}

	case ERopeDragonDemoState::Descend:
	{
		const float Alpha = FMath::Clamp(TimeInState / FMath::Max(DescendDuration, KINDA_SMALL_NUMBER), 0.0f, 1.0f);
		const float E = RopeDragonDemo::Ease(Alpha);

		const FVector LandOffset = bReturnToStartOnLand
			? FVector::ZeroVector
			: FVector(AscendEndOffset.X, AscendEndOffset.Y, 0.0f);
		const FVector Offset = FMath::Lerp(AscendEndOffset, LandOffset, E);
		const float Pitch = -ClimbPitchDeg * FMath::Sin(PI * Alpha);
		ApplyPose(Offset, 0.0f, Pitch, 0.0f);

		if (Alpha >= 1.0f)
		{
			EnterState(ERopeDragonDemoState::Finished);
		}
		break;
	}

	default:
		break;
	}
}

void URopeDragonFlightDemoComponent::ApplyPose(const FVector& LocalOffset, float YawDeg, float PitchDeg, float RollDeg)
{
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}

	const FRotator AnchorBasis(0.0f, AnchorYawDeg, 0.0f);
	const FVector World = AnchorLocation + AnchorBasis.RotateVector(LocalOffset);
	const FRotator Rotation(PitchDeg, AnchorYawDeg + YawDeg, RollDeg);

	// 연출 전용 — 경로를 그대로 덮어쓴다(스윕 없음, 테더/충돌 변위는 남지 않는다).
	Owner->SetActorLocationAndRotation(World, Rotation, /*bSweep*/ false);
}

float URopeDragonFlightDemoComponent::ResolveHowlDuration() const
{
	if (HowlDuration > 0.0f)
	{
		return HowlDuration;
	}
	if (HowlAnim && HowlAnim->GetPlayLength() > 0.0f)
	{
		return HowlAnim->GetPlayLength();
	}
	return RopeDragonDemo::FallbackHowlDuration;
}

void URopeDragonFlightDemoComponent::PlayAnim(UAnimSequenceBase* Anim, bool bLoop)
{
	USkeletalMeshComponent* Mesh = ResolveMesh();
	if (!Mesh || !Anim)
	{
		return;
	}
	// AnimBP 없이 싱글 노드 모드로 직접 재생 — PlayAnimation이 모드 전환까지 겸한다.
	Mesh->PlayAnimation(Anim, bLoop);
}

USkeletalMeshComponent* URopeDragonFlightDemoComponent::ResolveMesh() const
{
	const AActor* Owner = GetOwner();
	return Owner ? Owner->FindComponentByClass<USkeletalMeshComponent>() : nullptr;
}

#if !UE_BUILD_SHIPPING
//======================================================================================
// 촬영용 콘솔 명령 — 월드 내 이 컴포넌트가 붙은 액터 전부에 적용.
//======================================================================================

namespace RopeDragonDemoConsole
{
	static void ForEach(UWorld* World, TFunctionRef<void(URopeDragonFlightDemoComponent&)> Fn)
	{
		int32 Count = 0;
		for (TObjectIterator<URopeDragonFlightDemoComponent> It; It; ++It)
		{
			URopeDragonFlightDemoComponent* Comp = *It;
			if (IsValid(Comp) && Comp->GetWorld() == World && IsValid(Comp->GetOwner()))
			{
				Fn(*Comp);
				++Count;
			}
		}
		if (Count == 0)
		{
			UE_LOG(LogRopeDragonDemo, Warning,
				TEXT("URopeDragonFlightDemoComponent가 붙은 액터가 없다 — 드래곤 BP에 컴포넌트를 추가할 것."));
		}
	}

	static FAutoConsoleCommandWithWorldAndArgs GStartCmd(
		TEXT("Rope.Demo.Dragon"),
		TEXT("드래곤 데모 연출 시작(하울링 → 8자 몸부림 → 상승 → 하강)."),
		FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
		{
			ForEach(World, [](URopeDragonFlightDemoComponent& Comp)
			{
				Comp.StartSequence();
			});
		}));

	static FAutoConsoleCommandWithWorldAndArgs GResetCmd(
		TEXT("Rope.Demo.Dragon.Reset"),
		TEXT("드래곤 데모 연출 리셋(발동 지점/Idle로 복귀 — 반복 촬영용)."),
		FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
		{
			ForEach(World, [](URopeDragonFlightDemoComponent& Comp)
			{
				Comp.ResetSequence();
			});
		}));
}
#endif // !UE_BUILD_SHIPPING
