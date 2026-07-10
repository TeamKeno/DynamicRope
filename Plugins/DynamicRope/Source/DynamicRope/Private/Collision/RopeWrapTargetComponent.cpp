// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/RopeWrapTargetComponent.h"
#include "Subsystem/RopeSimSubsystem.h"
#include "Components/StaticMeshComponent.h"
#include "GameFramework/Actor.h"
#include "Engine/World.h"

URopeWrapTargetComponent::URopeWrapTargetComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void URopeWrapTargetComponent::BeginPlay()
{
	Super::BeginPlay();
	if (URopeSimSubsystem* Sim = URopeSimSubsystem::Get(GetWorld()))
	{
		Sim->RegisterColliderProvider(this);
	}
}

void URopeWrapTargetComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (URopeSimSubsystem* Sim = URopeSimSubsystem::Get(GetWorld()))
	{
		Sim->UnregisterColliderProvider(this);
	}
	Super::EndPlay(EndPlayReason);
}

USceneComponent* URopeWrapTargetComponent::ResolveTarget()
{
	if (!TargetComponent)
	{
		if (AActor* Owner = GetOwner())
		{
			// 기본: 첫 스태틱 메시(기둥 본체), 없으면 루트 컴포넌트.
			USceneComponent* Found = Owner->FindComponentByClass<UStaticMeshComponent>();
			if (!Found)
			{
				Found = Owner->GetRootComponent();
			}
			TargetComponent = Found;
		}
	}

	USceneComponent* Comp = TargetComponent;
	if (Comp && ResolvedBone.IsNone())
	{
		// 합성(가상) 본 이름: 지정값 우선, 없으면 컴포넌트 이름 기반으로 안정적 발급.
		ResolvedBone = WrapBoneName.IsNone()
			? FName(*FString::Printf(TEXT("%s_RopeWrapAnchor"), *Comp->GetName()))
			: WrapBoneName;
	}
	return Comp;
}

void URopeWrapTargetComponent::BuildCapsule(USceneComponent* Comp)
{
	const FTransform CompTM = Comp->GetComponentTransform();

	// 로컬 공간 축정렬 bounds(LocalToWorld=Identity → 로컬 반폭/중심).
	const FBoxSphereBounds LocalBounds = Comp->CalcBounds(FTransform::Identity);
	const FVector Ext = LocalBounds.BoxExtent;      // 로컬 반폭
	const FVector LocalCenter = LocalBounds.Origin; // 로컬 중심

	// 장축 인덱스 결정: 자동(최장 반폭) 또는 지정.
	int32 AxisIdx;
	if (bAutoAxis)
	{
		AxisIdx = (Ext.X >= Ext.Y && Ext.X >= Ext.Z) ? 0 : (Ext.Y >= Ext.Z) ? 1 : 2;
	}
	else
	{
		AxisIdx = (Axis == ERopeWrapAxis::X) ? 0 : (Axis == ERopeWrapAxis::Y) ? 1 : 2;
	}

	// 반지름: 명시값 또는 나머지 두 반폭의 최대(축정렬 단면을 덮는 캡슐 근사 — 사각 단면 모서리만 살짝 초과).
	double OtherMax = 0.0;
	for (int32 k = 0; k < 3; ++k)
	{
		if (k != AxisIdx)
		{
			OtherMax = FMath::Max(OtherMax, static_cast<double>(Ext[k]));
		}
	}
	const float CapRadius = (Radius > 0.0f) ? Radius : static_cast<float>(FMath::Max(OtherMax, 1.0));

	// 세그먼트 반길이 = 장축 반폭 - 반지름(반구가 끝을 넘지 않게; 음수면 0 = 구).
	const float SegHalf = static_cast<float>(FMath::Max(static_cast<double>(Ext[AxisIdx]) - CapRadius, 0.0));

	FVector LocalAxis = FVector::ZeroVector;
	LocalAxis[AxisIdx] = 1.0;
	const FVector WorldA = CompTM.TransformPosition(LocalCenter + LocalAxis * SegHalf);
	const FVector WorldB = CompTM.TransformPosition(LocalCenter - LocalAxis * SegHalf);

	// 랩 가능 캡슐: 가상 본 + SourceMesh=대상 컴포넌트. IsWorldStatic()=false(FCapsuleCollider 기본)라
	// detect 파이프라인에 포함된다 — 정적 월드 push-out 콜라이더(FRopeStaticCapsuleCollider)와 대조.
	Capsule = FCapsuleCollider(WorldA, WorldB, CapRadius, ResolvedBone, Comp);

	// 무버블 프롭 표면 속도: (현재-이전 끝점)/dt. 정적이면 사실상 0(=기존 동작). 첫 프레임은 prev 없음 → 0.
	const float FrameDt = GetWorld() ? GetWorld()->GetDeltaSeconds() : 0.0f;
	const float InvDt = (FrameDt > KINDA_SMALL_NUMBER) ? (1.0f / FrameDt) : 0.0f;
	if (bHasPrevEndpoints && InvDt > 0.0f)
	{
		Capsule.PrevA = PrevA;
		Capsule.PrevB = PrevB;
		Capsule.InvDeltaTime = InvDt;
	}
	PrevA = WorldA;
	PrevB = WorldB;
	bHasPrevEndpoints = true;
}

void URopeWrapTargetComponent::GatherColliders(TArrayView<const FBox> /*RopeRegions*/, TArray<IRopeCollider*>& OutColliders)
{
	USceneComponent* Comp = ResolveTarget();
	if (!Comp)
	{
		return;
	}

	// 프레임당 1회만 빌드(디둡): 같은 대상을 노리는 여러 로프가 호출해도 캡슐을 재구성하지 않는다.
	const uint64 Frame = GFrameCounter;
	if (BuiltFrame != Frame)
	{
		BuiltFrame = Frame;
		BuildCapsule(Comp);
	}

	OutColliders.Add(&Capsule); // 캐시된 캡슐 포인터(해당 프레임 solve 동안 유효).
}
