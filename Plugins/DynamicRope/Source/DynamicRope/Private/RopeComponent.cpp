// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeComponent.h"
#include "Collision/RopeCollider.h"
#include "Collision/RopeColliderProvider.h"
#include "Render/RopeSceneProxy.h"
#include "Debug/RopeDebugDraw.h"
#include "Components/SkeletalMeshComponent.h"
#include "GameFramework/Actor.h"

URopeComponent::URopeComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;

	// primitive가 motion vector를 출력하도록 Movable로 설정한다(TAA/TSR가 움직이는 rope를 유지하게 한다).
	Mobility = EComponentMobility::Movable;
}

void URopeComponent::InitRope()
{
	const int32 N = FMath::Max(2, NumParticles);
	Sim.Positions.SetNum(N);
	Sim.PrevPositions.SetNum(N);
	Sim.InvMass.SetNum(N);
	Sim.RopeLength = RopeLength;
	Sim.SegmentLength = RopeLength / static_cast<float>(N - 1);

	const FVector Start = GetComponentLocation();
	const FVector End = Start + GetForwardVector() * RopeLength;
	for (int32 i = 0; i < N; ++i)
	{
		const float Alpha = static_cast<float>(i) / static_cast<float>(N - 1);
		Sim.Positions[i] = FMath::Lerp(Start, End, Alpha);
		Sim.PrevPositions[i] = Sim.Positions[i];
		Sim.InvMass[i] = 1.0f;
	}

	// 시작점을 컴포넌트(hand/socket)에 pin한다; solver가 substep에 걸쳐 이를 sweep한다.
	Sim.InvMass[0] = 0.0f;
	Sim.bStartPinned = true;
	Sim.StartPinTarget = Start;
	Sim.StartPinPrev = Start;
}

void URopeComponent::GatherFrameColliders(TArray<IRopeCollider*>& OutColliders) const
{
	OutColliders.Reset();
	FBox RopeBounds(ForceInit);
	for (const FVector& P : Sim.Positions)
	{
		RopeBounds += P;
	}
	// contact reach만큼 확장한다: 거의 직선인 rope를 감싼 tight box는 두께가 ~0이라, 실제로는 contact
	// 거리 안에 있는 capsule을 잘못 cull해 버린다. narrow phase와 맞춘다
	// (node ContactRadius; capsule 자신의 반지름은 이미 GetWorldBounds()에 포함되어 있다).
	if (RopeBounds.IsValid)
	{
		RopeBounds = RopeBounds.ExpandBy(Radius + WrapConfig.ContactRadius + 5.0f);
	}

	RopeDebug::DrawBounds(GetWorld(), RopeBounds, bDrawDebugCenterline);

	for (const TScriptInterface<IRopeColliderProvider>& Provider : ColliderProviders)
	{
		if (IRopeColliderProvider* Raw = Provider.GetInterface())
		{
			Raw->GatherColliders(RopeBounds, OutColliders);
		}
	}

	RopeDebug::DrawStats(GetWorld(), reinterpret_cast<uint64>(this), Phase,
		ColliderProviders.Num(), OutColliders.Num(), WrapController.State.BoneName, bDrawDebugCenterline);
}

void URopeComponent::EnsureColliderProviders()
{
	if (ColliderProviders.Num() > 0)
	{
		return;
	}

	auto AddFrom = [this](AActor* Actor)
	{
		if (!Actor)
		{
			return;
		}
		for (UActorComponent* Comp : Actor->GetComponentsByInterface(URopeColliderProvider::StaticClass()))
		{
			ColliderProviders.AddUnique(TScriptInterface<IRopeColliderProvider>(Comp));
		}
	};

	// Cross-actor: rope가 한 actor에 고정되어 있지만 *다른* body를 잡아야 할 때, provider는 그 다른
	// actor 위에 존재한다. 명시적 리스트가 설정되어 있으면 그것을 사용하고, 그렇지 않으면 우리
	// 자신의 owner로 기본 설정한다(same-actor 경우).
	if (ColliderSourceActors.Num() > 0)
	{
		for (AActor* Actor : ColliderSourceActors)
		{
			AddFrom(Actor);
		}
	}
	else
	{
		AddFrom(GetOwner());
	}

	// 명시적으로 지정된 wrap-target mesh의 owner만 따라간다; 여기서는 절대 auto-resolve 하지 않는다(그러면
	// 우리 자신의 owner를 다시 추가하게 되어 rope가 자기 자신에게 latch하도록 만든다).
	if (WrapTargetMesh)
	{
		AddFrom(WrapTargetMesh->GetOwner());
	}
}

USkeletalMeshComponent* URopeComponent::ResolveWrapTargetMesh()
{
	if (!WrapTargetMesh)
	{
		if (AActor* Owner = GetOwner())
		{
			WrapTargetMesh = Owner->FindComponentByClass<USkeletalMeshComponent>();
		}
	}
	return WrapTargetMesh;
}

void URopeComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (Sim.Num() == 0)
	{
		InitRope();
	}

	// pinned-start target을 전진시킨다; solver가 substep에 걸쳐 Prev->Target을 sweep하므로 빠른
	// 캐릭터 이동이 chain을 홱 잡아당겨(폭주시켜) 버리지 않는다.
	if (Sim.bStartPinned)
	{
		Sim.StartPinPrev = Sim.StartPinTarget;
		Sim.StartPinTarget = GetComponentLocation();
	}

	EnsureColliderProviders();

	switch (Phase)
	{
	case ERopePhase::Free:        // 손에서 늘어뜨려진 채 캐릭터를 따라간다
	case ERopePhase::Flight:
	case ERopePhase::Contacting:
	{
		TArray<IRopeCollider*> Colliders;
		GatherFrameColliders(Colliders);
		Solver.Step(Sim, SolverConfig, Colliders, DeltaTime);

		// Contact decision(physics → logic gate). throw 이후에만 동작한다; 자유롭게 늘어진 rope가
		// body를 스치는 것만으로 latch해서는 안 된다.
		if (Phase != ERopePhase::Free)
		{
			FRopeWrapState Seed;
			if (WrapController.DecideWrap(Sim, Colliders, WrapConfig, DeltaTime, Seed))
			{
				WrapController.BeginWrap(Sim, Seed, ResolveWrapTargetMesh());
				Phase = ERopePhase::Wrapped;
				OnRopeWrapped.Broadcast(WrapController.State.BoneName);
			}
		}
		break;
	}
	case ERopePhase::Wrapped:
	{
		// logic이 latch된 node들을 소유한다(skinned bone에 올라탄다); solver는 여전히 free span을
		// settle하여 rope가 늘어지고 node 0에서 손에 붙어 있도록 유지한다.
		WrapController.Hold(Sim, ResolveWrapTargetMesh(), DeltaTime);
		TArray<IRopeCollider*> Colliders;
		GatherFrameColliders(Colliders);
		Solver.Step(Sim, SolverConfig, Colliders, DeltaTime);
		break;
	}
	case ERopePhase::Releasing:
		// 모든 node를 solver에 다시 넘긴다(hand pin만 유지), 그런 다음 free simulation을 재개한다.
		for (int32 i = 0; i < Sim.Num(); ++i)
		{
			Sim.InvMass[i] = (i == 0 && Sim.bStartPinned) ? 0.0f : 1.0f;
		}
		Phase = ERopePhase::Free;
		break;
	default:
		break;
	}

	// 새 centerline을 render proxy로 push하고 bounds를 갱신한다.
	MarkRenderDynamicDataDirty();
	MarkRenderTransformDirty();

	RopeDebug::DrawCenterline(GetWorld(), Sim, Phase, WrapController.State, bDrawDebugCenterline);
}

void URopeComponent::SendRenderDynamicData_Concurrent()
{
	Super::SendRenderDynamicData_Concurrent();

	if (!SceneProxy || Sim.Num() < 2)
	{
		return;
	}

	// centerline을 component-local 공간으로 보낸다; proxy는 GetLocalToWorld()를 통해 렌더링한다.
	const FTransform Xform = GetComponentTransform();
	FRopeDynamicData* DynamicData = new FRopeDynamicData;
	DynamicData->Points.SetNumUninitialized(Sim.Num());
	for (int32 i = 0; i < Sim.Num(); ++i)
	{
		DynamicData->Points[i] = Xform.InverseTransformPosition(Sim.Positions[i]);
	}

	FRopeSceneProxy* Proxy = static_cast<FRopeSceneProxy*>(SceneProxy);
	ENQUEUE_RENDER_COMMAND(RopeUpdateCenterline)(
		[Proxy, DynamicData](FRHICommandListBase& RHICmdList)
		{
			Proxy->SetDynamicData_RenderThread(RHICmdList, DynamicData);
		});
}

void URopeComponent::Throw(const FVector& AimDir)
{
	if (Sim.Num() == 0)
	{
		InitRope();
	}

	Phase = ERopePhase::Flight;

	// Scaffold launch: Verlet prev-position offset을 통해 free tip에 초기 속도를 부여한다.
	const int32 Last = Sim.Num() - 1;
	if (Last > 0)
	{
		Sim.InvMass[Last] = 1.0f;
		const FVector Velocity = AimDir.GetSafeNormal() * ThrowParams.ThrowSpeed * (1.0f / 60.0f);
		Sim.PrevPositions[Last] = Sim.Positions[Last] - Velocity;
	}
}

bool URopeComponent::DebugForceWrap()
{
	if (Sim.Num() == 0)
	{
		InitRope();
	}

	EnsureColliderProviders();
	TArray<IRopeCollider*> Colliders;
	GatherFrameColliders(Colliders);

	// 단일 접촉 node가 이번 frame에 commit되도록 decision gate를 완화한다(DecideWrap은 candidate의
	// 누적 시간이 >= WrapDecisionTime일 때 commit한다; 0은 "지금 즉시"를 의미한다).
	FRopeWrapConfig Relaxed = WrapConfig;
	Relaxed.WrapDecisionTime = 0.0f;
	Relaxed.MinLatchNodes = 1;

	FRopeWrapState Seed;
	if (!WrapController.DecideWrap(Sim, Colliders, Relaxed, 0.0f, Seed))
	{
		return false; // 접촉 중인 것이 없다; 먼저 rope/capsule을 움직여 겹치게 한다
	}

	WrapController.BeginWrap(Sim, Seed, ResolveWrapTargetMesh());
	Phase = ERopePhase::Wrapped;
	OnRopeWrapped.Broadcast(WrapController.State.BoneName);
	return true;
}

void URopeComponent::ReleaseWrap()
{
	const FName Bone = WrapController.State.BoneName;
	WrapController.Release(ERopeReleaseReason::Manual);
	Phase = ERopePhase::Releasing;
	OnRopeReleased.Broadcast(Bone, ERopeReleaseReason::Manual);
}

FPrimitiveSceneProxy* URopeComponent::CreateSceneProxy()
{
	return new FRopeSceneProxy(this);
}

int32 URopeComponent::GetNumMaterials() const
{
	return 1;
}

UMaterialInterface* URopeComponent::GetMaterial(int32 /*ElementIndex*/) const
{
	return RopeMaterial;
}

void URopeComponent::SetMaterial(int32 /*ElementIndex*/, UMaterialInterface* Material)
{
	RopeMaterial = Material;
	MarkRenderStateDirty();
}

FBoxSphereBounds URopeComponent::CalcBounds(const FTransform& LocalToWorld) const
{
	// bounds를 컴포넌트(pinned start)에 anchor하되, rope가 어떻게 변형되든 항상 rope를 포함하는 반지름을
	// 사용한다: chain은 inextensible하므로 어떤 particle도 pin으로부터 RopeLength(+ tube radius)보다 멀리
	// 떨어지지 않는다. 대신 per-frame sim point로부터 bounds를 도출하면 render thread보다 한 frame 뒤처지며;
	// 빠른 캐릭터 모션 중에는 rope가 그 tight box를 앞질러 shadow/main pass에서 cull된다 -> 움직이는 동안
	// shadow가 사라지고 VSM cache는 오래된 afterimage를 유지한다. component transform에 anchor하면 엔진의
	// 추적되는 transform을 통해 bounds가 캐릭터와 함께 움직이므로, lag도 없고 잘못된 culling도 없다.
	const float Reach = RopeLength + Radius + 1.0f;
	return FBoxSphereBounds(LocalToWorld.GetLocation(), FVector(Reach), Reach);
}
