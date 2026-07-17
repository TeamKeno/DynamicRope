// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/RopeSkeletalColliderProvider.h"
#include "DynamicRopeLog.h"
#include "Subsystem/RopeSimSubsystem.h"
#include "Components/SkeletalMeshComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"

namespace
{
	// 로프 region(브로드페이즈 질의 bounds) 중 하나라도 메시의 현재 월드 bounds와 겹치는지. 어느 로프도
	// 근처에 없으면 전 본 collider 재빌드를 건너뛴다(#12). 보수적: bounds가 불명이면 true(빌드)로 본다.
	// region은 접촉/예측 마진을, 메시 bounds는 스킨 범위를 이미 포함하므로 겹치면 접촉 가능 = 빌드.
	bool AnyRopeRegionNearMesh(const USkeletalMeshComponent& Mesh, TArrayView<const FBox> Regions)
	{
		const FBox MeshBox = Mesh.Bounds.GetBox();
		if (!MeshBox.IsValid)
		{
			return true;
		}
		for (const FBox& Region : Regions)
		{
			if (Region.IsValid && Region.Intersect(MeshBox))
			{
				return true;
			}
		}
		return false;
	}
}

URopeSkeletalColliderProvider::URopeSkeletalColliderProvider()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void URopeSkeletalColliderProvider::BeginPlay()
{
	Super::BeginPlay();
	if (URopeSimSubsystem* Sim = URopeSimSubsystem::Get(GetWorld()))
	{
		Sim->RegisterColliderProvider(this);
	}
}

void URopeSkeletalColliderProvider::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (URopeSimSubsystem* Sim = URopeSimSubsystem::Get(GetWorld()))
	{
		Sim->UnregisterColliderProvider(this);
	}
	Super::EndPlay(EndPlayReason);
}

USkeletalMeshComponent* URopeSkeletalColliderProvider::ResolveMesh()
{
	if (!SkeletalMesh)
	{
		if (AActor* Owner = GetOwner())
		{
			SkeletalMesh = Owner->FindComponentByClass<USkeletalMeshComponent>();
		}
	}
	return SkeletalMesh;
}

void URopeSkeletalColliderProvider::GatherColliders(FRopeColliderGatherContext& Gather)
{
	USkeletalMeshComponent* Mesh = ResolveMesh();
	if (!Mesh)
	{
		UE_LOG(LogRopeCollision, Verbose, TEXT("%s on %s: no skeletal mesh resolved — no colliders."),
			*GetClass()->GetName(), *GetNameSafe(GetOwner()));
		return;
	}
	if (!HasColliderData())
	{
		// 데이터 없음(예: SDFData 미지정) — 서브클래스가 사유를 로그하고 no-op.
		return;
	}

	// 프레임당 1회만 빌드(디둡): 같은 메시를 잡는 여러 로프가 호출해도 collider를 재구성하지 않는다.
	// region별 배정은 아래 MapCollidersToRegionsByBounds가 만든다(빌드는 region 무관 — 전 본 빌드).
	const uint64 Frame = GFrameCounter;
	if (BuiltFrame != Frame)
	{
		BuiltFrame = Frame;
		// 근접 게이트(#12): 어느 로프 region도 이 메시 근처에 없으면 전 본 collider 재빌드를 건너뛴다. 본은 매
		// 프레임 애니로 움직여 캐시가 무의미하므로, 근접 로프가 없으면 아예 만들지 않는다(빌드/append 스킵).
		bBuiltThisFrame = AnyRopeRegionNearMesh(*Mesh, Gather.RopeRegions);
		if (bBuiltThisFrame)
		{
			// 표면 속도(드래그) 산출용 프레임 dt. 서브클래스가 (현재-이전)/dt 로 collider의 표면 속도를 만든다.
			const float FrameDt = GetWorld() ? GetWorld()->GetDeltaSeconds() : 0.0f;
			const float InvDt = (FrameDt > KINDA_SMALL_NUMBER) ? (1.0f / FrameDt) : 0.0f;
			RebuildColliders(Mesh, InvDt);
		}
	}
	if (!bBuiltThisFrame)
	{
		// 이 프레임 근접 로프 없음 → collider 공급 없음(스테일 append 방지).
		return;
	}

	// 캐시된 collider 포인터를 넘긴다(해당 프레임 동안 유효). region 매핑: 메시(collider 유니언) 선-거절 →
	// 걸린 로프만 collider별 bounds 배정. 원거리 로프는 메시당 비교 1회로 끝난다 — 서브시스템의 로프별
	// 풀 전체 재-컬(O(로프×풀))을 대체하는 부분.
	const int32 StartIndex = Gather.Colliders.Num();
	AppendColliderPointers(Gather);
	RopeColliderGather::MapCollidersToRegionsByBounds(Gather, StartIndex);
}
