// Copyright Epic Games, Inc. All Rights Reserved.

#include "Core/RopeWrapTarget.h"
#include "Components/SceneComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Collision/RopeCollider.h"

FTransform ResolveBindingWorld(const FRopeBindingFrame& Frame)
{
	// 대상 소실(cross-actor 파괴 등)이면 포인터 오버로드가 Identity를 반환한다.
	// 호출자는 IsValid()로 먼저 걸러 release 하는 것을 권장.
	return ResolveBindingWorld(Frame.Component.Get(), Frame.SocketOrBone);
}

FTransform ResolveBindingWorld(const USceneComponent* Component, FName SocketOrBone)
{
	if (!Component)
	{
		return FTransform::Identity;
	}

	// 스켈레탈 + 본 이름이 있으면 스키닝된 소켓 트랜스폼 — 기존 Mesh->GetSocketTransform(Bone) 과 100% 동일.
	if (const USkeletalMeshComponent* Skel = Cast<USkeletalMeshComponent>(Component))
	{
		if (!SocketOrBone.IsNone())
		{
			return Skel->GetSocketTransform(SocketOrBone);
		}
	}

	// 정적/무버블 컴포넌트: 지정된 이름이 실재 소켓이면 소켓 트랜스폼, 아니면(가상 본 이름 등) 컴포넌트
	// 트랜스폼(5번 정적 랩 경로). DoesSocketExist 가드로, 랩 대상이 발급한 합성(가상) 본 이름이 우연히
	// 스태틱 메시 소켓과 겹치지 않는 한 항상 컴포넌트 트랜스폼을 따르게 해 거동을 결정적으로 만든다.
	if (!SocketOrBone.IsNone() && Component->DoesSocketExist(SocketOrBone))
	{
		return Component->GetSocketTransform(SocketOrBone);
	}
	return Component->GetComponentTransform();
}

namespace RopeWrapTargets
{
	bool IsSkeletalTarget(const USceneComponent* Mesh)
	{
		return Cast<USkeletalMeshComponent>(Mesh) != nullptr;
	}

	FName GetParentTargetKey(const USceneComponent* Mesh, FName Bone)
	{
		// 스켈레탈만 본 그래프가 있다. 정적/가상 본 대상(Cast 실패) 또는 루트 본이면 None.
		const USkeletalMeshComponent* Skel = Cast<USkeletalMeshComponent>(Mesh);
		return (Skel && !Bone.IsNone()) ? Skel->GetParentBone(Bone) : NAME_None;
	}

	void AppendChildTargetKeys(const USceneComponent* Mesh, FName Bone, TArray<FName>& OutChildren)
	{
		const USkeletalMeshComponent* Skel = Cast<USkeletalMeshComponent>(Mesh);
		if (!Skel || Bone.IsNone())
		{
			return;
		}

		// 자식 열거는 전 본 스캔(스켈레톤에 자식 인덱스 테이블이 없다). 호출자(SVF 그래프 확장)는
		// depth/cost 상한이 있는 소규모 탐색이라 이 O(본 수) 스캔이 기존 구현과 동일 비용이다.
		const int32 NumBones = Skel->GetNumBones();
		for (int32 BoneIndex = 0; BoneIndex < NumBones; ++BoneIndex)
		{
			const FName BoneName = Skel->GetBoneName(BoneIndex);
			if (!BoneName.IsNone() && Skel->GetParentBone(BoneName) == Bone)
			{
				OutChildren.Add(BoneName);
			}
		}
	}

	void FilterWrappableColliders(
		const TArray<IRopeCollider*>& InColliders,
		TFunctionRef<bool(const USceneComponent*, FName)> CanWrapTarget,
		TArray<IRopeCollider*>& OutColliders)
	{
		OutColliders.Reset(InColliders.Num());
		for (IRopeCollider* Collider : InColliders)
		{
			if (!Collider)
			{
				continue;
			}

			FName Bone = NAME_None;
			const USceneComponent* Mesh = nullptr;
			Collider->GetGPUAttribution(Bone, Mesh);

			// 귀속 없음 = 감김 대상이 아니라 표면 기하일 뿐 → 게이트 대상에서 제외(항상 유지).
			const bool bAttributed = !Bone.IsNone() || Mesh != nullptr;
			if (bAttributed && !CanWrapTarget(Mesh, Bone))
			{
				continue;
			}

			OutColliders.Add(Collider);
		}
	}
}
