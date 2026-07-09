// Copyright Epic Games, Inc. All Rights Reserved.

#include "Core/RopeWrapTarget.h"
#include "Components/SceneComponent.h"
#include "Components/SkeletalMeshComponent.h"

FTransform ResolveBindingWorld(const FRopeBindingFrame& Frame)
{
	const USceneComponent* Comp = Frame.Component.Get();
	if (!Comp)
	{
		// 대상 소실(cross-actor 파괴 등). 호출자는 IsValid()로 먼저 걸러 release 하는 것을 권장.
		return FTransform::Identity;
	}

	// 스켈레탈 + 본 이름이 있으면 스키닝된 소켓 트랜스폼 — 기존 Mesh->GetSocketTransform(Bone) 과 100% 동일.
	if (const USkeletalMeshComponent* Skel = Cast<USkeletalMeshComponent>(Comp))
	{
		if (!Frame.SocketOrBone.IsNone())
		{
			return Skel->GetSocketTransform(Frame.SocketOrBone);
		}
	}

	// 정적/무버블 컴포넌트: 소켓이 지정됐으면 소켓 트랜스폼, 아니면 컴포넌트 트랜스폼(5번 경로).
	return Frame.SocketOrBone.IsNone()
		? Comp->GetComponentTransform()
		: Comp->GetSocketTransform(Frame.SocketOrBone);
}
