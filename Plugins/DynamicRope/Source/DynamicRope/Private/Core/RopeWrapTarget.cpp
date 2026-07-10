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

	// 정적/무버블 컴포넌트: 지정된 이름이 실재 소켓이면 소켓 트랜스폼, 아니면(가상 본 이름 등) 컴포넌트
	// 트랜스폼(5번 정적 랩 경로). DoesSocketExist 가드로, 랩 대상이 발급한 합성(가상) 본 이름이 우연히
	// 스태틱 메시 소켓과 겹치지 않는 한 항상 컴포넌트 트랜스폼을 따르게 해 거동을 결정적으로 만든다.
	if (!Frame.SocketOrBone.IsNone() && Comp->DoesSocketExist(Frame.SocketOrBone))
	{
		return Comp->GetSocketTransform(Frame.SocketOrBone);
	}
	return Comp->GetComponentTransform();
}
