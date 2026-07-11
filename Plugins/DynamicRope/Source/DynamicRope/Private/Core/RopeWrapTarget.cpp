// Copyright Epic Games, Inc. All Rights Reserved.

#include "Core/RopeWrapTarget.h"
#include "Components/SceneComponent.h"
#include "Components/SkeletalMeshComponent.h"

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
