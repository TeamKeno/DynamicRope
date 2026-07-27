// Copyright Epic Games, Inc. All Rights Reserved.

#include "Core/RopeWrapTarget.h"
#include "Components/SceneComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Collision/RopeCollider.h"

FTransform ResolveBindingWorld(const FRopeBindingFrame& Frame)
{
	// If the target is lost, as when a cross-actor target is destroyed, the pointer overload returns the identity.
	// Callers are advised to filter with IsValid() first and release.
	return ResolveBindingWorld(Frame.Component.Get(), Frame.SocketOrBone);
}

FTransform ResolveBindingWorld(const USceneComponent* Component, FName SocketOrBone)
{
	if (!Component)
	{
		return FTransform::Identity;
	}

	// With a skeletal mesh and a bone name it is the skinned socket transform, exactly as Mesh->GetSocketTransform(Bone) would give.
	if (const USkeletalMeshComponent* Skel = Cast<USkeletalMeshComponent>(Component))
	{
		if (!SocketOrBone.IsNone())
		{
			return Skel->GetSocketTransform(SocketOrBone);
		}
	}

	// A static or movable component: the socket transform if the given name is a real socket, and otherwise, as with a
	// virtual bone name, the component transform, which is the static wrap path. The DoesSocketExist guard makes the
	// behaviour deterministic by always following the component transform unless a synthetic virtual bone name issued
	// by a wrap target happens to collide with a static mesh socket.
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
	// Only a skeletal mesh has a bone graph, so a static or virtual bone target, where the cast fails, or the root bone gives none.
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

	// Enumerating children is a scan of every bone, since a skeleton carries no child index table. The caller, being
	// the surface vector field's graph expansion, is a small search bounded by depth and cost, so this scan over the
	// bone count costs the same as the previous implementation.
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

			// With no attribution it is not a wrap target but merely surface geometry, so it is excluded from the gate and always kept.
			const bool bAttributed = !Bone.IsNone() || Mesh != nullptr;
			if (bAttributed && !CanWrapTarget(Mesh, Bone))
			{
				continue;
			}

			OutColliders.Add(Collider);
		}
	}
}
