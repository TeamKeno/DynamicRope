// Copyright Epic Games, Inc. All Rights Reserved.
//
// PoC — experimental, not shipping. See Docs/PoC/01_PostWrapModel.md.

#include "PoC/RopePoCSkeletalColliderComponent.h"

#include "Components/SkeletalMeshComponent.h"
#include "Engine/SkeletalMesh.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "PhysicsEngine/SkeletalBodySetup.h"
#include "PhysicsEngine/AggregateGeom.h"
#include "DrawDebugHelpers.h"

URopePoCSkeletalColliderComponent::URopePoCSkeletalColliderComponent()
{
	PrimaryComponentTick.bCanEverTick = false; // pulled by the rope, no tick needed
}

USkeletalMeshComponent* URopePoCSkeletalColliderComponent::ResolveMesh() const
{
	if (TargetMesh)
	{
		return TargetMesh;
	}
	if (const AActor* Owner = GetOwner())
	{
		return Owner->FindComponentByClass<USkeletalMeshComponent>();
	}
	return nullptr;
}

void URopePoCSkeletalColliderComponent::GatherRopeCapsules(TArray<FRopeCapsule>& OutCapsules) const
{
	USkeletalMeshComponent* Mesh = ResolveMesh();
	if (!Mesh)
	{
		return;
	}

	const int32 FirstNew = OutCapsules.Num();

	if (bUsePhysicsAsset && Mesh->GetPhysicsAsset())
	{
		GatherFromPhysicsAsset(Mesh, OutCapsules);
	}
	else
	{
		GatherManual(Mesh, OutCapsules);
	}

	if (bDrawDebug)
	{
		DrawCapsules(OutCapsules, FirstNew);
	}
}

void URopePoCSkeletalColliderComponent::GatherFromPhysicsAsset(USkeletalMeshComponent* Mesh, TArray<FRopeCapsule>& OutCapsules) const
{
	const UPhysicsAsset* PhysicsAsset = Mesh->GetPhysicsAsset();
	if (!PhysicsAsset)
	{
		return;
	}

	for (const TObjectPtr<USkeletalBodySetup>& BodyPtr : PhysicsAsset->SkeletalBodySetups)
	{
		const USkeletalBodySetup* Body = BodyPtr;
		if (!Body)
		{
			continue;
		}

		const FName BoneName = Body->BoneName;
		if (Bones.Num() > 0 && !Bones.Contains(BoneName))
		{
			continue;
		}

		const int32 BoneIndex = Mesh->GetBoneIndex(BoneName);
		if (BoneIndex == INDEX_NONE)
		{
			continue;
		}

		const FTransform BoneTM = Mesh->GetBoneTransform(BoneIndex);
		const FKAggregateGeom& Agg = Body->AggGeom;

		for (const FKSphylElem& Sphyl : Agg.SphylElems)
		{
			const FTransform ElemTM = Sphyl.GetTransform() * BoneTM;
			const float Scale = ElemTM.GetScale3D().GetAbsMax();
			const FVector Up = ElemTM.GetUnitAxis(EAxis::Z);
			const float HalfLen = (Sphyl.Length * 0.5f) * Scale;

			FRopeCapsule Cap;
			Cap.A = ElemTM.GetLocation() - Up * HalfLen;
			Cap.B = ElemTM.GetLocation() + Up * HalfLen;
			Cap.Radius = Sphyl.Radius * Scale;
			OutCapsules.Add(Cap);
		}

		for (const FKSphereElem& Sphere : Agg.SphereElems)
		{
			const FTransform ElemTM = Sphere.GetTransform() * BoneTM;
			const float Scale = ElemTM.GetScale3D().GetAbsMax();

			FRopeCapsule Cap;
			Cap.A = ElemTM.GetLocation();
			Cap.B = Cap.A;
			Cap.Radius = Sphere.Radius * Scale;
			OutCapsules.Add(Cap);
		}
		// Boxes / convex bodies are ignored for the PoC — limbs are sphyls/spheres.
	}
}

void URopePoCSkeletalColliderComponent::GatherManual(USkeletalMeshComponent* Mesh, TArray<FRopeCapsule>& OutCapsules) const
{
	const USkeletalMesh* SkelMesh = Mesh->GetSkeletalMeshAsset();
	if (!SkelMesh)
	{
		return;
	}

	const FReferenceSkeleton& Ref = SkelMesh->GetRefSkeleton();

	for (const FName& BoneName : Bones)
	{
		const int32 BoneIndex = Mesh->GetBoneIndex(BoneName);
		if (BoneIndex == INDEX_NONE)
		{
			continue;
		}

		// Capsule from this bone to its first child, so it spans the limb segment.
		const int32 RefBoneIndex = Ref.FindBoneIndex(BoneName);
		int32 ChildRefIndex = INDEX_NONE;
		for (int32 i = 0; i < Ref.GetNum(); ++i)
		{
			if (Ref.GetParentIndex(i) == RefBoneIndex)
			{
				ChildRefIndex = i;
				break;
			}
		}

		const FVector P0 = Mesh->GetBoneTransform(BoneIndex).GetLocation();
		FVector P1 = P0;
		if (ChildRefIndex != INDEX_NONE)
		{
			const int32 ChildBoneIndex = Mesh->GetBoneIndex(Ref.GetBoneName(ChildRefIndex));
			if (ChildBoneIndex != INDEX_NONE)
			{
				P1 = Mesh->GetBoneTransform(ChildBoneIndex).GetLocation();
			}
		}

		FRopeCapsule Cap;
		Cap.A = P0;
		Cap.B = P1;
		Cap.Radius = ManualRadius;
		OutCapsules.Add(Cap);
	}
}

void URopePoCSkeletalColliderComponent::DrawCapsules(const TArray<FRopeCapsule>& Capsules, int32 FirstNew) const
{
	const UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	for (int32 i = FirstNew; i < Capsules.Num(); ++i)
	{
		const FRopeCapsule& Cap = Capsules[i];
		const FVector Center = (Cap.A + Cap.B) * 0.5f;
		const FVector Axis = Cap.B - Cap.A;
		const float HalfHeight = Axis.Size() * 0.5f + Cap.Radius;
		const FQuat Rot = Axis.IsNearlyZero()
			? FQuat::Identity
			: FRotationMatrix::MakeFromZ(Axis.GetSafeNormal()).ToQuat();

		DrawDebugCapsule(World, Center, HalfHeight, Cap.Radius, Rot, FColor::Green, false, -1.0f, SDPG_World, 0.5f);
	}
}
