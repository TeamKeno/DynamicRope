// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/RopeBoneCapsuleProvider.h"
#include "DynamicRopeLog.h"
#include "Components/SkeletalMeshComponent.h"
#include "Engine/SkinnedAsset.h"
#include "GameFramework/Actor.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "PhysicsEngine/SkeletalBodySetup.h"
#include "ReferenceSkeleton.h"

void URopeBoneCapsuleProvider::RebuildColliders(USkeletalMeshComponent* Mesh, float InvDt)
{
	Capsules.Reset();
	BuildCapsules(Mesh);

	// The previous frame's endpoints are matched up by index, so that the difference between the current and previous
	// endpoints over the delta gives each capsule's surface velocity. On a count mismatch, meaning the first frame or
	// a configuration change, this frame is treated as static, with an inverse delta of zero giving zero velocity,
	// leaving FCapsuleCollider with its previous endpoints already initialized to the current ones.
	if (PrevEndpoints.Num() == Capsules.Num())
	{
		for (int32 i = 0; i < Capsules.Num(); ++i)
		{
			Capsules[i].PrevA = PrevEndpoints[i].Key;
			Capsules[i].PrevB = PrevEndpoints[i].Value;
			Capsules[i].InvDeltaTime = InvDt;
		}
	}

	// Stores the current endpoints for the next frame.
	PrevEndpoints.Reset(Capsules.Num());
	for (const FCapsuleCollider& Cap : Capsules)
	{
		PrevEndpoints.Emplace(Cap.A, Cap.B);
	}
}

void URopeBoneCapsuleProvider::AppendColliderPointers(FRopeColliderGatherContext& Gather)
{
	Gather.Colliders.Reserve(Gather.Colliders.Num() + Capsules.Num());
	for (FCapsuleCollider& Cap : Capsules)
	{
		Gather.Colliders.Add(&Cap);
	}
}

void URopeBoneCapsuleProvider::BuildCapsules(USkeletalMeshComponent* Mesh)
{
	// 1) The explicit list: one capsule per listed bone, spanning the bone to its parent, with the configured radius. This is the existing behaviour.
	if (Bones.Num() > 0)
	{
		for (const FName& Bone : Bones)
		{
			if (Bone.IsNone())
			{
				continue;
			}
			const FName    Parent = Mesh->GetParentBone(Bone);
			const FVector  P0 = Mesh->GetSocketTransform(Bone).GetLocation();
			const FVector  P1 = Parent.IsNone() ? P0 : Mesh->GetSocketTransform(Parent).GetLocation();
			Capsules.Add(FCapsuleCollider(P0, P1, CapsuleRadius, Bone, Mesh));
		}
		UE_LOG(LogRopeCollision, VeryVerbose, TEXT("CapsuleProvider on %s: built %d capsule(s) from %d listed bone(s)."),
			*GetNameSafe(GetOwner()), Capsules.Num(), Bones.Num());
		return;
	}

	// 2) Automatic, from a physics asset: the body shapes are used as capsules. A sphyl is used as it is, a sphere as a
	// degenerate capsule whose endpoints coincide, and a box as a capsule approximation aligned to its long axis, with
	// the axis being the longest side and the radius the larger of the other two half extents, which over-covers the
	// cross-section's corners slightly. This gives the real dimensions per bone and naturally excludes the phantom
	// segments of unskinned IK and twist bones.
	// Every other shape, such as a convex, is skipped, which is why producing no shapes at all falls through to the
	// skeleton fallback below, so that a physics asset made entirely of convexes does not lose its collision entirely.
	if (const UPhysicsAsset* PhysAsset = Mesh->GetPhysicsAsset())
	{
		for (const TObjectPtr<USkeletalBodySetup>& Setup : PhysAsset->SkeletalBodySetups)
		{
			if (!Setup)
			{
				continue;
			}
			const FName BoneName = Setup->BoneName;
			const int32 BoneIndex = Mesh->GetBoneIndex(BoneName);
			if (BoneIndex == INDEX_NONE)
			{
				// A bone present in the asset but absent from the current mesh.
				continue;
			}
			const FTransform BoneTM = Mesh->GetBoneTransform(BoneIndex);
			const FVector Scale3D = BoneTM.GetScale3D();

			for (const FKSphylElem& Sphyl : Setup->AggGeom.SphylElems)
			{
				const FTransform ElemTM = Sphyl.GetTransform() * BoneTM;
				// A sphyl's axis is its local Z.
				const FVector Axis = ElemTM.GetUnitAxis(EAxis::Z);
				const FVector Center = ElemTM.GetLocation();
				const float HalfLen = Sphyl.GetScaledCylinderLength(Scale3D) * 0.5f;
				Capsules.Add(FCapsuleCollider(Center + Axis * HalfLen, Center - Axis * HalfLen,
					Sphyl.GetScaledRadius(Scale3D), BoneName, Mesh));
			}
			for (const FKSphereElem& Sphere : Setup->AggGeom.SphereElems)
			{
				const FVector Center = BoneTM.TransformPosition(Sphere.Center);
				const float ScaledRadius = Sphere.Radius * static_cast<float>(Scale3D.GetAbsMin());
				Capsules.Add(FCapsuleCollider(Center, Center, ScaledRadius, BoneName, Mesh));
			}
			for (const FKBoxElem& Box : Setup->AggGeom.BoxElems)
			{
				// The X, Y and Z are full lengths. The longest side becomes the capsule axis and the larger of the
				// other two half extents, being the median of the three, becomes the radius, which covers the long
				// side of the cross-section rectangle and over-covers only its corners. The segment's half length is
				// the longest half extent minus the radius, so the hemispheres do not extend past the ends of the
				// box, clamped to zero, which gives a sphere.
				const double UniformScale = Scale3D.GetAbsMin();
				const double Hx = Box.X * 0.5 * UniformScale;
				const double Hy = Box.Y * 0.5 * UniformScale;
				const double Hz = Box.Z * 0.5 * UniformScale;
				const double LongHalf = FMath::Max3(Hx, Hy, Hz);
				const double MidHalf = Hx + Hy + Hz - LongHalf - FMath::Min3(Hx, Hy, Hz);
				const EAxis::Type LongAxis = (Hx >= Hy && Hx >= Hz) ? EAxis::X : (Hy >= Hz) ? EAxis::Y : EAxis::Z;
				const float SegHalf = static_cast<float>(FMath::Max(LongHalf - MidHalf, 0.0));
				const FTransform ElemTM = Box.GetTransform() * BoneTM;
				const FVector Axis = ElemTM.GetUnitAxis(LongAxis);
				const FVector Center = ElemTM.GetLocation();
				Capsules.Add(FCapsuleCollider(Center + Axis * SegHalf, Center - Axis * SegHalf,
					static_cast<float>(MidHalf), BoneName, Mesh));
			}
		}
		if (Capsules.Num() > 0)
		{
			UE_LOG(LogRopeCollision, VeryVerbose, TEXT("CapsuleProvider on %s: built %d capsule(s) from physics asset %s."),
				*GetNameSafe(GetOwner()), Capsules.Num(), *GetNameSafe(PhysAsset));
			return;
		}
		UE_LOG(LogRopeCollision, Verbose, TEXT("CapsuleProvider on %s: physics asset %s has no usable shapes — falling back to skeleton."),
			*GetNameSafe(GetOwner()), *GetNameSafe(PhysAsset));
	}

	// 3) Automatic, from the skeleton, as a fallback when there is no physics asset: every bone-to-parent segment,
	// with the configured radius. Segments shorter than AutoMinBoneLength are excluded, which cuts out finger and
	// twist noise. Phantom segments from unskinned bones such as IK bones can be included, so this is for rough
	// testing and a character is better given a physics asset.
	const USkinnedAsset* Asset = Mesh->GetSkinnedAsset();
	if (!Asset)
	{
		return;
	}
	const FReferenceSkeleton& RefSkel = Asset->GetRefSkeleton();
	const int32 NumBones = Mesh->GetNumBones();
	const float MinLenSq = FMath::Square(FMath::Max(AutoMinBoneLength, 0.0f));
	for (int32 i = 0; i < NumBones; ++i)
	{
		const int32 ParentIndex = RefSkel.GetParentIndex(i);
		if (ParentIndex == INDEX_NONE)
		{
			// The root, which has no parent segment.
			continue;
		}
		const FVector P0 = Mesh->GetBoneTransform(i).GetLocation();
		const FVector P1 = Mesh->GetBoneTransform(ParentIndex).GetLocation();
		if (FVector::DistSquared(P0, P1) < MinLenSq)
		{
			continue;
		}
		Capsules.Add(FCapsuleCollider(P0, P1, CapsuleRadius, Mesh->GetBoneName(i), Mesh));
	}
	UE_LOG(LogRopeCollision, VeryVerbose, TEXT("CapsuleProvider on %s: built %d capsule(s) from skeleton fallback (%d bone(s), no physics asset)."),
		*GetNameSafe(GetOwner()), Capsules.Num(), NumBones);
}
