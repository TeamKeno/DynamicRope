// Copyright Epic Games, Inc. All Rights Reserved.

#include "Gameplay/RopeWrapCameraComponent.h"

#include "GameFramework/Actor.h"

URopeWrapCameraComponent::URopeWrapCameraComponent()
{
	// Never active, never ticking. The view is produced through ARopeWrapCameraRig::CalcCamera, which calls
	// GetCameraView directly, and that does not test IsActive; staying deactivated is what keeps this marker
	// from being picked by AActor::CalcCamera on a target that has a gameplay camera of its own. The
	// Activation category is hidden so the auto-activate checkbox cannot reintroduce that conflict.
	bAutoActivate = false;
	PrimaryComponentTick.bCanEverTick = false;

	BlendIn.BlendTime = 0.35f;
	BlendIn.BlendFunction = VTBlend_Cubic;
	BlendOut.BlendTime = 0.5f;
	BlendOut.BlendFunction = VTBlend_Cubic;
}

bool URopeWrapCameraComponent::MatchesBones(const TArray<FName>& WrappedBones) const
{
	if (BoneFilter.IsEmpty())
	{
		return true;
	}
	for (const FName Bone : BoneFilter)
	{
		if (WrappedBones.Contains(Bone))
		{
			return true;
		}
	}
	return false;
}

URopeWrapCameraComponent* URopeWrapCameraComponent::SelectForWrap(const AActor* TargetActor,
	const TArray<FName>& WrappedBones, const FVector& WrapLocation)
{
	if (!TargetActor)
	{
		return nullptr;
	}

	TInlineComponentArray<URopeWrapCameraComponent*> Markers;
	TargetActor->GetComponents(Markers);

	URopeWrapCameraComponent* Best = nullptr;
	bool bBestNamesBone = false;
	int32 BestPriority = 0;
	double BestDistSq = 0.0;

	for (URopeWrapCameraComponent* Marker : Markers)
	{
		if (!IsValid(Marker))
		{
			continue;
		}
		// A filter that misses this wrap rules the marker out entirely; an empty filter is the catch-all.
		const bool bNamesBone = !Marker->BoneFilter.IsEmpty();
		if (bNamesBone && !Marker->MatchesBones(WrappedBones))
		{
			continue;
		}

		const double DistSq = FVector::DistSquared(Marker->GetComponentLocation(), WrapLocation);
		bool bWins = (Best == nullptr);
		if (!bWins)
		{
			if (bNamesBone != bBestNamesBone)
			{
				// Specificity first: naming the wrapped bone outranks any priority on a catch-all.
				bWins = bNamesBone;
			}
			else if (Marker->Priority != BestPriority)
			{
				bWins = Marker->Priority > BestPriority;
			}
			else
			{
				bWins = DistSq < BestDistSq;
			}
		}

		if (bWins)
		{
			Best = Marker;
			bBestNamesBone = bNamesBone;
			BestPriority = Marker->Priority;
			BestDistSq = DistSq;
		}
	}

	return Best;
}
