// Copyright Epic Games, Inc. All Rights Reserved.

#include "Logic/RopeTipPlacement.h"

#include "RopeMathHelpers.h"

namespace
{
	FVector ProjectOntoPlaneOrFallback(const FVector& Candidate, const FVector& Normal)
	{
		const FVector Axis = Normal.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
		FVector Projected = Candidate - FVector::DotProduct(Candidate, Axis) * Axis;
		if (Projected.Normalize(KINDA_SMALL_NUMBER))
		{
			return Projected;
		}
		return RopeMath::AnyTangentFromNormal(Axis);
	}

	FQuat MakeAxisAlignmentRotation(const FVector& LocalAxisInput, const FVector& WorldAxisInput,
		const FVector& LocalUpHintInput)
	{
		const FVector LocalAxis = LocalAxisInput.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
		const FVector WorldAxis = WorldAxisInput.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
		const FVector LocalUp = ProjectOntoPlaneOrFallback(LocalUpHintInput, LocalAxis);

		const FQuat ShortestSwing = FQuat::FindBetweenNormals(LocalAxis, WorldAxis);
		const FVector WorldUp = ProjectOntoPlaneOrFallback(ShortestSwing.RotateVector(LocalUp), WorldAxis);

		const FQuat LocalBasis = FRotationMatrix::MakeFromXZ(LocalAxis, LocalUp).ToQuat();
		const FQuat WorldBasis = FRotationMatrix::MakeFromXZ(WorldAxis, WorldUp).ToQuat();
		return WorldBasis * LocalBasis.Inverse();
	}
}

void FRopeTipPlacement::SolvePierceEmbed(const FVector& HitPoint, const FVector& PierceDir,
	const FTransform& HeadSocketLocal, bool bHasTailSocket, const FTransform& TailSocketLocal,
	FTransform& OutComponentWorld, FVector& OutTailWorld)
{
	const FVector Dir = PierceDir.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
	if (bHasTailSocket)
	{
		const FVector LocalTailToHeadAxis = HeadSocketLocal.GetLocation() - TailSocketLocal.GetLocation();
		if (!LocalTailToHeadAxis.IsNearlyZero())
		{
			const FQuat ComponentRot = MakeAxisAlignmentRotation(
				LocalTailToHeadAxis, Dir, HeadSocketLocal.GetUnitAxis(EAxis::Z));
			const FVector ComponentLoc = HitPoint - ComponentRot.RotateVector(HeadSocketLocal.GetLocation());
			OutComponentWorld = FTransform(ComponentRot, ComponentLoc, FVector::OneVector);
			OutTailWorld = ComponentRot.RotateVector(TailSocketLocal.GetLocation()) + ComponentLoc;
			return;
		}
	}

	const FQuat ComponentRot = MakeAxisAlignmentRotation(
		HeadSocketLocal.GetUnitAxis(EAxis::X), Dir, HeadSocketLocal.GetUnitAxis(EAxis::Z));
	const FVector ComponentLoc = HitPoint - ComponentRot.RotateVector(HeadSocketLocal.GetLocation());
	OutComponentWorld = FTransform(ComponentRot, ComponentLoc, FVector::OneVector);
	OutTailWorld = OutComponentWorld.GetLocation();
}

void FRopeTipPlacement::SolveSocketFollow(const FVector& RopeAttachWorld, const FVector& ForwardDir,
	const FTransform& TailSocketLocal, bool bHasHeadSocket, const FTransform& HeadSocketLocal,
	FTransform& OutComponentWorld)
{
	const FVector Dir = ForwardDir.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
	const FVector TailToHead = HeadSocketLocal.GetLocation() - TailSocketLocal.GetLocation();
	const bool bUseHeadAxis = bHasHeadSocket && !TailToHead.IsNearlyZero();
	const FVector LocalAxis = bUseHeadAxis ? TailToHead : TailSocketLocal.GetUnitAxis(EAxis::X);
	const FVector LocalUpHint = bUseHeadAxis
		? HeadSocketLocal.GetUnitAxis(EAxis::Z)
		: TailSocketLocal.GetUnitAxis(EAxis::Z);
	const FQuat ComponentRot = MakeAxisAlignmentRotation(LocalAxis, Dir, LocalUpHint);
	const FVector ComponentLoc = RopeAttachWorld - ComponentRot.RotateVector(TailSocketLocal.GetLocation());
	OutComponentWorld = FTransform(ComponentRot, ComponentLoc, FVector::OneVector);
}

FVector FRopeTipPlacement::MakeAimYawLockedDirection(const FVector& SourceDirInput,
	const FVector& AimDirInput, const FVector& UpHintInput)
{
	const FVector AimDir = AimDirInput.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::ForwardVector);
	const FVector SourceDir = SourceDirInput.GetSafeNormal(KINDA_SMALL_NUMBER, AimDir);
	const FVector Up = UpHintInput.GetSafeNormal(KINDA_SMALL_NUMBER, FVector::UpVector);

	FVector AimFlat = AimDir - FVector::DotProduct(AimDir, Up) * Up;
	if (!AimFlat.Normalize(KINDA_SMALL_NUMBER))
	{
		return SourceDir;
	}

	const float Vertical = FMath::Clamp(FVector::DotProduct(SourceDir, Up), -1.0f, 1.0f);
	const float HorizontalScale = FMath::Sqrt(FMath::Max(0.0f, 1.0f - FMath::Square(Vertical)));
	const FVector LockedDir = AimFlat * HorizontalScale + Up * Vertical;
	return LockedDir.GetSafeNormal(KINDA_SMALL_NUMBER, SourceDir);
}
