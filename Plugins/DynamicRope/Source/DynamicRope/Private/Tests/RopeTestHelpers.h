// Copyright Epic Games, Inc. All Rights Reserved.
//
// Helpers for the solver and logic unit tests: a POD FRopeSimState fixture builder and a scripted IRopeCollider mock.
// They have no UObject dependency, which is what allows pure unit tests with no world, and is why the solver is POD.

#pragma once

#include "CoreMinimal.h"
#include "Components/SkeletalMeshComponent.h"
#include "Core/RopeSimTypes.h"
#include "Collision/RopeCollider.h"

namespace RopeTest
{
	/** A straight chain fixture. Inverse mass one, zero velocity with the previous positions equal to the current ones, and by default no pins, giving a free chain. */
	inline FRopeSimState MakeStraightRope(int32 NumNodes, float Length,
		const FVector& Start = FVector::ZeroVector, const FVector& Dir = FVector(1, 0, 0))
	{
		FRopeSimState S;
		const int32 N = FMath::Max(2, NumNodes);
		S.Positions.SetNum(N);
		S.PrevPositions.SetNum(N);
		S.InvMass.SetNum(N);
		S.RopeLength = Length;
		S.SegmentLength = Length / static_cast<float>(N - 1);
		const FVector D = Dir.GetSafeNormal();
		for (int32 i = 0; i < N; ++i)
		{
			const float Alpha = static_cast<float>(i) / static_cast<float>(N - 1);
			const FVector P = Start + D * (Length * Alpha);
			S.Positions[i] = P;
			S.PrevPositions[i] = P;
			S.InvMass[i] = 1.0f;
		}
		return S;
	}

	/** The maximum segment length error, being the absolute difference between the length and the segment length. */
	inline float MaxSegmentError(const FRopeSimState& S)
	{
		float MaxErr = 0.0f;
		for (int32 i = 0; i + 1 < S.Num(); ++i)
		{
			const float Len = static_cast<float>(FVector::Dist(S.Positions[i], S.Positions[i + 1]));
			MaxErr = FMath::Max(MaxErr, FMath::Abs(Len - S.SegmentLength));
		}
		return MaxErr;
	}

	inline bool AnyNaN(const FRopeSimState& S)
	{
		for (const FVector& P : S.Positions)
		{
			if (P.ContainsNaN())
			{
				return true;
			}
		}
		return false;
	}

	/**
	 * A scripted sphere collider. It reports a contact with an outward normal for any query within the radius of its
	 * centre.
	 * As the FRopeContact contract requires, it reports a source mesh too, since a skeletal collider has to fill in
	 * the mesh that owns the bone and the wrap pipeline decides the mesh to wrap from that value. A test can pass an
	 * empty USkeletalMeshComponent created with NewObject; it is stored for identity alone and is dereferenced only at
	 * the moment of latching.
	 */
	class FSphereMockCollider : public IRopeCollider
	{
	public:
		FVector Center = FVector::ZeroVector;
		float   Radius = 0.0f;
		FName   Bone = NAME_None;
		const USkeletalMeshComponent* SourceMesh = nullptr;
		// The contact point's surface velocity, in centimetres per second. It is contractually an additive field,
		// defaulting to zero for a static surface, so it is compatible with the existing tests.
		// Used by the tests covering a pose pop spike on a ragdoll transition frame, meaning the friction clamp and the relative motion evaluation.
		FVector SurfaceVelocity = FVector::ZeroVector;

		FSphereMockCollider() = default;
		FSphereMockCollider(const FVector& InCenter, float InRadius, FName InBone,
			const USkeletalMeshComponent* InSourceMesh = nullptr)
			: Center(InCenter), Radius(InRadius), Bone(InBone), SourceMesh(InSourceMesh) {}

		virtual FRopeContact Query(const FVector& WorldPos, float NodeRadius) const override
		{
			FRopeContact C;
			const FVector D = WorldPos - Center;
			const float Dist = static_cast<float>(D.Size());
			const float Pen = (Radius + NodeRadius) - Dist;
			if (Pen > 0.0f)
			{
				C.bHit = true;
				C.Penetration = Pen;
				C.Normal = (Dist > KINDA_SMALL_NUMBER) ? (D / Dist) : FVector::UpVector;
				C.SurfacePoint = Center + C.Normal * Radius;
				C.Bone = Bone;
				C.SourceMesh = SourceMesh;
				C.SurfaceVelocity = SurfaceVelocity;
			}
			return C;
		}

		virtual FBox GetWorldBounds() const override
		{
			return FBox(Center - FVector(Radius), Center + FVector(Radius));
		}

		virtual void GetGPUAttribution(FName& OutBone, const USceneComponent*& OutMesh) const override
		{
			OutBone = Bone;
			OutMesh = SourceMesh;
		}
	};
}
