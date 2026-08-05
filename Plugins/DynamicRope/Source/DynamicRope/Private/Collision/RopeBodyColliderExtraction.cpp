// Copyright 2026 TeamKeno. All Rights Reserved.

#include "Collision/RopeBodyColliderExtraction.h"
#include "PhysicsEngine/BodySetup.h"
// FKConvexElem::GetPlanes, for extracting world planes.
#include "PhysicsEngine/ConvexElem.h"

namespace
{
	// The element-local to world matrix. Unreal's convex convention, as in FKConvexElem::CalcAABB, is the element
	// transform composed with the scale and then with the unscaled component transform. The scale is separated from
	// the rotation and translation and composed as a matrix, since FTransform cannot represent shear, so the planes
	// transform exactly even under a non-uniform scale combined with a rotation.
	FMatrix ComposeConvexToWorld(const FTransform& ElemTM, const FVector& Scale3D, const FTransform& CompTM)
	{
		FTransform CompNoScale = CompTM;
		CompNoScale.SetScale3D(FVector::OneVector);
		return ElemTM.ToMatrixWithScale() * FScaleMatrix(Scale3D) * CompNoScale.ToMatrixWithScale();
	}

	// The body-local transform, meaning the element and the scale but excluding the component's rigid rotation and
	// translation, so that world space is the body-local space composed with that rigid transform. Only the rigid
	// part moves between frames, assuming the scale is constant, so the body-local planes are invariant, which is
	// what allows a dynamic body's substep pose to be interpolated rigidly.
	FMatrix ComposeConvexBodyLocal(const FTransform& ElemTM, const FVector& Scale3D)
	{
		return ElemTM.ToMatrixWithScale() * FScaleMatrix(Scale3D);
	}

	// Transforms a set of local planes into world space and normalizes them to unit outward normals.
	// FPlane::TransformBy transforms the normal correctly through the inverse transpose and is therefore exact even
	// under shear, but the length changes, so the plane is divided by the normal's length.
	void TransformPlanesToWorld(const TArray<FPlane>& Local, const FMatrix& M, TArray<FPlane>& OutWorld)
	{
		OutWorld.Reset(Local.Num());
		for (const FPlane& LP : Local)
		{
			FPlane WP = LP.TransformBy(M);
			const double NLen = FMath::Sqrt(WP.X * WP.X + WP.Y * WP.Y + WP.Z * WP.Z);
			if (NLen > UE_SMALL_NUMBER)
			{
				WP.X /= NLen; WP.Y /= NLen; WP.Z /= NLen; WP.W /= NLen;
				OutWorld.Add(WP);
			}
		}
	}

	// The six planes, with outward axis normals, of a local box centred on the origin with the given half extents. Used to route a sheared box to a convex exactly.
	TArray<FPlane> MakeBoxLocalPlanes(const FVector& Half)
	{
		TArray<FPlane> P;
		P.Reserve(6);
		P.Add(FPlane(FVector(1, 0, 0), Half.X));
		P.Add(FPlane(FVector(-1, 0, 0), Half.X));
		P.Add(FPlane(FVector(0, 1, 0), Half.Y));
		P.Add(FPlane(FVector(0, -1, 0), Half.Y));
		P.Add(FPlane(FVector(0, 0, 1), Half.Z));
		P.Add(FPlane(FVector(0, 0, -1), Half.Z));
		return P;
	}
}

namespace RopeBodyColliderExtraction
{
	bool AppendBodyColliders(const UBodySetup& Setup, const FTransform& CompTM, const FTransform& PrevCompTM,
		float InvDeltaTime, int32 MaxColliders, int32 MaxConvexPlanes,
		TArray<FRopeBoxCollider>& OutBoxes, TArray<FRopeStaticCapsuleCollider>& OutCapsules,
		TArray<FRopeConvexCollider>& OutConvexes, const TFunctionRef<void(int32)>& OnConvexFallback,
		FName AttributionBone, const USceneComponent* AttributionMesh)
	{
		const FVector Scale3D = CompTM.GetScale3D();
		const auto BudgetLeft = [&]() { return OutBoxes.Num() + OutCapsules.Num() + OutConvexes.Num() < MaxColliders; };

		// A sphyl uses the same scale convention as the skinned capsule provider, GetScaledRadius and
		// CylinderLength. A dynamic one also fills in the previous endpoints so it rides the existing surface
		// velocity and substep CCD machinery of FCapsuleCollider with no shader or GPU change.
		for (const FKSphylElem& Sphyl : Setup.AggGeom.SphylElems)
		{
			if (!BudgetLeft())
			{
				return false;
			}
			const FTransform ElemTM = Sphyl.GetTransform() * CompTM;
			// A sphyl's axis is its local Z.
			const FVector Axis = ElemTM.GetUnitAxis(EAxis::Z);
			const FVector Center = ElemTM.GetLocation();
			const float HalfLen = Sphyl.GetScaledCylinderLength(Scale3D) * 0.5f;
			FRopeStaticCapsuleCollider Cap(Center + Axis * HalfLen, Center - Axis * HalfLen, Sphyl.GetScaledRadius(Scale3D));
			if (InvDeltaTime > 0.0f)
			{
				const FTransform PrevElemTM = Sphyl.GetTransform() * PrevCompTM;
				const FVector PrevAxis = PrevElemTM.GetUnitAxis(EAxis::Z);
				const FVector PrevCenter = PrevElemTM.GetLocation();
				Cap.PrevA = PrevCenter + PrevAxis * HalfLen;
				Cap.PrevB = PrevCenter - PrevAxis * HalfLen;
				Cap.InvDeltaTime = InvDeltaTime;
			}
			Cap.Bone = AttributionBone;
			Cap.SourceMesh = AttributionMesh;
			OutCapsules.Add(MoveTemp(Cap));
		}

		// A sphere is a degenerate capsule whose endpoints coincide.
		for (const FKSphereElem& Sphere : Setup.AggGeom.SphereElems)
		{
			if (!BudgetLeft())
			{
				return false;
			}
			const FVector Center = CompTM.TransformPosition(Sphere.Center);
			const float ScaledRadius = Sphere.Radius * static_cast<float>(Scale3D.GetAbsMin());
			FRopeStaticCapsuleCollider Cap(Center, Center, ScaledRadius);
			if (InvDeltaTime > 0.0f)
			{
				const FVector PrevCenter = PrevCompTM.TransformPosition(Sphere.Center);
				Cap.PrevA = PrevCenter;
				Cap.PrevB = PrevCenter;
				Cap.InvDeltaTime = InvDeltaTime;
			}
			Cap.Bone = AttributionBone;
			Cap.SourceMesh = AttributionMesh;
			OutCapsules.Add(MoveTemp(Cap));
		}

		// A box is an analytic OBB, which is the substance of handling corners exactly. The X, Y and Z are full
		// lengths. Only a shear, meaning an element with a non-uniform scale combined with a rotation, is routed to a
		// six-plane convex to stay exact, since an OBB cannot represent it. Everything else is an exact OBB.
		for (const FKBoxElem& Box : Setup.AggGeom.BoxElems)
		{
			if (!BudgetLeft())
			{
				return false;
			}
			// The half extents in element space, before the scale.
			const FVector HalfLocal(Box.X * 0.5, Box.Y * 0.5, Box.Z * 0.5);
			const FVector AbsScale = Scale3D.GetAbs();
			const bool bUniform = FMath::IsNearlyEqual(AbsScale.GetMax(), AbsScale.GetMin(), UE_KINDA_SMALL_NUMBER);
			if (bUniform || Box.Rotation.IsNearlyZero())
			{
			// An exact OBB, when the scale is uniform across all axes or the element rotation is the identity, meaning it is aligned to the component axes so the per-axis scale is exact.
				const FTransform ElemTM = Box.GetTransform() * CompTM;
				const FVector Half = bUniform ? HalfLocal * AbsScale.X : HalfLocal * AbsScale;
				FRopeBoxCollider BoxCol(ElemTM.GetLocation(), ElemTM.GetRotation(), Half);
				if (InvDeltaTime > 0.0f)
				{
					const FTransform PrevElemTM = Box.GetTransform() * PrevCompTM;
					BoxCol.PrevCenter = PrevElemTM.GetLocation();
					BoxCol.PrevRot = PrevElemTM.GetRotation();
					BoxCol.InvDeltaTime = InvDeltaTime;
				}
				BoxCol.Bone = AttributionBone;
				BoxCol.SourceMesh = AttributionMesh;
				OutBoxes.Add(MoveTemp(BoxCol));
			}
			else
			{
				// Shear, handled exactly as a six-plane convex, since planes transform exactly even under a shear
				// matrix. It is stored as body-local planes plus the component's rigid transform, so a moving sheared
				// box is supported too.
				TArray<FPlane> LocalPlanes;
				const FMatrix BodyLocalM = ComposeConvexBodyLocal(Box.GetTransform(), Scale3D);
				TransformPlanesToWorld(MakeBoxLocalPlanes(HalfLocal), BodyLocalM, LocalPlanes);
				const FBox LB = FBox(-HalfLocal, HalfLocal).TransformBy(BodyLocalM);
				if (LocalPlanes.Num() == 6 && LB.IsValid)
				{
					FRopeConvexCollider Cv(MoveTemp(LocalPlanes), LB, CompTM.GetRotation(), CompTM.GetTranslation());
					if (InvDeltaTime > 0.0f)
					{
						Cv.PrevRot = PrevCompTM.GetRotation();
						Cv.PrevTrans = PrevCompTM.GetTranslation();
						Cv.InvDeltaTime = InvDeltaTime;
					}
					Cv.Bone = AttributionBone;
					Cv.SourceMesh = AttributionMesh;
					OutConvexes.Add(MoveTemp(Cv));
				}
			}
		}

		// A convex transforms the Chaos convex's plane set into world space as an analytic convex collider. When it
		// is uncooked, meaning the plane set is empty, or has more planes than the limit, it falls back to the
		// element box as an OBB approximation rather than losing the collision entirely.
		for (const FKConvexElem& Convex : Setup.AggGeom.ConvexElems)
		{
			if (!BudgetLeft())
			{
				return false;
			}
			TArray<FPlane> ElemPlanes;
			Convex.GetPlanes(ElemPlanes);
			const FMatrix BodyLocalM = ComposeConvexBodyLocal(Convex.GetTransform(), Scale3D);

			if (ElemPlanes.Num() >= 4 && ElemPlanes.Num() <= MaxConvexPlanes && Convex.ElemBox.IsValid)
			{
				TArray<FPlane> LocalPlanes;
				// Element space to body-local space, excluding the rigid transform.
				TransformPlanesToWorld(ElemPlanes, BodyLocalM, LocalPlanes);
				const FBox LB = Convex.ElemBox.TransformBy(BodyLocalM);
				if (LocalPlanes.Num() >= 4 && LB.IsValid)
				{
					FRopeConvexCollider Cv(MoveTemp(LocalPlanes), LB, CompTM.GetRotation(), CompTM.GetTranslation());
					if (InvDeltaTime > 0.0f)
					{
						Cv.PrevRot = PrevCompTM.GetRotation();
						Cv.PrevTrans = PrevCompTM.GetTranslation();
						Cv.InvDeltaTime = InvDeltaTime;
					}
					Cv.Bone = AttributionBone;
					Cv.SourceMesh = AttributionMesh;
					OutConvexes.Add(MoveTemp(Cv));
					continue;
				}
			}

			// The fallback, an element box OBB approximation, taken when the convex is uncooked, has too many planes
			// or is invalid. It is coarse, but keeping a collision beats losing one. It is treated as static, this being a rare path.
			if (Convex.ElemBox.IsValid)
			{
				const FMatrix M = ComposeConvexToWorld(Convex.GetTransform(), Scale3D, CompTM);
				const FVector CenterW = M.TransformPosition(Convex.ElemBox.GetCenter());
				const FQuat   RotW = M.GetMatrixWithoutScale().ToQuat();
				const FVector HalfW = Convex.ElemBox.GetExtent() * static_cast<float>(Scale3D.GetAbsMin());
				FRopeBoxCollider FallbackBox(CenterW, RotW, HalfW);
			// In attributed mode the fallback OBB is attributed as wrappable exactly like any other element.
				FallbackBox.Bone = AttributionBone;
				FallbackBox.SourceMesh = AttributionMesh;
				OutBoxes.Add(MoveTemp(FallbackBox));
				OnConvexFallback(ElemPlanes.Num());
			}
		}

		return true;
	}
}
