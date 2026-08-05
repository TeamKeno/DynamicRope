// Copyright 2026 TeamKeno. All Rights Reserved.

#include "Collision/RopeWrapTargetComponent.h"
#include "Subsystem/RopeSimSubsystem.h"
// LogRopeCollision, the diagnostic log category.
#include "DynamicRopeLog.h"
#include "Components/StaticMeshComponent.h"
// GetBodySetup for the simple collision extraction, plus UBodySetup and FKAggregateGeom for the sphyl,
// box and sphere elements.
#include "Components/PrimitiveComponent.h"
#include "PhysicsEngine/BodySetup.h"
#include "GameFramework/Actor.h"
#include "Engine/World.h"
// Extracting the whole simple collision into push-out colliders, through the helper shared with the
// static body provider.
#include "Collision/RopeBodyColliderExtraction.h"
// UDynamicRopeSettings, for the budget, the convex plane limit and the channel decision behind
// bIncludeWorldDynamic.
#include "Settings/DynamicRopeSettings.h"

URopeWrapTargetComponent::URopeWrapTargetComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void URopeWrapTargetComponent::BeginPlay()
{
	Super::BeginPlay();
	URopeSimSubsystem* Sim = URopeSimSubsystem::Get(GetWorld());
	if (Sim)
	{
		Sim->RegisterColliderProvider(this);
	}

	// Diagnostics: log once whether it registered and whether the target component and virtual bone
	// resolved.
	USceneComponent* Comp = ResolveTarget();
	UE_LOG(LogRopeCollision, Log,
		TEXT("[WrapTarget] BeginPlay actor=%s: subsystem=%s, target=%s, virtualBone=%s"),
		*GetNameSafe(GetOwner()),
		Sim ? TEXT("OK") : TEXT("NULL (registration failed)"),
		*GetNameSafe(Comp),
		*ResolvedBone.ToString());
}

void URopeWrapTargetComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (URopeSimSubsystem* Sim = URopeSimSubsystem::Get(GetWorld()))
	{
		Sim->UnregisterColliderProvider(this);
	}
	Super::EndPlay(EndPlayReason);
}

USceneComponent* URopeWrapTargetComponent::ResolveTarget()
{
	if (!TargetComponent)
	{
		if (AActor* Owner = GetOwner())
		{
			// The default is the first static mesh, which is the pillar body itself, falling back to the
			// root component.
			USceneComponent* Found = Owner->FindComponentByClass<UStaticMeshComponent>();
			if (!Found)
			{
				Found = Owner->GetRootComponent();
			}
			TargetComponent = Found;
		}
	}

	USceneComponent* Comp = TargetComponent;
	if (Comp && ResolvedBone.IsNone())
	{
		// The synthetic virtual bone name: the assigned value takes priority, and otherwise a stable one is
		// issued from the component's name.
		ResolvedBone = WrapBoneName.IsNone()
			? FName(*FString::Printf(TEXT("%s_RopeWrapAnchor"), *Comp->GetName()))
			: WrapBoneName;
	}
	return Comp;
}

void URopeWrapTargetComponent::BuildCapsule(USceneComponent* Comp)
{
	FVector WorldA = FVector::ZeroVector;
	FVector WorldB = FVector::ZeroVector;
	float   CapRadius = 0.0f;

	// The capsule is taken from the target's authored simple collision first, whether a sphyl, box or
	// sphere, because it is tight to the visual mesh and leaves no gap between the rope and the surface.
	// With no simple collision, or only convexes, it falls back to approximating the local bounds.
	bUsedSimpleCollision = BuildCapsuleFromSimpleCollision(Comp, WorldA, WorldB, CapRadius);
	if (!bUsedSimpleCollision)
	{
		BuildCapsuleFromBounds(Comp, WorldA, WorldB, CapRadius);
	}

	// The designer's radius override changes the thickness alone, keeping the shape and the axis endpoints.
	if (Radius > 0.0f)
	{
		CapRadius = Radius;
	}

	// A wrappable capsule: a virtual bone plus a source mesh naming the target component. IsWorldStatic()
	// is false, which is the capsule collider's default, so it takes part in the detection pipeline,
	// unlike the static world push-out capsule.
	Capsule = FCapsuleCollider(WorldA, WorldB, CapRadius, ResolvedBone, Comp);

	// The surface velocity of a movable prop, from the change in endpoints over the frame delta. A static
	// target gives effectively zero, and the first frame has no previous state and gives zero.
	const float FrameDt = GetWorld() ? GetWorld()->GetDeltaSeconds() : 0.0f;
	const float InvDt = (FrameDt > KINDA_SMALL_NUMBER) ? (1.0f / FrameDt) : 0.0f;
	if (bHasPrevEndpoints && InvDt > 0.0f)
	{
		Capsule.PrevA = PrevA;
		Capsule.PrevB = PrevB;
		Capsule.InvDeltaTime = InvDt;
	}
	PrevA = WorldA;
	PrevB = WorldB;
	bHasPrevEndpoints = true;
}

bool URopeWrapTargetComponent::BuildCapsuleFromSimpleCollision(USceneComponent* Comp,
	FVector& OutA, FVector& OutB, float& OutRadius) const
{
	UPrimitiveComponent* Prim = Cast<UPrimitiveComponent>(Comp);
	UBodySetup* Setup = Prim ? Prim->GetBodySetup() : nullptr;
	if (!Setup)
	{
		return false;
	}

	const FKAggregateGeom& Agg = Setup->AggGeom;
	const FTransform CompTM = Comp->GetComponentTransform();
	const FVector Scale = CompTM.GetScale3D();

	// The priority is sphyl, then box, then sphere, and within one type the largest, which is the main
	// body. With only convexes it returns a failure and the caller falls back to the bounds. The scale
	// convention matches the bone capsule and the static provider.

	// First, sphyls, taking the longest; a cylindrical pillar matches its radius exactly here and the gap
	// disappears.
	{
		const FKSphylElem* Best = nullptr;
		float BestSize = -1.0f;
		for (const FKSphylElem& Sphyl : Agg.SphylElems)
		{
			const float Size = Sphyl.GetScaledCylinderLength(Scale) + 2.0f * Sphyl.GetScaledRadius(Scale);
			if (Size > BestSize)
			{
				BestSize = Size;
				Best = &Sphyl;
			}
		}
		if (Best)
		{
			const FTransform ElemTM = Best->GetTransform() * CompTM;
			// A sphyl's axis is its local Z.
			const FVector AxisDir = ElemTM.GetUnitAxis(EAxis::Z);
			const FVector Center = ElemTM.GetLocation();
			const float HalfLen = Best->GetScaledCylinderLength(Scale) * 0.5f;
			OutA = Center + AxisDir * HalfLen;
			OutB = Center - AxisDir * HalfLen;
			OutRadius = Best->GetScaledRadius(Scale);
			return true;
		}
	}

	// Second, a box converted to a capsule, aligned to its longest axis with a radius of the larger of the
	// other two half extents, which is the same approximation the bone capsule provider uses.
	{
		const FKBoxElem* Best = nullptr;
		double BestSize = -1.0;
		for (const FKBoxElem& BoxElem : Agg.BoxElems)
		{
			const double Size = FMath::Max3(static_cast<double>(BoxElem.X), static_cast<double>(BoxElem.Y), static_cast<double>(BoxElem.Z));
			if (Size > BestSize)
			{
				BestSize = Size;
				Best = &BoxElem;
			}
		}
		if (Best)
		{
			const double UniformScale = Scale.GetAbsMin();
			const double Hx = Best->X * 0.5 * UniformScale;
			const double Hy = Best->Y * 0.5 * UniformScale;
			const double Hz = Best->Z * 0.5 * UniformScale;
			const double LongHalf = FMath::Max3(Hx, Hy, Hz);
			const double MidHalf = Hx + Hy + Hz - LongHalf - FMath::Min3(Hx, Hy, Hz);
			const EAxis::Type LongAxis = (Hx >= Hy && Hx >= Hz) ? EAxis::X : (Hy >= Hz) ? EAxis::Y : EAxis::Z;
			const float SegHalf = static_cast<float>(FMath::Max(LongHalf - MidHalf, 0.0));
			const FTransform ElemTM = Best->GetTransform() * CompTM;
			const FVector AxisDir = ElemTM.GetUnitAxis(LongAxis);
			const FVector Center = ElemTM.GetLocation();
			OutA = Center + AxisDir * SegHalf;
			OutB = Center - AxisDir * SegHalf;
			OutRadius = static_cast<float>(MidHalf);
			return true;
		}
	}

	// Third, a sphere, as a degenerate capsule with coincident endpoints.
	{
		const FKSphereElem* Best = nullptr;
		float BestRadius = -1.0f;
		for (const FKSphereElem& Sphere : Agg.SphereElems)
		{
			const float R = Sphere.Radius * static_cast<float>(Scale.GetAbsMin());
			if (R > BestRadius)
			{
				BestRadius = R;
				Best = &Sphere;
			}
		}
		if (Best)
		{
			const FVector Center = CompTM.TransformPosition(Best->Center);
			OutA = Center;
			OutB = Center;
			OutRadius = BestRadius;
			return true;
		}
	}

	// No sphyl, box or sphere, as with a convex-only body, so the caller falls back to the bounds.
	return false;
}

void URopeWrapTargetComponent::BuildCapsuleFromBounds(USceneComponent* Comp,
	FVector& OutA, FVector& OutB, float& OutRadius) const
{
	const FTransform CompTM = Comp->GetComponentTransform();

	// The axis-aligned bounds in local space, obtained with an identity transform, which gives local half
	// extents and a local centre.
	const FBoxSphereBounds LocalBounds = Comp->CalcBounds(FTransform::Identity);
	// The local half extents and centre.
	const FVector Ext = LocalBounds.BoxExtent;
	const FVector LocalCenter = LocalBounds.Origin;
	// The world half extents with the component scale applied. The radius and segment have to be in world
	// units or the thickness would not match the axis length on a scaled target: with only the radius in
	// local space, a target scaled by S would have its axis scaled but not its thickness and would pass
	// through the surface. The endpoints are placed below from the world centre along the world axis unit
	// vector, which avoids applying the component scale twice. A scale of 1 leaves the behaviour unchanged.
	const FVector Scale = CompTM.GetScale3D().GetAbs();
	const FVector ScaledExt = Ext * Scale;

	// Choose the long axis, either automatically as the largest world half extent, or as configured.
	int32 AxisIdx;
	if (bAutoAxis)
	{
		AxisIdx = (ScaledExt.X >= ScaledExt.Y && ScaledExt.X >= ScaledExt.Z) ? 0 : (ScaledExt.Y >= ScaledExt.Z) ? 1 : 2;
	}
	else
	{
		AxisIdx = (Axis == ERopeWrapAxis::X) ? 0 : (Axis == ERopeWrapAxis::Y) ? 1 : 2;
	}

	// The radius is the larger of the other two world half extents, which is the capsule approximation
	// covering the axis-aligned cross-section; only the corners of a rectangular section poke out slightly.
	double OtherMax = 0.0;
	for (int32 k = 0; k < 3; ++k)
	{
		if (k != AxisIdx)
		{
			OtherMax = FMath::Max(OtherMax, static_cast<double>(ScaledExt[k]));
		}
	}
	OutRadius = static_cast<float>(FMath::Max(OtherMax, 1.0));

	// The segment half length in world units is the long axis half extent minus the radius, which keeps
	// the hemispherical caps from overshooting the ends; a negative value becomes zero, giving a sphere.
	const float SegHalf = static_cast<float>(FMath::Max(static_cast<double>(ScaledExt[AxisIdx]) - OutRadius, 0.0));

	// The endpoints are the world centre offset along the world axis unit vector by the segment half
	// length. The scale is already applied to the extents, so the component scale is not applied again to
	// the offset and only the rotation is used. At a scale of 1 this gives the same result as transforming
	// the local centre and offset together.
	FVector LocalAxis = FVector::ZeroVector;
	LocalAxis[AxisIdx] = 1.0;
	const FVector WorldAxis = CompTM.TransformVectorNoScale(LocalAxis).GetSafeNormal();
	const FVector WorldCenter = CompTM.TransformPosition(LocalCenter);
	OutA = WorldCenter + WorldAxis * SegHalf;
	OutB = WorldCenter - WorldAxis * SegHalf;
}

void URopeWrapTargetComponent::BuildBox(USceneComponent* Comp)
{
	const FTransform CompTM = Comp->GetComponentTransform();
	const FVector Scale = CompTM.GetScale3D().GetAbs();

	FVector WorldCenter = FVector::ZeroVector;
	FVector HalfExtents = FVector::ZeroVector;
	FQuat   Rot = FQuat::Identity;

	// First, the largest box element of the simple collision, as a tight oriented box that matches the
	// authored shape exactly.
	UPrimitiveComponent* Prim = Cast<UPrimitiveComponent>(Comp);
	UBodySetup* Setup = Prim ? Prim->GetBodySetup() : nullptr;
	const FKBoxElem* Best = nullptr;
	double BestSize = -1.0;
	if (Setup)
	{
		for (const FKBoxElem& BoxElem : Setup->AggGeom.BoxElems)
		{
			const double Size = FMath::Max3(static_cast<double>(BoxElem.X), static_cast<double>(BoxElem.Y), static_cast<double>(BoxElem.Z));
			if (Size > BestSize)
			{
				BestSize = Size;
				Best = &BoxElem;
			}
		}
	}

	if (Best)
	{
		const FTransform ElemTM = Best->GetTransform() * CompTM;
		WorldCenter = ElemTM.GetLocation();
		Rot = ElemTM.GetRotation();
		// The element's dimensions are full lengths, so they become half extents scaled by the component.
		HalfExtents = FVector(Best->X, Best->Y, Best->Z) * 0.5 * Scale;
	}
	else
	{
		// Second, the fallback: the component's local bounds as an oriented box, used when there is no box
		// simple collision.
		const FBoxSphereBounds LocalBounds = Comp->CalcBounds(FTransform::Identity);
		WorldCenter = CompTM.TransformPosition(LocalBounds.Origin);
		Rot = CompTM.GetRotation();
		HalfExtents = LocalBounds.BoxExtent * Scale;
	}

	// A virtual bone plus a source mesh makes IsWorldStatic() false, so it takes part in detection as a
	// wrap target. It assumes a static target, with a zero reciprocal delta and therefore no surface
	// velocity.
	Box = FRopeBoxCollider(WorldCenter, Rot, HalfExtents);
	Box.Bone = ResolvedBone;
	Box.SourceMesh = Comp;
}

bool URopeWrapTargetComponent::BuildFullSet(USceneComponent* Comp)
{
	UPrimitiveComponent* Prim = Cast<UPrimitiveComponent>(Comp);
	UBodySetup* Setup = Prim ? Prim->GetBodySetup() : nullptr;
	if (!Setup)
	{
		return false;
	}

	const UDynamicRopeSettings* Settings = UDynamicRopeSettings::Get();
	const int32 MaxCol = Settings ? Settings->StaticBodyMaxCollidersPerRope : 32;
	const int32 MaxPlanes = Settings ? Settings->StaticBodyMaxConvexPlanes : 32;

	// The surface velocity of a movable prop, from the component's previous rigid transform, which the
	// extraction helper expands into per-element previous endpoints and transforms.
	const FTransform CompTM = Comp->GetComponentTransform();
	const float FrameDt = GetWorld() ? GetWorld()->GetDeltaSeconds() : 0.0f;
	const float InvDt = (bHasPrevCompTM && FrameDt > KINDA_SMALL_NUMBER) ? 1.0f / FrameDt : 0.0f;
	const FTransform PrevTM = bHasPrevCompTM ? PrevCompTM : CompTM;

	// Extract the whole simple collision with virtual bone attribution, which makes every sphyl, sphere,
	// box and convex element wrappable; the GPU detection kernel has a convex loop, so convexes take part
	// in detection too.
	RopeBodyColliderExtraction::AppendBodyColliders(*Setup, CompTM, PrevTM, InvDt, MaxCol, MaxPlanes,
		WrapBoxes, WrapCapsules, WrapConvexes, [](int32) {}, ResolvedBone, Comp);
	PrevCompTM = CompTM;
	bHasPrevCompTM = true;

	// The designer's radius override means the same as on the single-shape path, keeping the shape and the
	// axis and changing only the thickness, applied uniformly across every capsule.
	if (Radius > 0.0f)
	{
		for (FRopeStaticCapsuleCollider& Cap : WrapCapsules)
		{
			Cap.Radius = Radius;
		}
	}


	// With not a single wrappable element, as when no collision was authored, it falls back to the
	// single-shape path.
	if (WrapBoxes.Num() + WrapCapsules.Num() + WrapConvexes.Num() == 0)
	{
		WrapConvexes.Reset();
		return false;
	}
	return true;
}

bool URopeWrapTargetComponent::EffectiveServeBox(USceneComponent* Comp) const
{
	if (Shape == ERopeWrapShape::Box)     { return true; }
	if (Shape == ERopeWrapShape::Capsule) { return false; }

	// Under Auto: a box when the dominant, that is largest, primitive of the simple collision is a box,
	// and a capsule for a sphyl or a sphere.
	UPrimitiveComponent* Prim = Cast<UPrimitiveComponent>(Comp);
	UBodySetup* Setup = Prim ? Prim->GetBodySetup() : nullptr;
	if (!Setup)
	{
		// With no simple collision it becomes a capsule, through the bounds fallback.
		return false;
	}
	const FKAggregateGeom& Agg = Setup->AggGeom;

	double BoxSize = -1.0;
	for (const FKBoxElem& BoxElem : Agg.BoxElems)
	{
		BoxSize = FMath::Max(BoxSize, FMath::Max3(static_cast<double>(BoxElem.X), static_cast<double>(BoxElem.Y), static_cast<double>(BoxElem.Z)));
	}
	double CapsuleSize = -1.0;
	for (const FKSphylElem& Sphyl : Agg.SphylElems)
	{
		CapsuleSize = FMath::Max(CapsuleSize, static_cast<double>(Sphyl.Length + 2.0f * Sphyl.Radius));
	}
	for (const FKSphereElem& Sphere : Agg.SphereElems)
	{
		CapsuleSize = FMath::Max(CapsuleSize, static_cast<double>(2.0f * Sphere.Radius));
	}

	// A box wins when one exists and is at least as large as the capsule-like primitives; with no box it is
	// a capsule.
	return BoxSize > 0.0 && BoxSize >= CapsuleSize;
}

void URopeWrapTargetComponent::GatherColliders(FRopeColliderGatherContext& Gather)
{
	USceneComponent* Comp = ResolveTarget();
	if (!Comp)
	{
		if (!bDiagnosticsLogged)
		{
			bDiagnosticsLogged = true;
			UE_LOG(LogRopeCollision, Warning,
				TEXT("[WrapTarget] %s: no target component, so no wrap capsule was created. Assign TargetComponent or add a static mesh."),
				*GetNameSafe(GetOwner()));
		}
		return;
	}

	// Built once per frame, deduplicated, so several ropes aiming at the same target do not rebuild the
	// capsule.
	const uint64 Frame = GFrameCounter;
	if (BuiltFrame != Frame)
	{
		BuiltFrame = Frame;
		WrapBoxes.Reset();
		WrapCapsules.Reset();
		WrapConvexes.Reset();
		PushOutBoxes.Reset();
		PushOutCapsules.Reset();
		PushOutConvexes.Reset();

		// Full-set mode, meaning Auto with simple collision present: instead of approximating with a single
		// dominant primitive, every simple collision element is served as wrappable, at the precision the
		// static body provider extracts. On failure, whether from absent collision or a convex-only body,
		// or when Capsule or Box is forced, it takes the single-shape path.
		bServeFullSet = (Shape == ERopeWrapShape::Auto) && BuildFullSet(Comp);
		if (!bServeFullSet)
		{
			// Under Auto the shape is decided from the simple collision.
			bServeBox = EffectiveServeBox(Comp);
			if (bServeBox)
			{
				BuildBox(Comp);
			}
			else
			{
				BuildCapsule(Comp);
			}

			// Detailed push-out, decided automatically from the channel: when the target sits outside the
			// static body provider's scan channels, meaning WorldStatic plus WorldDynamic where enabled, as
			// a physics-body prop made movable and simulating so it can be dragged, that provider cannot see
			// it, so the target's whole simple collision is extracted here as push-out colliders. Those have
			// no bone and are excluded from detection, leaving only the single shape above to take part in
			// wrapping. Where the channel is covered this is skipped and the static body provider handles
			// it, so nothing is duplicated. The condition reads bIncludeWorldDynamic to mirror the
			// provider's real scan range, which fills the gap when that setting is off.
			if (UPrimitiveComponent* Prim = Cast<UPrimitiveComponent>(Comp))
			{
				const UDynamicRopeSettings* Settings = UDynamicRopeSettings::Get();
				const ECollisionChannel ObjType = Prim->GetCollisionObjectType();
				const bool bCoveredByStaticProvider = ObjType == ECC_WorldStatic ||
					(Settings && Settings->bIncludeWorldDynamic && ObjType == ECC_WorldDynamic);
				UBodySetup* Setup = Prim->GetBodySetup();
				if (!bCoveredByStaticProvider && Setup)
				{
					const FTransform CompTM = Comp->GetComponentTransform();
					const float FrameDt = GetWorld() ? GetWorld()->GetDeltaSeconds() : 0.0f;
					const float InvDt = (bHasPrevCompTM && FrameDt > KINDA_SMALL_NUMBER) ? 1.0f / FrameDt : 0.0f;
					const FTransform PrevTM = bHasPrevCompTM ? PrevCompTM : CompTM;
					const int32 MaxCol = Settings ? Settings->StaticBodyMaxCollidersPerRope : 32;
					const int32 MaxPlanes = Settings ? Settings->StaticBodyMaxConvexPlanes : 32;
					RopeBodyColliderExtraction::AppendBodyColliders(*Setup, CompTM, PrevTM, InvDt,
						MaxCol, MaxPlanes, PushOutBoxes, PushOutCapsules, PushOutConvexes, [](int32){});
					PrevCompTM = CompTM;
					bHasPrevCompTM = true;
				}
			}
		} // End of the single-shape fallback path.

		// Diagnostics: log the shape, its geometry and its bone once on the first build. This log never
		// appearing means GatherColliders was never called, which is either the provider not being
		// registered, the rope's owner being excluded because it is the same actor, or no rope being
		// nearby.
		if (!bDiagnosticsLogged)
		{
			bDiagnosticsLogged = true;
			const FBoxSphereBounds DiagBounds = Comp->CalcBounds(FTransform::Identity);
			if (bServeFullSet)
			{
				UE_LOG(LogRopeCollision, Log,
					TEXT("[WrapTarget] %s: built the full wrap set, capsules=%d boxes=%d convexes=%d bone=%s"),
					*GetNameSafe(GetOwner()), WrapCapsules.Num(), WrapBoxes.Num(), WrapConvexes.Num(),
					*ResolvedBone.ToString());
			}
			else if (bServeBox)
			{
				UE_LOG(LogRopeCollision, Log,
					TEXT("[WrapTarget] %s: built a wrap box, localExtent=%s | center=%s half=%s bone=%s"),
					*GetNameSafe(GetOwner()), *DiagBounds.BoxExtent.ToCompactString(),
					*Box.Center.ToCompactString(), *Box.HalfExtents.ToCompactString(), *Box.Bone.ToString());
			}
			else
			{
				UE_LOG(LogRopeCollision, Log,
					TEXT("[WrapTarget] %s: built a wrap capsule, source=%s localExtent=%s | A=%s B=%s R=%.1f len=%.1f bone=%s"),
					*GetNameSafe(GetOwner()),
					bUsedSimpleCollision ? TEXT("simpleCollision") : TEXT("bounds"),
					*DiagBounds.BoxExtent.ToCompactString(),
					*Capsule.A.ToCompactString(), *Capsule.B.ToCompactString(),
					Capsule.Radius, static_cast<float>(FVector::Dist(Capsule.A, Capsule.B)),
					*Capsule.Bone.ToString());
			}
		}
	}

	// The cached collider pointers are placed in the pool, valid for that frame's solve, and the region
	// mapping is produced by the bounds helper, on the same contract as the skeletal providers; with a
	// single shape the union is simply itself.
	const int32 StartIndex = Gather.Colliders.Num();
	if (bServeFullSet)
	{
		// Full set: every wrappable element, attributed to the virtual bone so it takes part in detection.
		for (FRopeBoxCollider& B : WrapBoxes)              { Gather.Colliders.Add(&B); }
		for (FRopeStaticCapsuleCollider& C : WrapCapsules) { Gather.Colliders.Add(&C); }
		for (FRopeConvexCollider& Cv : WrapConvexes)       { Gather.Colliders.Add(&Cv); }
	}
	else if (bServeBox)
	{
		Gather.Colliders.Add(&Box);
	}
	else
	{
		Gather.Colliders.Add(&Capsule);
	}
	// The detailed push-out shapes are served alongside, where they exist, which is only when the automatic
	// channel decision found the target outside the covered channels. They are mapped to regions as one
	// group with the wrap shapes: the mapping works from world bounds, so mixing types is irrelevant and a
	// single call below suffices.
	for (FRopeBoxCollider& B : PushOutBoxes)              { Gather.Colliders.Add(&B); }
	for (FRopeStaticCapsuleCollider& C : PushOutCapsules) { Gather.Colliders.Add(&C); }
	for (FRopeConvexCollider& Cv : PushOutConvexes)       { Gather.Colliders.Add(&Cv); }
	RopeColliderGather::MapCollidersToRegionsByBounds(Gather, StartIndex);
}
