// Copyright Epic Games, Inc. All Rights Reserved.
//
// The wrap target abstraction. It lifts the assumption that a wrap target is one bone on a skeletal
// mesh out of the code, providing the common foundation shared by wrapping a static mesh and
// wrapping a group of bones.
//
// The binding seam is what one anchor follows each frame: FRopeBindingFrame together with
// ResolveBindingWorld, consumed by Hold and BeginWrap.
//
// ResolveBindingWorld is wired through the whole wrap path, covering Hold, BeginWrap, the path
// build, anchor restoration and the preview. The skeleton structure queries in RopeWrapTargets::,
// namely the parent and child keys and the skeletal test, likewise replace the inline casts that
// used to be spread through the wrap path, so the assumption that a target is skeletal now exists
// only in this header and its implementation file.
//
// Room to extend: once wrapping a group of bones, or an opt-in designer axis, becomes concrete, an
// aggregation seam is needed for the question of what unit contacts are aggregated under. At that
// point a registry declared by the provider that owns the target can be introduced and the bodies of
// the RopeWrapTargets:: functions replaced with lookups into it, leaving the call sites unchanged.
// It is deliberately not added before there is a real need for it.

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtrTemplates.h"
#include "Core/RopeContactTrackingTypes.h"
#include "Core/RopeSimTypes.h"
#include "Core/RopeWrappingTypes.h"

class USceneComponent;
class IRopeCollider;

/**
 * The binding frame describing what an anchor or wrap node is attached to and follows each frame.
 * Plain data, in keeping with the hot-loop rule. It replaces the calls to
 * Mesh->GetSocketTransform(Bone) scattered through the code with this one type plus
 * ResolveBindingWorld.
 * The component is held weakly, so a destroyed cross-actor target becomes null safely, following the
 * same rule as FRopeWrapState::Mesh.
 */
struct FRopeBindingFrame
{
	/**
	 * The component being followed. A skeletal one is downcast to USkeletalMeshComponent to use the
	 * skinned socket transform; anything else, such as a static or movable prop, uses the component or
	 * socket transform. ResolveBindingWorld decides which.
	 */
	TWeakObjectPtr<const USceneComponent> Component = nullptr;

	/** For a skeletal target, the bone or socket name. For a static one, the socket name if there is
	 *  one, or None to mean the component transform. */
	FName SocketOrBone = NAME_None;

	/** Whether a valid transform can still be produced, which detects a destroyed target. A caller
	 *  seeing false should release. */
	bool IsValid() const { return Component.IsValid(); }
};

/**
 * Resolves a binding frame into this frame's world transform. It is the single resolution point that
 * replaces every Mesh->GetSocketTransform(Bone) on the wrap path. A skeletal component with a bone
 * name gives the skinned socket transform; anything else gives the component or socket transform,
 * which makes a static target hold still automatically, since its transform never changes, while a
 * movable prop is followed.
 * An invalid component returns the identity; callers are advised to filter with IsValid() and release
 * first.
 */
DYNAMICROPE_API FTransform ResolveBindingWorld(const FRopeBindingFrame& Frame);

/**
 * The same resolution as a raw pointer overload, for callers that already hold a mesh and bone and
 * call it several times within a frame, such as the path build and anchor placement, so they need not
 * construct a weak frame. A null component gives the identity.
 */
DYNAMICROPE_API FTransform ResolveBindingWorld(const USceneComponent* Component, FName SocketOrBone);

/**
 * Structural queries about a wrap target's skeleton. The wrap flow, meaning the wrap axis, the bone
 * graph and classification, uses these instead of casting to decide what kind of target it has, which
 * isolates the skeletal assumption in this one implementation file.
 * Extension point: once bone groups or designer-authored transition edges arrive, the bodies of these
 * functions become registry lookups, leaving the call sites unchanged.
 */
namespace RopeWrapTargets
{
	/** Whether the target is skeletal, meaning it has a bone graph. False for static and virtual-bone
	 *  targets. */
	DYNAMICROPE_API bool IsSkeletalTarget(const USceneComponent* Mesh);

	/** The parent key of a target key, that is a bone. On a skeletal target this is the parent bone
	 *  name; on anything else, including a static target, a virtual bone or a root, it is None because
	 *  there is no graph. */
	DYNAMICROPE_API FName GetParentTargetKey(const USceneComponent* Mesh, FName Bone);

	/**
	 * Appends the child keys of a target key to OutChildren. On a skeletal target these are every bone
	 * whose parent is Bone; on anything else there are none. Used to enumerate neighbours while
	 * expanding the surface vector field's bone graph with a bounded Dijkstra search.
	 */
	DYNAMICROPE_API void AppendChildTargetKeys(const USceneComponent* Mesh, FName Bone, TArray<FName>& OutChildren);

	/**
	 * Applies the URopeComponent::CanWrapTarget gate to a collider snapshot and fills in OutColliders.
	 * It is the gate that stops the wrap path build from laying anchors on a forbidden target.
	 *
	 * It identifies targets the same way the wrap path does, through IRopeCollider::GetGPUAttribution.
	 * A collider with no attribution, such as static world geometry where the bone is None and the mesh
	 * is null, can never be a wrap target in the first place and is used purely as surface geometry, so
	 * it is kept regardless of the gate.
	 *
	 * When the gate's default implementation permits everything, the output equals the input, so a rope
	 * that does not override it behaves identically by definition.
	 */
	DYNAMICROPE_API void FilterWrappableColliders(
		const TArray<IRopeCollider*>& InColliders,
		TFunctionRef<bool(const USceneComponent*, FName)> CanWrapTarget,
		TArray<IRopeCollider*>& OutColliders);
}
