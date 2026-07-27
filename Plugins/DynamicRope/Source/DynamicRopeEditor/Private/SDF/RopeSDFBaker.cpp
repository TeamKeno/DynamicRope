// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeSDFBaker.h"
#include "DynamicRopeEditorLog.h"
#include "Collision/SDF/RopeSDFData.h"
#include "Engine/SkeletalMesh.h"
#include "Rendering/SkeletalMeshModel.h"
#include "Rendering/SkeletalMeshLODModel.h"
#include "Async/ParallelFor.h"

// The fast winding number from GeometryCore, used to decide the sign.
#include "DynamicMesh/DynamicMesh3.h"
#include "IndexTypes.h"
#include "Spatial/MeshAABBTree3.h"
#include "Spatial/FastWinding.h"

namespace
{
	using UE::Geometry::FDynamicMesh3;
	using UE::Geometry::TMeshAABBTree3;
	using UE::Geometry::TFastWindingTree;
	using UE::Geometry::FIndex3i;

	// Decides whether a point is inside or outside from the generalized winding number over the whole
	// mesh. Being inside is a global property of the whole body, treated as a closed surface, so the
	// winding has to be taken over the entire mesh rather than a bone's open patch to be robust; over an
	// open patch the interior of a short, wide bone segment is misjudged as outside.
	// The fast winding implementation, a bounding volume hierarchy with a multipole approximation, makes
	// each query logarithmic in the triangle count. It is built once per bake and queried per voxel, and
	// concurrent read-only queries after the build are safe.
	class FRopeSDFWindingClassifier
	{
	public:
		// Built from a triangle soup, duplicating vertices per triangle, which is robust to seams and
		// non-manifold geometry since winding does not depend on connectivity.
		// Triangle t is formed from the three indices starting at 3t. The vertices and the query points
		// have to be in the same coordinate space, which here is component space.
		FRopeSDFWindingClassifier(TConstArrayView<FVector3f> Positions, TConstArrayView<uint32> Indices)
		{
			if (Positions.Num() == 0 || Indices.Num() < 3)
			{
				return;
			}
			const int32 NumTris = Indices.Num() / 3;
			for (int32 t = 0; t < NumTris; ++t)
			{
				const int32 I0 = static_cast<int32>(Indices[3 * t + 0]);
				const int32 I1 = static_cast<int32>(Indices[3 * t + 1]);
				const int32 I2 = static_cast<int32>(Indices[3 * t + 2]);
				if (!Positions.IsValidIndex(I0) || !Positions.IsValidIndex(I1) || !Positions.IsValidIndex(I2))
				{
					continue;
				}
				const FVector3f& P0 = Positions[I0];
				const FVector3f& P1 = Positions[I1];
				const FVector3f& P2 = Positions[I2];
				const int32 V0 = Mesh.AppendVertex(FVector3d(P0.X, P0.Y, P0.Z));
				const int32 V1 = Mesh.AppendVertex(FVector3d(P1.X, P1.Y, P1.Z));
				const int32 V2 = Mesh.AppendVertex(FVector3d(P2.X, P2.Y, P2.Z));
				Mesh.AppendTriangle(FIndex3i(V0, V1, V2));
			}
			if (Mesh.TriangleCount() == 0)
			{
				return;
			}
			// The tree points at the mesh and the winding object points at the tree, so as members their
			// addresses have to stay stable for the object's lifetime. This classifier is therefore used in
			// place and never copied or moved; BakeMesh creates it as a local constant.
			Tree = MakeUnique<TMeshAABBTree3<FDynamicMesh3>>(&Mesh, true);
			Winding = MakeUnique<TFastWindingTree<FDynamicMesh3>>(Tree.Get(), true);
		}

		// A generalized winding number whose magnitude exceeds one half means inside. That is the midpoint
		// between roughly one inside a closed mesh and roughly zero outside, and taking the magnitude makes
		// it independent of the triangle winding order.
		bool IsInside(const FVector& P) const
		{
			if (!Winding)
			{
				return false;
			}
			return FMath::Abs(Winding->FastWindingNumber(FVector3d(P.X, P.Y, P.Z))) > 0.5;
		}

	private:
		FDynamicMesh3 Mesh;
		TUniquePtr<TMeshAABBTree3<FDynamicMesh3>> Tree;
		TUniquePtr<TFastWindingTree<FDynamicMesh3>> Winding;
	};
}

ERopeSDFBakeResult FRopeSDFBaker::BakeMesh(USkeletalMesh* Mesh, const TArray<FName>& BonesIn,
	const FRopeSDFBakeSettings& S, TArray<FRopeBoneSDFVolume>& Out,
	const FRopeSDFBakeProgress& Progress, const FRopeSDFBakeCancelPoll& CancelPoll,
	FRopeSDFBakeStats* OutStats)
{
	Out.Reset();
	if (OutStats)
	{
		*OutStats = FRopeSDFBakeStats();
	}
	if (!Mesh)
	{
		UE_LOG(LogRopeSDFBake, Warning, TEXT("BakeMesh aborted: null mesh."));
		return ERopeSDFBakeResult::NoGeometry;
	}

	FSkeletalMeshModel* Model = Mesh->GetImportedModel();
	if (!Model || Model->LODModels.Num() == 0)
	{
		UE_LOG(LogRopeSDFBake, Warning, TEXT("BakeMesh aborted: %s has no CPU geometry (cooked/stripped)."), *Mesh->GetName());
		// No CPU geometry, as in a cooked or stripped build.
		return ERopeSDFBakeResult::NoGeometry;
	}

	UE_LOG(LogRopeSDFBake, Log, TEXT("BakeMesh start: %s (voxel=%.2fcm, maxRes=%d, narrowBand=%.1fcm)"),
		*Mesh->GetName(), S.VoxelSize, S.MaxResolution, S.NarrowBand);

	const FSkeletalMeshLODModel& LOD = Model->LODModels[0];
	const FReferenceSkeleton& Ref = Mesh->GetRefSkeleton();

	// Step one: compute each bone's component-space reference pose transform by accumulating the parent
	// chain.
	// The vertices are stored in the reference pose's component space, so the inverse of that transform
	// moves them into bone-local space, and it is the same frame the runtime provider reconstructs through
	// the socket transform.
	const TArray<FTransform>& LocalPose = Ref.GetRefBonePose();
	TArray<FTransform> CompSpace;
	CompSpace.SetNum(LocalPose.Num());
	for (int32 b = 0; b < LocalPose.Num(); ++b)
	{
		const int32 Parent = Ref.GetParentIndex(b);
		CompSpace[b] = (Parent == INDEX_NONE) ? LocalPose[b] : LocalPose[b] * CompSpace[Parent];
	}

	// Step two: build the flat vertex list, in global index order matching the index buffer, plus the
	// section each vertex belongs to.
	// The section is needed to resolve a section-local influence index into a skeleton bone index through
	// the bone map.
	TArray<FSoftSkinVertex> Verts;
	LOD.GetVertices(Verts);

	TArray<int32> VertSection;
	VertSection.Init(0, Verts.Num());
	for (int32 s = 0; s < LOD.Sections.Num(); ++s)
	{
		const FSkelMeshSection& Sec = LOD.Sections[s];
		for (int32 v = 0; v < Sec.NumVertices; ++v)
		{
			VertSection[static_cast<int32>(Sec.BaseVertexIndex) + v] = s;
		}
	}

	// A vertex's normalized skin weight for a given skeleton bone. It is scale-independent, since it
	// divides by the sum of the weights, so it works whether the influence weights are 8-bit or 16-bit.
	auto WeightFor = [&](int32 Vtx, int32 BoneIdx) -> float
	{
		const FSoftSkinVertex& V = Verts[Vtx];
		const FSkelMeshSection& Sec = LOD.Sections[VertSection[Vtx]];
		float Sum = 0.0f, Match = 0.0f;
		for (int32 i = 0; i < MAX_TOTAL_INFLUENCES; ++i)
		{
			const float W = static_cast<float>(V.InfluenceWeights[i]);
			Sum += W;
			if (Sec.BoneMap.IsValidIndex(V.InfluenceBones[i]) &&
				Sec.BoneMap[V.InfluenceBones[i]] == BoneIdx)
			{
				Match += W;
			}
		}
		return Sum > 0.0f ? Match / Sum : 0.0f;
	};

	// The target bone set. An empty input means every bone appearing in a section's bone map, that is
	// every skinned bone.
	TArray<int32> Targets;
	if (BonesIn.Num() > 0)
	{
		for (const FName& Bone : BonesIn)
		{
			const int32 Idx = Ref.FindBoneIndex(Bone);
			if (Idx != INDEX_NONE)
			{
				Targets.AddUnique(Idx);
			}
		}
	}
	else
	{
		TSet<int32> Skinned;
		for (const FSkelMeshSection& Sec : LOD.Sections)
		{
			for (const FBoneIndexType Bm : Sec.BoneMap)
			{
				Skinned.Add(static_cast<int32>(Bm));
			}
		}
		Targets = Skinned.Array();
	}

	const TArray<uint32>& Indices = LOD.IndexBuffer;

	// For the sign: build the fast winding classifier once over the whole mesh. Being inside is a global
	// property of the whole body, treated as a closed surface, so the winding has to be taken over the
	// entire mesh rather than a bone's open patch to be robust. Distances are still measured against each
	// bone's own triangles, which preserves the attribution. The vertices stay in component space, the
	// space they are stored in, and the queries use that same space.
	TArray<FVector3f> AllPositions;
	AllPositions.Reserve(Verts.Num());
	for (const FSoftSkinVertex& V : Verts)
	{
		AllPositions.Add(V.Position);
	}
	const FRopeSDFWindingClassifier WindingClassifier(AllPositions, Indices);

	const int32 TotalTargets = Targets.Num();
	int32 DoneTargets = 0;
	for (int32 BoneIdx : Targets)
	{
		// Report progress just before processing each bone, including bones that will be skipped, since the
		// unit of progress is the target bone. A callback returning false aborts immediately.
		if (Progress)
		{
			const FName BoneName = CompSpace.IsValidIndex(BoneIdx) ? Ref.GetBoneName(BoneIdx) : NAME_None;
			if (!Progress(DoneTargets, TotalTargets, BoneName))
			{
				UE_LOG(LogRopeSDFBake, Log, TEXT("BakeMesh cancelled: %s at bone %d/%d (%s)."),
					*Mesh->GetName(), DoneTargets, TotalTargets, *BoneName.ToString());
				return ERopeSDFBakeResult::Cancelled;
			}
		}
		++DoneTargets;

		if (!CompSpace.IsValidIndex(BoneIdx))
		{
			continue;
		}
		const FTransform InvBone = CompSpace[BoneIdx].Inverse();

		// Step three: gather this bone's triangles into bone-local space and grow the AABB alongside.
		TArray<FVector> TriA, TriB, TriC;
		FBox Local(ForceInit);
		for (const FSkelMeshSection& Sec : LOD.Sections)
		{
			for (uint32 t = 0; t < Sec.NumTriangles; ++t)
			{
				const uint32 I0 = Indices[Sec.BaseIndex + 3 * t + 0];
				const uint32 I1 = Indices[Sec.BaseIndex + 3 * t + 1];
				const uint32 I2 = Indices[Sec.BaseIndex + 3 * t + 2];

				const float Avg = (WeightFor(I0, BoneIdx) + WeightFor(I1, BoneIdx) + WeightFor(I2, BoneIdx)) / 3.0f;
				if (Avg < S.WeightThreshold)
				{
					continue;
				}

				const FVector A = InvBone.TransformPosition(FVector(Verts[I0].Position));
				const FVector B = InvBone.TransformPosition(FVector(Verts[I1].Position));
				const FVector C = InvBone.TransformPosition(FVector(Verts[I2].Position));
				TriA.Add(A); TriB.Add(B); TriC.Add(C);
				Local += A; Local += B; Local += C;
			}
		}
		if (TriA.Num() == 0)
		{
			// No skinned geometry is attributed to this bone.
			continue;
		}

		// Step three (b): drop thin bones. Of the three sides of the raw AABB, before any expansion, the
		// longest is the bone axis and is discarded; of the two remaining cross-section sides, the larger,
		// that is the median, is compared against the minimum girth, and a bone thinner than that in every
		// direction is not baked.
		// The median rather than the smallest is used so a bone that is flat in only one direction stays
		// catchable, which keeps the test conservative and avoids dropping one by mistake. Unlike merging,
		// dropping does not hand the triangles to the parent and simply excludes them.
		if (S.MinBoneGirth > 0.0f)
		{
		// The raw triangle AABB, before the padding and narrow band expansion.
			const FVector E = Local.GetSize();
			const double Girth = (E.X + E.Y + E.Z)
				// Remove the longest side, which is the bone axis.
				- FMath::Max3(E.X, E.Y, E.Z)
				// Remove the shortest side, which leaves the median.
				- FMath::Min3(E.X, E.Y, E.Z);
			if (Girth < S.MinBoneGirth)
			{
				if (OutStats)
				{
					OutStats->DroppedThinBones.Add(Ref.GetBoneName(BoneIdx));
				}
				UE_LOG(LogRopeSDFBake, Verbose, TEXT("  drop thin bone %s (girth %.2fcm < %.2fcm)"),
					*Ref.GetBoneName(BoneIdx).ToString(), Girth, S.MinBoneGirth);
				continue;
			}
		}

		// Step four (a): size the grid. The voxels are cubic, and if the sample count on any axis would
		// exceed the maximum resolution the voxel size is increased to fit.
		Local = Local.ExpandBy(S.BoundsPadding + S.NarrowBand);
		float Vox = FMath::Max(S.VoxelSize, KINDA_SMALL_NUMBER);
		const FVector Size = Local.GetSize();

		auto ResFor = [&](float V)
		{
			return FIntVector(
				FMath::CeilToInt(Size.X / V) + 1,
				FMath::CeilToInt(Size.Y / V) + 1,
				FMath::CeilToInt(Size.Z / V) + 1);
		};
		FIntVector Res = ResFor(Vox);
		const int32 MaxAxis = FMath::Max3(Res.X, Res.Y, Res.Z);
		// Whether the requested voxel size had to be increased because of the limit, for reporting.
		bool bCoarsened = false;
		if (MaxAxis > S.MaxResolution)
		{
			Vox *= static_cast<float>(MaxAxis) / static_cast<float>(S.MaxResolution);
			Res = ResFor(Vox);
			bCoarsened = true;
		}
		Res.X = FMath::Max(Res.X, 2);
		Res.Y = FMath::Max(Res.Y, 2);
		Res.Z = FMath::Max(Res.Z, 2);

		// Snap the bounds so the spacing is exactly the voxel size and the samples sit on corners.
		// sample(x,y,z) = Min + (x,y,z) * Vox,  Max = Min + (Res - 1) * Vox.
		const FVector Min = Local.Min;
		const FVector Max = Min + FVector(Res.X - 1, Res.Y - 1, Res.Z - 1) * Vox;

		// Step four (b): voxelize. For each sample the unsigned distance is the minimum point-to-triangle
		// distance against this bone, and the sign comes from the whole-mesh fast winding.
		// It is parallelized over batches of flat indices; both the distance triangles and the winding tree
		// are read-only, so there is no contention.
		// Arrays are counted with a 32-bit integer, so the linear index stays 32-bit as well. The resolution
		// is capped by the maximum resolution setting, which keeps it comfortably in range.
		const int32 Count = Res.X * Res.Y * Res.Z;
		// The first pass collects the signed distances as floats. The inward band is then derived per bone
		// from that data, and the second pass quantizes asymmetrically, with the inward band being the
		// deepest interior distance and the outward band the configured detection band. Each raw distance
		// is independent per index and therefore safe to compute in parallel.
		TArray<float> RawDist;
		RawDist.SetNumUninitialized(Count);

		// The distance is measured against this bone's triangles.
		const int32 NumTris = TriA.Num();
		// Converts a bone-local sample point into component space, which is the classifier's frame.
		const FTransform& BoneToComp = CompSpace[BoneIdx];

		// Computes and stores the signed distance for one flat index.
		auto ComputeSample = [&](int32 Flat)
		{
			const int32 x = Flat % Res.X;
			const int32 y = (Flat / Res.X) % Res.Y;
			const int32 z = Flat / (Res.X * Res.Y);
			// In bone-local space.
			const FVector P = Min + FVector(x, y, z) * Vox;

			// The unsigned distance: the minimum point-to-triangle distance against this bone's triangles,
			// which preserves the attribution.
			float Best = BIG_NUMBER;
			for (int32 k = 0; k < NumTris; ++k)
			{
				const FVector CP = FMath::ClosestPointOnTriangleToPoint(P, TriA[k], TriB[k], TriC[k]);
				Best = FMath::Min(Best, static_cast<float>(FVector::Dist(P, CP)));
			}

			// The sign, meaning inside or outside, comes from the whole-mesh fast winding, because a bone's
			// open patch misjudges the interior of a short, wide segment as outside. The sample point is
			// lifted into component space to query it.
			const FVector Pc = BoneToComp.TransformPosition(P);
			// Negative inside and positive outside.
			RawDist[Flat] = WindingClassifier.IsInside(Pc) ? -Best : Best;
		};

		// A heavy bone is split into several batches, with a cancellation poll between them, so the game
		// thread can process the cancel button. Within a batch it still uses every core through a parallel
		// loop, and splitting by flat index does not affect the result; a light bone ends up with a single
		// batch and behaves exactly as one parallel loop.
		const int64 Work = static_cast<int64>(Count) * static_cast<int64>(FMath::Max(NumTris, 64));
		// The approximate work per batch. It is kept small, shorter than the UI update throttle, so
		// cancelling responds immediately.
		const int64 TargetOpsPerBatch = 1024 * 1024;
		const int32 NumBatches = static_cast<int32>(FMath::Clamp<int64>(Work / TargetOpsPerBatch, 1, Count));
		const int32 PerBatch = FMath::DivideAndRoundUp(Count, NumBatches);

		for (int32 Start = 0; Start < Count; Start += PerBatch)
		{
			if (CancelPoll && CancelPoll())
			{
				UE_LOG(LogRopeSDFBake, Log, TEXT("BakeMesh cancelled: %s during bone %s voxelization."),
					*Mesh->GetName(), *Ref.GetBoneName(BoneIdx).ToString());
				return ERopeSDFBakeResult::Cancelled;
			}
			const int32 End = FMath::Min(Start + PerBatch, Count);
			ParallelFor(End - Start, [&](int32 i) { ComputeSample(Start + i); });
		}

		// Step four (c): settle the per-bone asymmetric bands and quantize, in two passes. The outward band
		// is the configured detection band, while the inward band is derived automatically as this bone's
		// deepest interior distance, that is the magnitude of the most negative value, so the whole interior
		// falls within the band and even a deeply penetrating node recovers towards the nearest surface.
		// The interior voxels already exist in the grid, so widening the band does not change the voxel
		// count and costs nothing; the only price is a larger quantization step, since the step is the total
		// band range divided by the code range.
		// The most negative distance, that is the deepest interior. With no interior it stays at zero, which
		// leaves the inward band at zero and allocates the whole code range outwards.
		float MinD = 0.0f;
		for (int32 i = 0; i < Count; ++i)
		{
			MinD = FMath::Min(MinD, RawDist[i]);
		}
		// At or above zero; the deepest interior distance.
		const float NBIn = -FMath::Min(MinD, 0.0f);
		// The configured outward detection band, floored to avoid dividing by zero.
		const float NBOut = FMath::Max(S.NarrowBand, KINDA_SMALL_NUMBER);
		// The configured bit depth.
		const int32 BytesPerCode = (S.Quantization == ERopeSDFQuantBits::UInt16) ? 2 : 1;

		// The blob of quantization codes, holding the configured bytes per voxel and mapping the band range
		// onto the code range.
		TArray<uint8> Distances;
		Distances.SetNumUninitialized(Count * BytesPerCode);
		ParallelFor(Count, [&](int32 i)
		{
			// Clamp to the band and quantize; the encoder handles both the clamping and the byte storage.
			// Positive is outside, as the frozen contact contract requires.
			FRopeBoneSDFVolume::EncodeInto(Distances, i, RawDist[i], NBIn, NBOut, BytesPerCode);
		});

		FRopeBoneSDFVolume Volume;
		Volume.Bone = Ref.GetBoneName(BoneIdx);
		Volume.LocalBounds = FBox(Min, Max);
		Volume.Resolution = Res;
		Volume.VoxelSize = Vox;
		// Dequantization: code zero maps to the negative inward band, which is derived per bone and covers
		// the interior.
		Volume.NarrowBandInner = NBIn;
		// Dequantization: the maximum code maps to the positive outward band, which is the configured
		// detection band.
		Volume.NarrowBandOuter = NBOut;
		// The byte layout, of one or two bytes per voxel.
		Volume.QuantBits = S.Quantization;
		Volume.Distances = MoveTemp(Distances);
		const float StepCm = (NBIn + NBOut) / static_cast<float>((BytesPerCode >= 2) ? 65535 : 255);
		UE_LOG(LogRopeSDFBake, Verbose, TEXT("  bone %s: res=%dx%dx%d, voxel=%.2fcm, %d tri(s), band[-%.2f,+%.2f]cm (%d-bit, step %.4fcm)"),
			*Volume.Bone.ToString(), Res.X, Res.Y, Res.Z, Vox, NumTris, NBIn, NBOut, BytesPerCode * 8, StepCm);
		if (OutStats)
		{
			++OutStats->BonesBaked;
			if (bCoarsened)
			{
				OutStats->CoarsenedBones.Add({ Volume.Bone, S.VoxelSize, Vox, Res });
			}
		}
		Out.Add(MoveTemp(Volume));
	}

	UE_LOG(LogRopeSDFBake, Log, TEXT("BakeMesh done: %s -> %d bone volume(s) (of %d target bone(s))."),
		*Mesh->GetName(), Out.Num(), Targets.Num());
	return ERopeSDFBakeResult::Success;
}
