// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeSDFBaker.h"
#include "Collision/SDF/RopeSDFData.h"
#include "Engine/SkeletalMesh.h"
#include "Rendering/SkeletalMeshModel.h"
#include "Rendering/SkeletalMeshLODModel.h"
#include "Async/ParallelFor.h"

namespace
{
	// Signed solid angle subtended by triangle (A,B,C) at the origin. The vertices must already
	// be relative to the query point. Summed over a mesh and divided by 4*PI this yields the
	// generalized winding number (Jacobson et al.): > 0.5 => the query point is inside the
	// surface. Robust on open / non-watertight patches, which a single bone's triangles are.
	double SolidAngle(const FVector& A, const FVector& B, const FVector& C)
	{
		const double la = A.Size(), lb = B.Size(), lc = C.Size();
		const double Num = FVector::DotProduct(A, FVector::CrossProduct(B, C));
		const double Den = la * lb * lc
			+ FVector::DotProduct(A, B) * lc
			+ FVector::DotProduct(B, C) * la
			+ FVector::DotProduct(C, A) * lb;
		return 2.0 * FMath::Atan2(Num, Den);
	}
}

bool FRopeSDFBaker::BakeMesh(USkeletalMesh* Mesh, const TArray<FName>& BonesIn,
	const FRopeSDFBakeSettings& S, TArray<FRopeBoneSDFVolume>& Out)
{
	Out.Reset();
	if (!Mesh)
	{
		return false;
	}

	FSkeletalMeshModel* Model = Mesh->GetImportedModel();
	if (!Model || Model->LODModels.Num() == 0)
	{
		return false; // no CPU geometry (cooked / stripped)
	}

	const FSkeletalMeshLODModel& LOD = Model->LODModels[0];
	const FReferenceSkeleton& Ref = Mesh->GetRefSkeleton();

	// --- (1) Component-space ref-pose transform per bone, by parent-chain accumulation.
	// Vertices are stored in component space at the ref pose; the inverse of this places them
	// into bone-local space, the same frame the runtime provider rebuilds via GetSocketTransform.
	const TArray<FTransform>& LocalPose = Ref.GetRefBonePose();
	TArray<FTransform> CompSpace;
	CompSpace.SetNum(LocalPose.Num());
	for (int32 b = 0; b < LocalPose.Num(); ++b)
	{
		const int32 Parent = Ref.GetParentIndex(b);
		CompSpace[b] = (Parent == INDEX_NONE) ? LocalPose[b] : LocalPose[b] * CompSpace[Parent];
	}

	// --- (2) Flat vertex list (global index order, matches the index buffer) + the owning
	// section per vertex, needed to resolve section-local influence indices through BoneMap.
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

	// Normalized skin weight of a vertex toward a skeleton bone index (scale-independent:
	// works whether InfluenceWeights are 8- or 16-bit because we divide by their sum).
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

	// --- Target bone set. Empty input => every bone that appears in a section BoneMap (skinned).
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

	for (int32 BoneIdx : Targets)
	{
		if (!CompSpace.IsValidIndex(BoneIdx))
		{
			continue;
		}
		const FTransform InvBone = CompSpace[BoneIdx].Inverse();

		// --- (3) Gather this bone's triangles in bone-local space + their AABB.
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
			continue; // no skin for this bone
		}

		// --- (4a) Grid sizing. Cubic voxels; raise VoxelSize if the band would exceed the cap.
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
		if (MaxAxis > S.MaxResolution)
		{
			Vox *= static_cast<float>(MaxAxis) / static_cast<float>(S.MaxResolution);
			Res = ResFor(Vox);
		}
		Res.X = FMath::Max(Res.X, 2);
		Res.Y = FMath::Max(Res.Y, 2);
		Res.Z = FMath::Max(Res.Z, 2);

		// Snap bounds so spacing is EXACTLY Vox and samples sit on corners:
		// sample(x,y,z) = Min + (x,y,z) * Vox,  Max = Min + (Res - 1) * Vox.
		const FVector Min = Local.Min;
		const FVector Max = Min + FVector(Res.X - 1, Res.Y - 1, Res.Z - 1) * Vox;

		// --- (4b) Voxelize. Per sample: unsigned distance = min point-triangle distance,
		// sign from the winding number. Parallelized over Z slices (read-only triangle soup).
		// TArray is int32-counted, so the linear index stays int32. Res is capped by MaxResolution
		// (default 48 => 48^3 ~ 110k), well within range.
		const int32 Count = Res.X * Res.Y * Res.Z;
		TArray<float> Distances;
		Distances.SetNumUninitialized(Count);

		const int32 NumTris = TriA.Num();
		ParallelFor(Res.Z, [&](int32 z)
		{
			for (int32 y = 0; y < Res.Y; ++y)
			{
				for (int32 x = 0; x < Res.X; ++x)
				{
					const FVector P = Min + FVector(x, y, z) * Vox;
					float Best = BIG_NUMBER;
					double Omega = 0.0;
					for (int32 k = 0; k < NumTris; ++k)
					{
						const FVector CP = FMath::ClosestPointOnTriangleToPoint(P, TriA[k], TriB[k], TriC[k]);
						Best = FMath::Min(Best, static_cast<float>(FVector::Dist(P, CP)));
						Omega += SolidAngle(TriA[k] - P, TriB[k] - P, TriC[k] - P);
					}
					const bool bInside = (Omega / (4.0 * PI)) > 0.5;
					float D = bInside ? -Best : Best;            // outside-positive, per the frozen FRopeContact contract
					D = FMath::Clamp(D, -S.NarrowBand, S.NarrowBand);
					Distances[x + y * Res.X + z * Res.X * Res.Y] = D;
				}
			}
		});

		FRopeBoneSDFVolume Volume;
		Volume.Bone = Ref.GetBoneName(BoneIdx);
		Volume.LocalBounds = FBox(Min, Max);
		Volume.Resolution = Res;
		Volume.VoxelSize = Vox;
		Volume.Distances = MoveTemp(Distances);
		Out.Add(MoveTemp(Volume));
	}

	return true;
}
