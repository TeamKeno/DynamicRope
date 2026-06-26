// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeSDFBaker.h"
#include "DynamicRopeEditorLog.h"
#include "Collision/SDF/RopeSDFData.h"
#include "Engine/SkeletalMesh.h"
#include "Rendering/SkeletalMeshModel.h"
#include "Rendering/SkeletalMeshLODModel.h"
#include "Async/ParallelFor.h"

namespace
{
	// 삼각형 (A,B,C)이 원점에서 두르는 부호 있는 입체각. 정점은 이미 query점 기준 상대좌표여야 한다.
	// 메시 전체에 대해 합산한 뒤 4*PI로 나누면 generalized winding number(Jacobson et al.)가 되며,
	// > 0.5 이면 query점이 표면 안쪽. 닫히지 않은(non-watertight) 패치 — 본 하나의 삼각형 — 에서도 강건하다.
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
		UE_LOG(LogRopeSDFBake, Warning, TEXT("BakeMesh aborted: null mesh."));
		return false;
	}

	FSkeletalMeshModel* Model = Mesh->GetImportedModel();
	if (!Model || Model->LODModels.Num() == 0)
	{
		UE_LOG(LogRopeSDFBake, Warning, TEXT("BakeMesh aborted: %s has no CPU geometry (cooked/stripped)."), *Mesh->GetName());
		return false; // CPU 지오메트리 없음(쿡/스트립)
	}

	UE_LOG(LogRopeSDFBake, Log, TEXT("BakeMesh start: %s (voxel=%.2fcm, maxRes=%d, narrowBand=%.1fcm)"),
		*Mesh->GetName(), S.VoxelSize, S.MaxResolution, S.NarrowBand);

	const FSkeletalMeshLODModel& LOD = Model->LODModels[0];
	const FReferenceSkeleton& Ref = Mesh->GetRefSkeleton();

	// --- (1) 본별 컴포넌트 공간 ref-pose 트랜스폼을 부모 체인 누적으로 구한다.
	// 정점은 ref 포즈의 컴포넌트 공간에 저장돼 있다. 이 트랜스폼의 역이 정점을 본 로컬 공간으로 옮기며,
	// 런타임 provider가 GetSocketTransform으로 재구성하는 프레임과 동일하다.
	const TArray<FTransform>& LocalPose = Ref.GetRefBonePose();
	TArray<FTransform> CompSpace;
	CompSpace.SetNum(LocalPose.Num());
	for (int32 b = 0; b < LocalPose.Num(); ++b)
	{
		const int32 Parent = Ref.GetParentIndex(b);
		CompSpace[b] = (Parent == INDEX_NONE) ? LocalPose[b] : LocalPose[b] * CompSpace[Parent];
	}

	// --- (2) 평탄 정점 리스트(전역 인덱스 순, 인덱스 버퍼와 일치) + 정점별 소속 섹션.
	// 섹션 로컬 influence 인덱스를 BoneMap을 거쳐 스켈레톤 본 인덱스로 풀려면 소속 섹션이 필요하다.
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

	// 정점이 특정 스켈레톤 본에 대해 갖는 정규화 스킨 가중치(스케일 무관: 가중치 합으로 나누므로
	// InfluenceWeights가 8비트든 16비트든 동작).
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

	// --- 타깃 본 집합. 입력이 비면 => 섹션 BoneMap에 등장하는 모든 본(= 스킨된 본).
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

		// --- (3) 이 본의 삼각형을 본 로컬 공간으로 모으고 AABB도 함께 키운다.
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
			continue; // 이 본에 귀속된 스킨 없음
		}

		// --- (4a) grid 크기 산정. 큐브 voxel; 밴드가 상한을 넘으면 VoxelSize를 키운다.
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

		// 간격이 정확히 Vox가 되고 샘플이 코너에 놓이도록 bounds를 스냅한다:
		// sample(x,y,z) = Min + (x,y,z) * Vox,  Max = Min + (Res - 1) * Vox.
		const FVector Min = Local.Min;
		const FVector Max = Min + FVector(Res.X - 1, Res.Y - 1, Res.Z - 1) * Vox;

		// --- (4b) voxel화. 샘플마다: unsigned 거리 = 최소 점-삼각형 거리, 부호 = winding number.
		// Z 슬라이스로 병렬화(삼각형 수프는 읽기 전용이라 경쟁 없음).
		// TArray는 int32 카운트라 선형 인덱스도 int32 유지. Res는 MaxResolution으로 상한
		// (기본 48 => 48^3 ~ 110k)이라 범위 내.
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
					float D = bInside ? -Best : Best;            // 바깥쪽 양수(frozen FRopeContact 계약)
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
		UE_LOG(LogRopeSDFBake, Verbose, TEXT("  bone %s: res=%dx%dx%d, voxel=%.2fcm, %d tri(s)"),
			*Volume.Bone.ToString(), Res.X, Res.Y, Res.Z, Vox, NumTris);
		Out.Add(MoveTemp(Volume));
	}

	UE_LOG(LogRopeSDFBake, Log, TEXT("BakeMesh done: %s -> %d bone volume(s) (of %d target bone(s))."),
		*Mesh->GetName(), Out.Num(), Targets.Num());
	return true;
}
