// Copyright Epic Games, Inc. All Rights Reserved.

#include "RopeSDFBaker.h"
#include "DynamicRopeEditorLog.h"
#include "Collision/SDF/RopeSDFData.h"
#include "Engine/SkeletalMesh.h"
#include "Rendering/SkeletalMeshModel.h"
#include "Rendering/SkeletalMeshLODModel.h"
#include "Async/ParallelFor.h"

// 부호 판정용 fast winding number(GeometryCore).
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

	// 메시 전체 generalized winding number로 점의 안/밖을 판정한다. "안/밖"은 몸 전체(닫힌 표면)에 대한
	// 전역 속성이라, 본별 열린 패치가 아니라 전체 메시로 winding을 봐야 강건하다(짧고 넓은 본 토막의 내부가
	// 열린 패치에선 w<0.5로 바깥 오판됨). GeometryCore fast winding(BVH + 다극 근사)으로 query당 O(log T).
	// 베이크 1회에 한 번 빌드해 voxel마다 질의한다. 빌드 후 동시(read-only) 질의 안전.
	class FRopeSDFWindingClassifier
	{
	public:
		// 삼각형 소프로 빌드(정점을 삼각형마다 복제 → 시임/비매니폴드에도 강건; winding은 연결성과 무관).
		// 삼각형 t = Positions[Indices[3t+0..2]]. 정점/질의점은 같은 좌표 공간(여기선 컴포넌트 공간)이어야 한다.
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
			// Tree는 &Mesh를, Winding은 Tree를 가리킨다. 멤버라 객체 수명 동안 주소가 안정적이어야 하므로
			// 이 분류기는 복사/이동하지 않고 제자리에서 쓴다(BakeMesh에서 지역 const로 생성).
			Tree = MakeUnique<TMeshAABBTree3<FDynamicMesh3>>(&Mesh, true);
			Winding = MakeUnique<TFastWindingTree<FDynamicMesh3>>(Tree.Get(), true);
		}

		// |generalized winding number(P)| > 0.5 이면 안쪽. 닫힌 메시 내부 |w|≈1, 바깥 ≈0의 중점이며,
		// abs라 메시 삼각형 방향(CW/CCW)과 무관하다.
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
		return ERopeSDFBakeResult::NoGeometry; // CPU 지오메트리 없음(쿡/스트립)
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

	// --- 부호 판정용: 메시 전체로 fast-winding 분류기를 한 번 빌드한다. "안/밖"은 몸 전체(닫힌 표면)에
	// 대한 전역 속성이라, 본별 열린 패치가 아니라 전체 메시로 winding을 봐야 강건하다(거리는 여전히 본별
	// 삼각형으로 재므로 본 귀속은 유지된다). 정점은 컴포넌트 공간(Verts 저장 공간) 그대로, query도 동일 공간.
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
		// 본 처리 직전에 진행 상황을 보고하고(스킵될 본 포함 — 진행률 단위는 '타깃 본'),
		// 콜백이 false를 반환하면 즉시 중단한다.
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

		// --- (3b) 가는 본 drop. AABB(확장 전 raw 살 크기) 세 변 중 가장 긴 변(=본 축)을 빼고 남은
		// 두 단면 변의 '큰 쪽'(= 중간값)이 MinBoneGirth 미만이면 사방으로 가늘다 → 굽지 않는다(drop).
		// 작은 변이 아니라 중간 변으로 보는 이유: 한 방향만 얇은 납작한 본을 catchable로 살려, 실수로
		// 떨구지 않게 보수적으로 판정. (drop은 absorb와 달리 삼각형을 부모로 넘기지 않고 그냥 제외.)
		if (S.MinBoneGirth > 0.0f)
		{
			const FVector E = Local.GetSize(); // raw 삼각형 AABB(BoundsPadding/NarrowBand 확장 전)
			const double Girth = (E.X + E.Y + E.Z)
				- FMath::Max3(E.X, E.Y, E.Z)   // 최장변(본 축) 제거
				- FMath::Min3(E.X, E.Y, E.Z);  // 최단변 제거 → 중간값만 남음
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

		// --- (4a) grid 크기 산정. 큐브 voxel; 축당 샘플 수가 MaxResolution(상한)을 넘으면 VoxelSize를 키워 맞춘다.
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
		bool bCoarsened = false; // 상한 때문에 요청 VoxelSize를 키웠는가(보고용)
		if (MaxAxis > S.MaxResolution)
		{
			Vox *= static_cast<float>(MaxAxis) / static_cast<float>(S.MaxResolution);
			Res = ResFor(Vox);
			bCoarsened = true;
		}
		Res.X = FMath::Max(Res.X, 2);
		Res.Y = FMath::Max(Res.Y, 2);
		Res.Z = FMath::Max(Res.Z, 2);

		// 간격이 정확히 Vox가 되고 샘플이 코너에 놓이도록 bounds를 스냅한다:
		// sample(x,y,z) = Min + (x,y,z) * Vox,  Max = Min + (Res - 1) * Vox.
		const FVector Min = Local.Min;
		const FVector Max = Min + FVector(Res.X - 1, Res.Y - 1, Res.Z - 1) * Vox;

		// --- (4b) voxel화. 샘플마다: unsigned 거리 = (이 본) 최소 점-삼각형 거리, 부호 = 전체 메시 fast-winding.
		// 평탄 인덱스 배치 단위로 병렬화한다(거리 삼각형·winding 트리 모두 읽기 전용이라 경쟁 없음).
		// TArray는 int32 카운트라 선형 인덱스도 int32 유지. Res는 MaxResolution으로 상한
		// (기본 48 => 48^3 ~ 110k)이라 범위 내.
		const int32 Count = Res.X * Res.Y * Res.Z;
		// 1패스: 부호 있는 거리(cm, float)를 임시로 모은다 → 본별 안쪽 밴드(NB_in)를 데이터에서 산출한 뒤
		// 2패스에서 비대칭 양자화한다(안쪽=내부 최대 깊이, 바깥=설정 감지 밴드). RawDist는 인덱스별 독립이라 병렬 안전.
		TArray<float> RawDist;
		RawDist.SetNumUninitialized(Count);

		const int32 NumTris = TriA.Num();                  // 거리(unsigned)는 이 본 삼각형으로
		const FTransform& BoneToComp = CompSpace[BoneIdx]; // 본 로컬 샘플점 → 컴포넌트 공간(분류기와 동일 프레임)

		// 평탄 인덱스(Flat = x + y*X + z*X*Y) 하나의 부호 있는 거리를 계산해 기록한다.
		auto ComputeSample = [&](int32 Flat)
		{
			const int32 x = Flat % Res.X;
			const int32 y = (Flat / Res.X) % Res.Y;
			const int32 z = Flat / (Res.X * Res.Y);
			const FVector P = Min + FVector(x, y, z) * Vox; // 본 로컬

			// 거리(unsigned): 이 본 삼각형까지의 최소 점-삼각형 거리 → 본 귀속 유지.
			float Best = BIG_NUMBER;
			for (int32 k = 0; k < NumTris; ++k)
			{
				const FVector CP = FMath::ClosestPointOnTriangleToPoint(P, TriA[k], TriB[k], TriC[k]);
				Best = FMath::Min(Best, static_cast<float>(FVector::Dist(P, CP)));
			}

			// 부호(안/밖): 전체 메시 fast-winding으로 가른다(본별 열린 패치는 짧고 넓은 토막의 내부를
			// 바깥 오판하므로 전역 메시로 봐야 강건). 샘플점을 컴포넌트 공간으로 올려 질의한다.
			const FVector Pc = BoneToComp.TransformPosition(P);
			RawDist[Flat] = WindingClassifier.IsInside(Pc) ? -Best : Best; // 안쪽 음수 / 바깥 양수
		};

		// 게임 스레드가 취소 버튼을 처리할 수 있도록 무거운 본을 여러 배치로 쪼개고, 배치 사이에서 취소를
		// 폴링한다. 배치 내부는 그대로 ParallelFor로 전 코어를 쓰며(평탄 인덱스 분할은 결과에 영향 없음),
		// 가벼운 본은 NumBatches==1이라 기존과 동일한 단일 ParallelFor가 된다.
		const int64 Work = static_cast<int64>(Count) * static_cast<int64>(FMath::Max(NumTris, 64));
		// 배치당 대략의 연산량. UI 갱신 throttle(0.2s)보다 짧게 유지해 취소가 즉각 반응하도록 작게 잡는다.
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

		// --- (4c) 본별 비대칭 밴드 확정 + uint8 양자화(2패스). 바깥 밴드(NB_out)=설정 감지 밴드,
		// 안쪽 밴드(NB_in)=이 본 내부 최대 깊이(가장 음수인 거리의 크기)로 자동 → 몸통 내부 전체가 밴드
		// 안에 들어와 깊이 박힌 노드도 최근접 표면 방향으로 회복한다. 안쪽 복셀은 이미 그리드에 존재하므로
		// 밴드를 넓혀도 복셀 수 불변(0 비용); 대가는 양자화 스텝 = (NB_in+NB_out)/255 가 커지는 것뿐이다.
		float MinD = 0.0f; // 가장 음수인 거리(내부 최대 깊이). 내부가 없으면 0 유지 → NB_in 0(바깥에 전체 코드 배분).
		for (int32 i = 0; i < Count; ++i)
		{
			MinD = FMath::Min(MinD, RawDist[i]);
		}
		const float NBIn = -FMath::Min(MinD, 0.0f);                       // >= 0 (내부 최대 깊이)
		const float NBOut = FMath::Max(S.NarrowBand, KINDA_SMALL_NUMBER); // 설정 바깥 감지 밴드(0 나눗셈 방지)

		TArray<uint8> Distances; // 양자화 코드(uint8, [-NBIn,+NBOut]→[0,255]). dequant는 볼륨 두 밴드로.
		Distances.SetNumUninitialized(Count);
		ParallelFor(Count, [&](int32 i)
		{
			// [-NBIn,+NBOut] clamp + uint8 양자화(EncodeDistance가 clamp 포함). 바깥쪽 양수(frozen FRopeContact 계약).
			Distances[i] = FRopeBoneSDFVolume::EncodeDistance(RawDist[i], NBIn, NBOut);
		});

		FRopeBoneSDFVolume Volume;
		Volume.Bone = Ref.GetBoneName(BoneIdx);
		Volume.LocalBounds = FBox(Min, Max);
		Volume.Resolution = Res;
		Volume.VoxelSize = Vox;
		Volume.NarrowBandInner = NBIn;  // dequant: 코드 0 → -NBIn (본별 자동, 내부 커버)
		Volume.NarrowBandOuter = NBOut; // dequant: 코드 255 → +NBOut (설정 감지 밴드)
		Volume.Distances = MoveTemp(Distances);
		UE_LOG(LogRopeSDFBake, Verbose, TEXT("  bone %s: res=%dx%dx%d, voxel=%.2fcm, %d tri(s), band[-%.2f,+%.2f]cm (step %.3fcm)"),
			*Volume.Bone.ToString(), Res.X, Res.Y, Res.Z, Vox, NumTris, NBIn, NBOut, (NBIn + NBOut) / 255.0f);
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
