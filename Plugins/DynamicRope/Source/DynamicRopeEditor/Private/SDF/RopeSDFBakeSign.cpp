// Copyright Epic Games, Inc. All Rights Reserved.

#include "SDF/RopeSDFBakeSign.h"

#include "DynamicMesh/DynamicMesh3.h"
#include "IndexTypes.h"
#include "Spatial/MeshAABBTree3.h"
#include "Spatial/FastWinding.h"

using UE::Geometry::FDynamicMesh3;
using UE::Geometry::TMeshAABBTree3;
using UE::Geometry::TFastWindingTree;
using UE::Geometry::FIndex3i;

// FDynamicMesh3용 AABB 트리 별칭(엔진 버전에 따라 FDynamicMeshAABBTree3 typedef 유무가 달라 직접 명시).
using FRopeAABBTree = TMeshAABBTree3<FDynamicMesh3>;

struct FRopeSDFWindingClassifier::FImpl
{
	FDynamicMesh3 Mesh;
	TUniquePtr<FRopeAABBTree> Tree;
	TUniquePtr<TFastWindingTree<FDynamicMesh3>> Winding; // 템플릿 인자는 메시 타입; 생성자는 AABB 트리 포인터를 받음
};

FRopeSDFWindingClassifier::FRopeSDFWindingClassifier(
	TConstArrayView<FVector3f> Positions, TConstArrayView<uint32> Indices)
{
	if (Positions.Num() == 0 || Indices.Num() < 3)
	{
		return;
	}

	TUniquePtr<FImpl> Local = MakeUnique<FImpl>();
	FDynamicMesh3& Mesh = Local->Mesh;

	// 삼각형 소프로 적재(정점을 삼각형마다 복제). 시임/중복 정점이 만드는 비매니폴드 엣지에서
	// AppendTriangle이 거부되어 삼각형이 누락되는 일을 막는다 — winding number는 연결성과 무관.
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
		return; // 유효 삼각형 없음 → bValid=false 유지.
	}

	// AABB 트리 + fast winding 트리를 빌드(둘 다 auto-build). Tree는 &Mesh를, Winding은 Tree를 가리키므로
	// FImpl이 셋을 함께 소유한다(TUniquePtr 이동은 힙 객체 주소를 바꾸지 않아 포인터가 계속 유효).
	Local->Tree = MakeUnique<FRopeAABBTree>(&Mesh, true);
	Local->Winding = MakeUnique<TFastWindingTree<FDynamicMesh3>>(Local->Tree.Get(), true);

	Impl = MoveTemp(Local);
	bValid = true;
}

FRopeSDFWindingClassifier::~FRopeSDFWindingClassifier() = default;

bool FRopeSDFWindingClassifier::IsInside(const FVector& P) const
{
	if (!bValid)
	{
		return false;
	}
	const double W = Impl->Winding->FastWindingNumber(FVector3d(P.X, P.Y, P.Z));
	return FMath::Abs(W) > 0.5;
}
