// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/RopeBodyColliderExtraction.h"
#include "PhysicsEngine/BodySetup.h"
// FKConvexElem::GetPlanes(월드 평면 추출)
#include "PhysicsEngine/ConvexElem.h"

namespace
{
	// elem 로컬 → 월드 변환 행렬. UE 컨벡스 규약(FKConvexElem::CalcAABB): WT = ElemTM * Scale * Comp(무스케일).
	// 스케일을 회전/이동과 분리해 매트릭스로 합성한다(FTransform은 전단 표현 불가) — 비균등 스케일 × 회전에서도
	// 평면이 정확히 변환된다.
	FMatrix ComposeConvexToWorld(const FTransform& ElemTM, const FVector& Scale3D, const FTransform& CompTM)
	{
		FTransform CompNoScale = CompTM;
		CompNoScale.SetScale3D(FVector::OneVector);
		return ElemTM.ToMatrixWithScale() * FScaleMatrix(Scale3D) * CompNoScale.ToMatrixWithScale();
	}

	// 바디-로컬 변환(elem + 스케일, 컴포넌트 강체 rot/trans 제외). 월드 = 바디로컬 ∘ 강체(컴포넌트 rot/trans).
	// 강체만 프레임 간 움직이므로(스케일 불변 가정) 바디-로컬 평면은 불변 → 동적 바디의 sub-포즈 강체 보간용.
	FMatrix ComposeConvexBodyLocal(const FTransform& ElemTM, const FVector& Scale3D)
	{
		return ElemTM.ToMatrixWithScale() * FScaleMatrix(Scale3D);
	}

	// 로컬 평면 집합을 월드로 변환 + 정규화(단위 법선·바깥). FPlane::TransformBy가 역전치로 법선을 올바르게
	// 변환하므로 전단에서도 정확 — 단 길이가 변하므로 (N,W)를 |N|으로 나눠 정규화한다.
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

	// 로컬 박스(중심 원점, 반폭 Half)의 6평면(바깥 법선 ±축). 전단 박스를 컨벡스로 정확히 라우팅할 때 쓴다.
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
		TArray<FRopeConvexCollider>& OutConvexes, const TFunctionRef<void(int32)>& OnConvexFallback)
	{
		const FVector Scale3D = CompTM.GetScale3D();
		const auto BudgetLeft = [&]() { return OutBoxes.Num() + OutCapsules.Num() + OutConvexes.Num() < MaxColliders; };

		// sphyl: 스킨 캡슐 provider와 동일한 스케일 규약(GetScaledRadius/CylinderLength). 동적이면 prev 끝점도
		// 채워 FCapsuleCollider의 표면 속도/substep CCD machinery를 그대로 탄다(셰이더/GPU 변경 불필요).
		for (const FKSphylElem& Sphyl : Setup.AggGeom.SphylElems)
		{
			if (!BudgetLeft())
			{
				return false;
			}
			const FTransform ElemTM = Sphyl.GetTransform() * CompTM;
			// sphyl 축 = 로컬 Z.
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
			OutCapsules.Add(MoveTemp(Cap));
		}

		// sphere: A==B 축퇴 캡슐.
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
			OutCapsules.Add(MoveTemp(Cap));
		}

		// box: 해석적 OBB — 모서리 정확 처리의 본체. X/Y/Z는 전체 길이. 전단(비균등 스케일 × 회전 elem)만
		// 6평면 컨벡스로 정확히 라우팅한다(OBB로는 표현 불가). 나머지는 정확한 OBB.
		for (const FKBoxElem& Box : Setup.AggGeom.BoxElems)
		{
			if (!BudgetLeft())
			{
				return false;
			}
			// elem 공간 반폭(스케일 전).
			const FVector HalfLocal(Box.X * 0.5, Box.Y * 0.5, Box.Z * 0.5);
			const FVector AbsScale = Scale3D.GetAbs();
			const bool bUniform = FMath::IsNearlyEqual(AbsScale.GetMax(), AbsScale.GetMin(), UE_KINDA_SMALL_NUMBER);
			if (bUniform || Box.Rotation.IsNearlyZero())
			{
				// 정확 OBB: 균등 스케일(전 축 동일) 또는 elem 회전 identity(컴포넌트 축 정렬 → 축별 스케일 정확).
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
				OutBoxes.Add(MoveTemp(BoxCol));
			}
			else
			{
				// 전단: 6평면 컨벡스로 정확히(평면은 전단 행렬로도 정확 변환). 바디-로컬 평면 + 컴포넌트 강체로
				// 저장해 동적(움직이는 전단 박스)도 지원. M1의 min-scale 근사를 대체.
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
					OutConvexes.Add(MoveTemp(Cv));
				}
			}
		}

		// convex: Chaos convex의 평면 집합을 월드로 변환해 해석적 컨벡스 collider로. 미쿡(빈 평면)이거나
		// 평면 과다(>상한)면 ElemBox를 OBB 근사로 폴백해 충돌을 통째로 잃지 않는다.
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
				// elem -> 바디로컬(강체 제외).
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
					OutConvexes.Add(MoveTemp(Cv));
					continue;
				}
			}

			// 폴백: ElemBox OBB 근사(미쿡/평면 과다/무효). 거칠지만 충돌 유지 > 통째 누락. 정적으로 처리(드문 경로).
			if (Convex.ElemBox.IsValid)
			{
				const FMatrix M = ComposeConvexToWorld(Convex.GetTransform(), Scale3D, CompTM);
				const FVector CenterW = M.TransformPosition(Convex.ElemBox.GetCenter());
				const FQuat   RotW = M.GetMatrixWithoutScale().ToQuat();
				const FVector HalfW = Convex.ElemBox.GetExtent() * static_cast<float>(Scale3D.GetAbsMin());
				OutBoxes.Add(FRopeBoxCollider(CenterW, RotW, HalfW));
				OnConvexFallback(ElemPlanes.Num());
			}
		}

		return true;
	}
}
