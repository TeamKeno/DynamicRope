// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/RopeBoneCapsuleProvider.h"
#include "DynamicRopeLog.h"
#include "Subsystem/RopeSimSubsystem.h"
#include "Components/SkeletalMeshComponent.h"
#include "Engine/SkinnedAsset.h"
#include "GameFramework/Actor.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "PhysicsEngine/SkeletalBodySetup.h"
#include "ReferenceSkeleton.h"

URopeBoneCapsuleProvider::URopeBoneCapsuleProvider()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void URopeBoneCapsuleProvider::BeginPlay()
{
	Super::BeginPlay();
	if (URopeSimSubsystem* Sim = URopeSimSubsystem::Get(GetWorld()))
	{
		Sim->RegisterColliderProvider(this);
	}
}

void URopeBoneCapsuleProvider::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (URopeSimSubsystem* Sim = URopeSimSubsystem::Get(GetWorld()))
	{
		Sim->UnregisterColliderProvider(this);
	}
	Super::EndPlay(EndPlayReason);
}

USkeletalMeshComponent* URopeBoneCapsuleProvider::ResolveMesh()
{
	if (!SkeletalMesh)
	{
		if (AActor* Owner = GetOwner())
		{
			SkeletalMesh = Owner->FindComponentByClass<USkeletalMeshComponent>();
		}
	}
	return SkeletalMesh;
}

void URopeBoneCapsuleProvider::GatherColliders(FRopeColliderGatherContext& Gather)
{
	USkeletalMeshComponent* Mesh = ResolveMesh();
	if (!Mesh)
	{
		UE_LOG(LogRopeCollision, Verbose, TEXT("CapsuleProvider on %s: no skeletal mesh resolved — no colliders."),
			*GetNameSafe(GetOwner()));
		return;
	}

	// 프레임당 1회만 빌드(디둡): 같은 메시를 잡는 여러 로프가 호출해도 capsule을 재구성하지 않는다.
	// region별 배정은 아래 MapCollidersToRegionsByBounds가 만든다(빌드는 region 무관 — 전 본 빌드).
	const uint64 Frame = GFrameCounter;
	if (BuiltFrame != Frame)
	{
		BuiltFrame = Frame;
		Capsules.Reset();
		BuildCapsules(Mesh);

		// 표면 속도(드래그) 산출용 프레임 dt. 캡슐별 (현재-이전 끝점)/dt 로 콜라이더가 표면 속도를 만든다.
		const float FrameDt = GetWorld() ? GetWorld()->GetDeltaSeconds() : 0.0f;
		const float InvDt = (FrameDt > KINDA_SMALL_NUMBER) ? (1.0f / FrameDt) : 0.0f;

		// 이전 프레임 끝점을 인덱스 정렬로 이어 붙인다. 개수 불일치(첫 프레임/구성 변경)면 이 프레임은
		// 정적(InvDt 0 = 속도 0) 취급 — 생성자가 이미 prev=현재로 초기화해 둔 상태 그대로.
		if (PrevEndpoints.Num() == Capsules.Num())
		{
			for (int32 i = 0; i < Capsules.Num(); ++i)
			{
				Capsules[i].PrevA = PrevEndpoints[i].Key;
				Capsules[i].PrevB = PrevEndpoints[i].Value;
				Capsules[i].InvDeltaTime = InvDt;
			}
		}

		// 다음 프레임용으로 현재 끝점 저장.
		PrevEndpoints.Reset(Capsules.Num());
		for (const FCapsuleCollider& Cap : Capsules)
		{
			PrevEndpoints.Emplace(Cap.A, Cap.B);
		}
	}

	// 캐시된 capsule 포인터를 넘긴다(해당 프레임 동안 유효).
	const int32 StartIndex = Gather.Colliders.Num();
	Gather.Colliders.Reserve(StartIndex + Capsules.Num());
	for (FCapsuleCollider& Cap : Capsules)
	{
		Gather.Colliders.Add(&Cap);
	}

	// region 매핑: 메시(캡슐 유니언) 선-거절 → 걸린 로프만 캡슐별 bounds 배정. 원거리 로프는 메시당
	// 비교 1회로 끝난다 — 서브시스템의 로프별 풀 전체 재-컬(O(로프×풀))을 대체하는 부분.
	RopeColliderGather::MapCollidersToRegionsByBounds(Gather, StartIndex);
}

void URopeBoneCapsuleProvider::BuildCapsules(USkeletalMeshComponent* Mesh)
{
	// 1) 명시 목록: 나열된 본마다 본→부모 세그먼트 캡슐(반지름 = CapsuleRadius). 기존 동작.
	if (Bones.Num() > 0)
	{
		for (const FName& Bone : Bones)
		{
			if (Bone.IsNone())
			{
				continue;
			}
			const FName    Parent = Mesh->GetParentBone(Bone);
			const FVector  P0 = Mesh->GetSocketTransform(Bone).GetLocation();
			const FVector  P1 = Parent.IsNone() ? P0 : Mesh->GetSocketTransform(Parent).GetLocation();
			Capsules.Add(FCapsuleCollider(P0, P1, CapsuleRadius, Bone, Mesh));
		}
		UE_LOG(LogRopeCollision, VeryVerbose, TEXT("CapsuleProvider on %s: built %d capsule(s) from %d listed bone(s)."),
			*GetNameSafe(GetOwner()), Capsules.Num(), Bones.Num());
		return;
	}

	// 2) 자동 — Physics Asset: 바디 셰이프를 캡슐로 쓴다. sphyl은 그대로, sphere는 A==B 축퇴 캡슐,
	// box는 장축 정렬 캡슐 근사(축 = 최장변, 반지름 = 나머지 두 반폭의 최대 — 단면 모서리만 살짝
	// 초과 커버). 본별 실제 치수를 얻고, 스킨 없는 IK/트위스트 본의 가짜 세그먼트도 자연히 배제된다.
	// convex 등 나머지 셰이프는 건너뛴다 — 그래서 셰이프를 하나도 못 만들면 아래 스켈레톤 폴백으로
	// 진행한다(convex 전용 PA에서 충돌이 통째로 사라지는 것 방지).
	if (const UPhysicsAsset* PhysAsset = Mesh->GetPhysicsAsset())
	{
		for (const TObjectPtr<USkeletalBodySetup>& Setup : PhysAsset->SkeletalBodySetups)
		{
			if (!Setup)
			{
				continue;
			}
			const FName BoneName = Setup->BoneName;
			const int32 BoneIndex = Mesh->GetBoneIndex(BoneName);
			if (BoneIndex == INDEX_NONE)
			{
				continue; // 에셋에만 있고 현재 메시에 없는 본.
			}
			const FTransform BoneTM = Mesh->GetBoneTransform(BoneIndex);
			const FVector Scale3D = BoneTM.GetScale3D();

			for (const FKSphylElem& Sphyl : Setup->AggGeom.SphylElems)
			{
				const FTransform ElemTM = Sphyl.GetTransform() * BoneTM;
				const FVector Axis = ElemTM.GetUnitAxis(EAxis::Z); // sphyl 축 = 로컬 Z
				const FVector Center = ElemTM.GetLocation();
				const float HalfLen = Sphyl.GetScaledCylinderLength(Scale3D) * 0.5f;
				Capsules.Add(FCapsuleCollider(Center + Axis * HalfLen, Center - Axis * HalfLen,
					Sphyl.GetScaledRadius(Scale3D), BoneName, Mesh));
			}
			for (const FKSphereElem& Sphere : Setup->AggGeom.SphereElems)
			{
				const FVector Center = BoneTM.TransformPosition(Sphere.Center);
				const float ScaledRadius = Sphere.Radius * static_cast<float>(Scale3D.GetAbsMin());
				Capsules.Add(FCapsuleCollider(Center, Center, ScaledRadius, BoneName, Mesh));
			}
			for (const FKBoxElem& Box : Setup->AggGeom.BoxElems)
			{
				// X/Y/Z는 전체 길이. 최장변을 캡슐 축으로, 나머지 두 반폭의 최대(= 세 반폭의 중간값)를
				// 반지름으로 — 단면 직사각형의 긴 변까지 덮는다(모서리만 살짝 초과). 세그먼트 반길이 =
				// 최장 반폭 - 반지름(반구가 상자 끝을 안 넘게, 음수면 0 = 구).
				const double UniformScale = Scale3D.GetAbsMin();
				const double Hx = Box.X * 0.5 * UniformScale;
				const double Hy = Box.Y * 0.5 * UniformScale;
				const double Hz = Box.Z * 0.5 * UniformScale;
				const double LongHalf = FMath::Max3(Hx, Hy, Hz);
				const double MidHalf = Hx + Hy + Hz - LongHalf - FMath::Min3(Hx, Hy, Hz);
				const EAxis::Type LongAxis = (Hx >= Hy && Hx >= Hz) ? EAxis::X : (Hy >= Hz) ? EAxis::Y : EAxis::Z;
				const float SegHalf = static_cast<float>(FMath::Max(LongHalf - MidHalf, 0.0));
				const FTransform ElemTM = Box.GetTransform() * BoneTM;
				const FVector Axis = ElemTM.GetUnitAxis(LongAxis);
				const FVector Center = ElemTM.GetLocation();
				Capsules.Add(FCapsuleCollider(Center + Axis * SegHalf, Center - Axis * SegHalf,
					static_cast<float>(MidHalf), BoneName, Mesh));
			}
		}
		if (Capsules.Num() > 0)
		{
			UE_LOG(LogRopeCollision, VeryVerbose, TEXT("CapsuleProvider on %s: built %d capsule(s) from physics asset %s."),
				*GetNameSafe(GetOwner()), Capsules.Num(), *GetNameSafe(PhysAsset));
			return;
		}
		UE_LOG(LogRopeCollision, Verbose, TEXT("CapsuleProvider on %s: physics asset %s has no usable shapes — falling back to skeleton."),
			*GetNameSafe(GetOwner()), *GetNameSafe(PhysAsset));
	}

	// 3) 자동 — 스켈레톤 폴백(Physics Asset 없음): 모든 본-부모 세그먼트(반지름 = CapsuleRadius).
	// AutoMinBoneLength 미만은 제외(손가락/트위스트 잡음 컷). 스킨 없는 본(IK 등)의 가짜 세그먼트가
	// 섞일 수 있다 — 러프한 테스트용이며 캐릭터는 Physics Asset을 두는 쪽을 권장.
	const USkinnedAsset* Asset = Mesh->GetSkinnedAsset();
	if (!Asset)
	{
		return;
	}
	const FReferenceSkeleton& RefSkel = Asset->GetRefSkeleton();
	const int32 NumBones = Mesh->GetNumBones();
	const float MinLenSq = FMath::Square(FMath::Max(AutoMinBoneLength, 0.0f));
	for (int32 i = 0; i < NumBones; ++i)
	{
		const int32 ParentIndex = RefSkel.GetParentIndex(i);
		if (ParentIndex == INDEX_NONE)
		{
			continue; // 루트: 부모 세그먼트 없음.
		}
		const FVector P0 = Mesh->GetBoneTransform(i).GetLocation();
		const FVector P1 = Mesh->GetBoneTransform(ParentIndex).GetLocation();
		if (FVector::DistSquared(P0, P1) < MinLenSq)
		{
			continue;
		}
		Capsules.Add(FCapsuleCollider(P0, P1, CapsuleRadius, Mesh->GetBoneName(i), Mesh));
	}
	UE_LOG(LogRopeCollision, VeryVerbose, TEXT("CapsuleProvider on %s: built %d capsule(s) from skeleton fallback (%d bone(s), no physics asset)."),
		*GetNameSafe(GetOwner()), Capsules.Num(), NumBones);
}
