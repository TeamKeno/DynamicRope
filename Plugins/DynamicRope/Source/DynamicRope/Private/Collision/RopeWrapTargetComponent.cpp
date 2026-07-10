// Copyright Epic Games, Inc. All Rights Reserved.

#include "Collision/RopeWrapTargetComponent.h"
#include "Subsystem/RopeSimSubsystem.h"
#include "DynamicRopeLog.h"                 // LogRopeCollision(진단 로그)
#include "Components/StaticMeshComponent.h"
#include "Components/PrimitiveComponent.h" // GetBodySetup(심플 콜리전 추출)
#include "PhysicsEngine/BodySetup.h"       // UBodySetup / FKAggregateGeom(sphyl/box/sphere)
#include "GameFramework/Actor.h"
#include "Engine/World.h"

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

	// 진단: 등록됐는지 + 대상 컴포넌트/가상 본이 해석됐는지 1회 로그.
	USceneComponent* Comp = ResolveTarget();
	UE_LOG(LogRopeCollision, Log,
		TEXT("[WrapTarget] BeginPlay actor=%s: subsystem=%s, target=%s, virtualBone=%s"),
		*GetNameSafe(GetOwner()),
		Sim ? TEXT("OK") : TEXT("NULL(등록 실패)"),
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
			// 기본: 첫 스태틱 메시(기둥 본체), 없으면 루트 컴포넌트.
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
		// 합성(가상) 본 이름: 지정값 우선, 없으면 컴포넌트 이름 기반으로 안정적 발급.
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

	// 우선 대상의 저작 심플 콜리전(sphyl/box/sphere)에서 캡슐을 뽑는다 — 시각 메시에 타이트해 로프가
	// 표면에서 뜨지 않는다. 심플 콜리전이 없으면(또는 convex뿐이면) 로컬 bounds 근사로 폴백한다.
	bUsedSimpleCollision = BuildCapsuleFromSimpleCollision(Comp, WorldA, WorldB, CapRadius);
	if (!bUsedSimpleCollision)
	{
		BuildCapsuleFromBounds(Comp, WorldA, WorldB, CapRadius);
	}

	// 디자이너 반지름 override: 형상/축(끝점)은 유지하고 두께만 바꾼다.
	if (Radius > 0.0f)
	{
		CapRadius = Radius;
	}

	// 랩 가능 캡슐: 가상 본 + SourceMesh=대상 컴포넌트. IsWorldStatic()=false(FCapsuleCollider 기본)라
	// detect 파이프라인에 포함된다 — 정적 월드 push-out 콜라이더(FRopeStaticCapsuleCollider)와 대조.
	Capsule = FCapsuleCollider(WorldA, WorldB, CapRadius, ResolvedBone, Comp);

	// 무버블 프롭 표면 속도: (현재-이전 끝점)/dt. 정적이면 사실상 0(=기존 동작). 첫 프레임은 prev 없음 → 0.
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

	// 우선순위: sphyl(캡슐) → box → sphere. 같은 타입은 가장 큰 것(주 몸체)을 고른다. convex뿐이면 실패
	// 반환 → bounds 폴백. 스케일 규약은 본 캡슐/정적 provider와 동일(GetScaled*).

	// 1) sphyl — 가장 긴 것(원기둥 기둥은 여기서 반지름이 정확히 맞아 뜸이 사라진다).
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
			const FVector AxisDir = ElemTM.GetUnitAxis(EAxis::Z); // sphyl 축 = 로컬 Z
			const FVector Center = ElemTM.GetLocation();
			const float HalfLen = Best->GetScaledCylinderLength(Scale) * 0.5f;
			OutA = Center + AxisDir * HalfLen;
			OutB = Center - AxisDir * HalfLen;
			OutRadius = Best->GetScaledRadius(Scale);
			return true;
		}
	}

	// 2) box → 캡슐(장축 정렬, 반지름 = 나머지 두 반폭의 최대) — 본 캡슐 provider와 동일 근사.
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

	// 3) sphere — A==B 축퇴 캡슐(구).
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

	return false; // sphyl/box/sphere 없음(예: convex 전용) → 호출자가 bounds 폴백.
}

void URopeWrapTargetComponent::BuildCapsuleFromBounds(USceneComponent* Comp,
	FVector& OutA, FVector& OutB, float& OutRadius) const
{
	const FTransform CompTM = Comp->GetComponentTransform();

	// 로컬 공간 축정렬 bounds(LocalToWorld=Identity → 로컬 반폭/중심).
	const FBoxSphereBounds LocalBounds = Comp->CalcBounds(FTransform::Identity);
	const FVector Ext = LocalBounds.BoxExtent;      // 로컬 반폭
	const FVector LocalCenter = LocalBounds.Origin; // 로컬 중심

	// 장축 인덱스 결정: 자동(최장 반폭) 또는 지정.
	int32 AxisIdx;
	if (bAutoAxis)
	{
		AxisIdx = (Ext.X >= Ext.Y && Ext.X >= Ext.Z) ? 0 : (Ext.Y >= Ext.Z) ? 1 : 2;
	}
	else
	{
		AxisIdx = (Axis == ERopeWrapAxis::X) ? 0 : (Axis == ERopeWrapAxis::Y) ? 1 : 2;
	}

	// 반지름: 나머지 두 반폭의 최대(축정렬 단면을 덮는 캡슐 근사 — 사각 단면 모서리만 살짝 초과).
	double OtherMax = 0.0;
	for (int32 k = 0; k < 3; ++k)
	{
		if (k != AxisIdx)
		{
			OtherMax = FMath::Max(OtherMax, static_cast<double>(Ext[k]));
		}
	}
	OutRadius = static_cast<float>(FMath::Max(OtherMax, 1.0));

	// 세그먼트 반길이 = 장축 반폭 - 반지름(반구가 끝을 넘지 않게; 음수면 0 = 구).
	const float SegHalf = static_cast<float>(FMath::Max(static_cast<double>(Ext[AxisIdx]) - OutRadius, 0.0));

	FVector LocalAxis = FVector::ZeroVector;
	LocalAxis[AxisIdx] = 1.0;
	OutA = CompTM.TransformPosition(LocalCenter + LocalAxis * SegHalf);
	OutB = CompTM.TransformPosition(LocalCenter - LocalAxis * SegHalf);
}

void URopeWrapTargetComponent::BuildBox(USceneComponent* Comp)
{
	const FTransform CompTM = Comp->GetComponentTransform();
	const FVector Scale = CompTM.GetScale3D().GetAbs();

	FVector WorldCenter = FVector::ZeroVector;
	FVector HalfExtents = FVector::ZeroVector;
	FQuat   Rot = FQuat::Identity;

	// 1) 심플 콜리전의 가장 큰 박스 elem → 타이트 OBB(저작 형상에 정확히 맞음).
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
		HalfExtents = FVector(Best->X, Best->Y, Best->Z) * 0.5 * Scale; // X/Y/Z=전체 길이 → 반폭×스케일
	}
	else
	{
		// 2) 폴백: 컴포넌트 로컬 bounds OBB(박스 심플 콜리전이 없을 때).
		const FBoxSphereBounds LocalBounds = Comp->CalcBounds(FTransform::Identity);
		WorldCenter = CompTM.TransformPosition(LocalBounds.Origin);
		Rot = CompTM.GetRotation();
		HalfExtents = LocalBounds.BoxExtent * Scale;
	}

	// 가상 본 + SourceMesh → IsWorldStatic()=false → 감지 참여(랩 대상). v1은 정적 가정(InvDt=0, 표면 속도 0).
	Box = FRopeBoxCollider(WorldCenter, Rot, HalfExtents);
	Box.Bone = ResolvedBone;
	Box.SourceMesh = Comp;
}

bool URopeWrapTargetComponent::EffectiveServeBox(USceneComponent* Comp) const
{
	if (Shape == ERopeWrapShape::Box)     { return true; }
	if (Shape == ERopeWrapShape::Capsule) { return false; }

	// Auto: 심플 콜리전의 지배(최대) 프리미티브가 박스면 Box, sphyl/sphere면 Capsule.
	UPrimitiveComponent* Prim = Cast<UPrimitiveComponent>(Comp);
	UBodySetup* Setup = Prim ? Prim->GetBodySetup() : nullptr;
	if (!Setup)
	{
		return false; // 심플 콜리전 없음 → 캡슐(bounds 폴백).
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

	// 박스가 있고 캡슐형(sphyl/sphere)보다 크거나 같으면 Box. 박스가 없으면 캡슐.
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
				TEXT("[WrapTarget] %s: 대상 컴포넌트 없음 — 랩 캡슐 미생성(TargetComponent 지정 또는 스태틱 메시 필요)."),
				*GetNameSafe(GetOwner()));
		}
		return;
	}

	// 프레임당 1회만 빌드(디둡): 같은 대상을 노리는 여러 로프가 호출해도 캡슐을 재구성하지 않는다.
	const uint64 Frame = GFrameCounter;
	if (BuiltFrame != Frame)
	{
		BuiltFrame = Frame;
		bServeBox = EffectiveServeBox(Comp); // Auto면 심플 콜리전으로 셰이프 결정.
		if (bServeBox)
		{
			BuildBox(Comp);
		}
		else
		{
			BuildCapsule(Comp);
		}

		// 진단: 첫 빌드 시 셰이프/형상·본을 1회 로그(이 로그가 안 뜨면 GatherColliders 미호출 =
		// provider 미등록 또는 로프 소유자 제외(같은 액터) 또는 근처 로프 없음).
		if (!bDiagnosticsLogged)
		{
			bDiagnosticsLogged = true;
			const FBoxSphereBounds DiagBounds = Comp->CalcBounds(FTransform::Identity);
			if (bServeBox)
			{
				UE_LOG(LogRopeCollision, Log,
					TEXT("[WrapTarget] %s: 랩 박스 생성 localExtent=%s | center=%s half=%s bone=%s"),
					*GetNameSafe(GetOwner()), *DiagBounds.BoxExtent.ToCompactString(),
					*Box.Center.ToCompactString(), *Box.HalfExtents.ToCompactString(), *Box.Bone.ToString());
			}
			else
			{
				UE_LOG(LogRopeCollision, Log,
					TEXT("[WrapTarget] %s: 랩 캡슐 생성 source=%s localExtent=%s | A=%s B=%s R=%.1f len=%.1f bone=%s"),
					*GetNameSafe(GetOwner()),
					bUsedSimpleCollision ? TEXT("simpleCollision") : TEXT("bounds"),
					*DiagBounds.BoxExtent.ToCompactString(),
					*Capsule.A.ToCompactString(), *Capsule.B.ToCompactString(),
					Capsule.Radius, static_cast<float>(FVector::Dist(Capsule.A, Capsule.B)),
					*Capsule.Bone.ToString());
			}
		}
	}

	// 캐시된 콜라이더 포인터를 풀에 담고(해당 프레임 solve 동안 유효), region 매핑은 bounds 헬퍼가 만든다
	// (스켈레탈 provider와 동일 계약 — 셰이프 1개라 유니언=자기 자신).
	const int32 StartIndex = Gather.Colliders.Num();
	if (bServeBox)
	{
		Gather.Colliders.Add(&Box);
	}
	else
	{
		Gather.Colliders.Add(&Capsule);
	}
	RopeColliderGather::MapCollidersToRegionsByBounds(Gather, StartIndex);
}
