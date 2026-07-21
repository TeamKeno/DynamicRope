// Copyright Epic Games, Inc. All Rights Reserved.
//
// FRopeWrappingPhase composite-SDF path tests.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Logic/RopeWrappingPhase.h"
#include "Collision/SDF/RopeSDFCollider.h"
#include "Collision/SDF/RopeSDFSynthetic.h"
#include "Collision/SDF/RopeSDFData.h"
#include "Components/SceneComponent.h"

// 현재 포즈에서 로프 지름보다 좁은 gap으로 이어진 collider들은 skeleton depth와 무관하게 하나의
// wrap island가 되고, 남은 로프 길이로 도달할 수 없는 표면은 같은 mesh라도 island에서 빠져야 한다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrappingPoseSpaceIslandTest,
	"DynamicRope.Wrapping.PoseSpaceIslandUsesSurfaceGapAndReach",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrappingPoseSpaceIslandTest::RunTest(const FString& Parameters)
{
	USceneComponent* Mesh = NewObject<USceneComponent>();
	FRopeBoneSDFVolume VolumeA =
		RopeSDFSynthetic::MakeSphere(FName("arm_r"), FVector::ZeroVector, 10.0f, FIntVector(15), 3.0f);
	FRopeBoneSDFVolume VolumeB =
		RopeSDFSynthetic::MakeSphere(FName("spine"), FVector::ZeroVector, 10.0f, FIntVector(15), 3.0f);
	FRopeBoneSDFVolume VolumeC =
		RopeSDFSynthetic::MakeSphere(FName("arm_l"), FVector::ZeroVector, 10.0f, FIntVector(15), 3.0f);
	FRopeBoneSDFVolume VolumeFar =
		RopeSDFSynthetic::MakeSphere(FName("pelvis"), FVector::ZeroVector, 10.0f, FIntVector(15), 3.0f);

	const FTransform TransformA(FQuat::Identity, FVector(0.0f, 0.0f, 0.0f));
	const FTransform TransformB(FQuat::Identity, FVector(22.0f, 0.0f, 0.0f));
	const FTransform TransformC(FQuat::Identity, FVector(44.0f, 0.0f, 0.0f));
	const FTransform TransformFar(FQuat::Identity, FVector(120.0f, 0.0f, 0.0f));
	FRopeSDFCollider ColliderA(&VolumeA, TransformA, TransformA, 0.0f, VolumeA.Bone, Mesh);
	FRopeSDFCollider ColliderB(&VolumeB, TransformB, TransformB, 0.0f, VolumeB.Bone, Mesh);
	FRopeSDFCollider ColliderC(&VolumeC, TransformC, TransformC, 0.0f, VolumeC.Bone, Mesh);
	FRopeSDFCollider ColliderFar(&VolumeFar, TransformFar, TransformFar, 0.0f, VolumeFar.Bone, Mesh);
	TArray<IRopeCollider*> Colliders = { &ColliderA, &ColliderB, &ColliderC, &ColliderFar };

	FRopeWrapConfig Config;
	Config.ContactQueryRadius = 3.0f;
	Config.bEnableMultiBoneWrapping = true;
	const FRopeWrappingPhase::FContext Ctx{ Config, Colliders,
		3.0f, TEXT("PoseSpaceIslandTest"), true };

	FRopeSimState Sim;
	Sim.SegmentLength = 12.0f;
	for (int32 NodeIndex = 0; NodeIndex < 6; ++NodeIndex)
	{
		Sim.Positions.Add(FVector(static_cast<float>(NodeIndex) * Sim.SegmentLength, 0.0f, 0.0f));
		Sim.PrevPositions.Add(Sim.Positions.Last());
		Sim.InvMass.Add(1.0f);
	}

	FRopeSurfaceAnchor Latch;
	Latch.NodeIndex = 0;
	Latch.Bone = VolumeA.Bone;
	Latch.Mesh = Mesh;

	FRopeWrappingPhase Wrapping;
	Wrapping.State.NumTailNodes = Sim.Num();
	Wrapping.State.PathSurfaceWorld = FVector(10.0f, 0.0f, 0.0f);
	Wrapping.State.PathAxisOrigin = FVector::ZeroVector;
	Wrapping.State.PathAxisDirection = FVector::UpVector;

	TArray<FName> IslandBones;
	TArray<FRopeWrapIslandDebugMember> DebugMembers;
	TArray<FRopeWrapIslandDebugPortal> DebugPortals;
	float AvailableSlack = 0.0f;
	Wrapping.GatherPoseSpaceWrapIsland(
		Latch, Sim, Mesh, IslandBones, DebugMembers, DebugPortals, AvailableSlack, Ctx);

	TestTrue(TEXT("contact arm remains in island"), IslandBones.Contains(VolumeA.Bone));
	TestTrue(TEXT("near torso joins by rope-radius surface gap"), IslandBones.Contains(VolumeB.Bone));
	TestTrue(TEXT("opposite arm joins through the same pose-space component"), IslandBones.Contains(VolumeC.Bone));
	TestFalse(TEXT("surface outside remaining-rope reach is excluded"), IslandBones.Contains(VolumeFar.Bone));
	TestEqual(TEXT("island contains the three touching-column surfaces"), IslandBones.Num(), 3);
	TestEqual(TEXT("debug snapshot reuses the three selected collider bounds"), DebugMembers.Num(), 3);
	TestTrue(TEXT("debug snapshot keeps evaluated surface portals"), DebugPortals.Num() >= 2);
	return true;
}

// Composite Analytic Helix 실패 후에는 실패 지점에서 경로를 이어가지 않고, 최초 latch 본 하나로 경로와
// projection 대상을 완전히 재설정해야 한다.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrappingSingleBoneFallbackTest,
	"DynamicRope.Wrapping.CompositeFailureRestartsSingleBonePath",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrappingSingleBoneFallbackTest::RunTest(const FString& Parameters)
{
	USceneComponent* Mesh = NewObject<USceneComponent>();
	FRopeBoneSDFVolume BodyVolume =
		RopeSDFSynthetic::MakeSphere(FName("body"), FVector::ZeroVector, 10.0f, FIntVector(21), 5.0f);
	FRopeBoneSDFVolume ArmVolume =
		RopeSDFSynthetic::MakeSphere(FName("arm"), FVector::ZeroVector, 5.0f, FIntVector(21), 5.0f);
	const FTransform BodyTransform = FTransform::Identity;
	const FTransform ArmTransform(FQuat::Identity, FVector(15.0f, 0.0f, 0.0f));
	FRopeSDFCollider BodyCollider(
		&BodyVolume, BodyTransform, BodyTransform, 0.0f, BodyVolume.Bone, Mesh);
	FRopeSDFCollider ArmCollider(
		&ArmVolume, ArmTransform, ArmTransform, 0.0f, ArmVolume.Bone, Mesh);
	TArray<IRopeCollider*> Colliders = { &BodyCollider, &ArmCollider };

	FRopeWrapConfig Config;
	Config.ContactQueryRadius = 3.0f;
	Config.bEnableMultiBoneWrapping = true;
	FRopeWrappingPhase::FContext Ctx{ Config, Colliders,
		3.0f, TEXT("SingleBoneFallbackTest"), true };
	Ctx.ResolveMode = ERopeWrapResolveMode::FullSimulation;

	FRopeSimState Sim;
	Sim.SegmentLength = 8.0f;
	for (int32 NodeIndex = 0; NodeIndex < 6; ++NodeIndex)
	{
		Sim.Positions.Add(FVector(10.0f, static_cast<float>(NodeIndex) * Sim.SegmentLength, 0.0f));
		Sim.PrevPositions.Add(Sim.Positions.Last());
		Sim.InvMass.Add(1.0f);
	}

	FRopeSurfaceAnchor Latch;
	Latch.NodeIndex = 0;
	Latch.Bone = BodyVolume.Bone;
	Latch.Mesh = Mesh;
	Latch.LocalSurfacePosition = FVector(10.0f, 0.0f, 0.0f);
	Latch.LocalNormal = FVector::ForwardVector;
	Latch.LocalTangent = FVector::RightVector;

	FRopeWrappingPhase Wrapping;
	Wrapping.State.Mesh = Mesh;
	Wrapping.State.LatchAnchor = Latch;
	Wrapping.State.PathWrapIslandBones = { BodyVolume.Bone, ArmVolume.Bone };
	Wrapping.State.Path.AddDefaulted();
	Wrapping.State.Anchors.AddDefaulted();
	Wrapping.State.PathCompositeSweepAngleRad = PI * 0.5f;

	TestTrue(TEXT("composite failure restarts path initialization"),
		Wrapping.RestartPathBuildAsSingleBoneFallback(Sim, Ctx, TEXT("AutomationTest")));
	TestTrue(TEXT("fallback mode remains active for this wrapping attempt"),
		Wrapping.State.bPathUsesSingleBoneFallback);
	TestFalse(TEXT("fallback does not retain the composite island selector"),
		Wrapping.State.bPathUsesPoseSpaceIsland);
	TestEqual(TEXT("fallback clears composite island members"),
		Wrapping.State.PathWrapIslandBones.Num(), 0);
	TestEqual(TEXT("fallback path restarts from one latch point"), Wrapping.State.Path.Num(), 1);
	TestEqual(TEXT("fallback latch point stays attributed to the original bone"),
		Wrapping.State.Path[0].Bone, BodyVolume.Bone);

	FVector Surface = FVector(10.0f, 0.0f, 0.0f);
	FVector Normal = FVector::ForwardVector;
	FVector Tangent = FVector::RightVector;
	FVector Circumference = FVector::RightVector;
	FName SelectedBone = NAME_None;
	const USceneComponent* SelectedMesh = Mesh;
	TestTrue(TEXT("single-bone projection succeeds at the shared surface"),
		Wrapping.ProjectWrapPointToSingleBone(Mesh, Sim, Ctx,
			Surface, Normal, Tangent, Circumference, SelectedBone, SelectedMesh));
	TestEqual(TEXT("single-bone projection ignores the touching arm collider"),
		SelectedBone, BodyVolume.Bone);

	// Analytic Helix 자체가 terminal failure로 끝나면 같은 throw 안에서 실제 SingleBone
	// Surface Vector Field fallback으로 자동 전환해야 한다.
	FRopeWrappingPhase AutomaticFallbackWrapping;
	TestTrue(TEXT("automatic fallback scenario initializes as a valid wrap"),
		AutomaticFallbackWrapping.Begin(Latch, 0.5f, Sim, Ctx));
	TestTrue(TEXT("full simulation enables composite multi-bone"),
		AutomaticFallbackWrapping.State.bPathUsesPoseSpaceIsland);
	// island 불변식을 깨 terminal failure를 유도한다. public dispatcher가 이를 감지해
	// 구형 Composite Surface Vector Field를 거치지 않고 fallback을 재초기화해야 한다.
	AutomaticFallbackWrapping.State.PathWrapIslandBones.Reset();
	AutomaticFallbackWrapping.AdvancePathBuild(Sim, Ctx);
	TestTrue(TEXT("automatic runtime path enters single-bone fallback"),
		AutomaticFallbackWrapping.State.bPathUsesSingleBoneFallback);
	TestFalse(TEXT("automatic fallback clears composite selector"),
		AutomaticFallbackWrapping.State.bPathUsesPoseSpaceIsland);
	TestFalse(TEXT("successful fallback initialization clears terminal failure"),
		AutomaticFallbackWrapping.State.bPathBuildFailed);
	TestEqual(TEXT("automatic fallback restarts at the original latch"),
		AutomaticFallbackWrapping.State.Path.Num(), 1);

	// Composite Multi-Bone은 FullSimulation 전용이다. 같은 collider/config라도 Assisted에서는
	// pose-space island를 만들지 않고 기존 Sequential Multi-Bone 경로를 사용해야 한다.
	FRopeWrappingPhase::FContext AssistedCtx = Ctx;
	AssistedCtx.ResolveMode = ERopeWrapResolveMode::AssistedJudged;
	FRopeWrappingPhase AssistedWrapping;
	TestTrue(TEXT("assisted sequential multi-bone scenario initializes"),
		AssistedWrapping.Begin(Latch, 0.5f, Sim, AssistedCtx));
	TestFalse(TEXT("assisted mode does not enable composite multi-bone"),
		AssistedWrapping.State.bPathUsesPoseSpaceIsland);
	TestEqual(TEXT("assisted mode does not build a composite island"),
		AssistedWrapping.State.PathWrapIslandBones.Num(), 0);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
