// Copyright 2026 TeamKeno. All Rights Reserved.
//
// FRopeWrappingPhase composite-SDF path tests.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Logic/RopeWrappingPhase.h"
#include "Collision/SDF/RopeSDFCollider.h"
#include "RopeSDFSynthetic.h"
#include "Collision/SDF/RopeSDFData.h"
#include "Components/SceneComponent.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrappingAxisFollowsTailUpTest,
	"DynamicRope.Wrapping.AxisSignFollowsTailRelativeToCharacterUp",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrappingAxisFollowsTailUpTest::RunTest(const FString& Parameters)
{
	USceneComponent* Mesh = NewObject<USceneComponent>();
	Mesh->SetWorldTransform(FTransform(
		FQuat(FVector::RightVector, UE_HALF_PI), FVector::ZeroVector));
	const FVector CharacterUp = Mesh->GetUpVector().GetSafeNormal();

	FRopeSurfaceAnchor Latch;
	Latch.NodeIndex = 0;
	FRopeSimState Sim;
	Sim.Positions = { FVector::ZeroVector, CharacterUp * 10.0f, CharacterUp * 30.0f };

	FRopeWrappingPhase Wrapping;
	FVector AxisDirection = CharacterUp;
	Wrapping.OrientWrappingAxisByTail(Latch, Sim, Mesh, AxisDirection);
	TestTrue(TEXT("upper tail orients the axis away from character Up"),
		FVector::DotProduct(AxisDirection, CharacterUp) < -0.99f);

	Sim.Positions = { FVector::ZeroVector, -CharacterUp * 10.0f, -CharacterUp * 30.0f };
	AxisDirection = -CharacterUp;
	Wrapping.OrientWrappingAxisByTail(Latch, Sim, Mesh, AxisDirection);
	TestTrue(TEXT("lower tail orients the axis toward character Up"),
		FVector::DotProduct(AxisDirection, CharacterUp) > 0.99f);
	return true;
}

// Colliders joined by a gap narrower than the rope's diameter in the current pose form a single wrap island
// regardless of skeleton depth, and a surface the remaining rope length cannot reach has to drop out of the island
// even on the same mesh.
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
	TArray<FRopeWrapIslandMember> Members;
	TArray<FRopeWrapIslandPortal> Portals;
	float AvailableSlack = 0.0f;
	Wrapping.GatherPoseSpaceWrapIsland(
		Latch, Sim, Mesh, IslandBones, Members, Portals, AvailableSlack, Ctx);

	TestTrue(TEXT("contact arm remains in island"), IslandBones.Contains(VolumeA.Bone));
	TestTrue(TEXT("near torso joins by rope-radius surface gap"), IslandBones.Contains(VolumeB.Bone));
	TestTrue(TEXT("opposite arm joins through the same pose-space component"), IslandBones.Contains(VolumeC.Bone));
	TestFalse(TEXT("surface outside remaining-rope reach is excluded"), IslandBones.Contains(VolumeFar.Bone));
	TestEqual(TEXT("island contains the three touching-column surfaces"), IslandBones.Num(), 3);
	TestEqual(TEXT("island members reuse the three selected collider bounds"), Members.Num(), 3);
	TestTrue(TEXT("island keeps evaluated surface portals"), Portals.Num() >= 2);
	return true;
}

// A fully straightened flight guide leaves the captured rope span entirely in the wrapping plane. Composite
// fallback pitch is then controlled by the angle between that plane normal and the SDF island's principal axis:
// parallel axes stay on one slice, while perpendicular axes receive the configured maximum pitch.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeWrappingStraightContactIslandAxisPitchTest,
	"DynamicRope.Wrapping.CompositeStraightContactTracksIslandAxis",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeWrappingStraightContactIslandAxisPitchTest::RunTest(const FString& Parameters)
{
	USceneComponent* Mesh = NewObject<USceneComponent>();
	FRopeBoneSDFVolume BodyVolume =
		RopeSDFSynthetic::MakeSphere(FName("body"), FVector::ZeroVector, 10.0f, FIntVector(21), 5.0f);
	FRopeBoneSDFVolume ArmVolume =
		RopeSDFSynthetic::MakeSphere(FName("arm"), FVector::ZeroVector, 5.0f, FIntVector(21), 5.0f);
	const FTransform BodyTransform = FTransform::Identity;
	// Separating the two oriented SDF bounds along Z gives the aggregate island an unambiguous Z axis.
	const FTransform ArmTransform(FQuat::Identity, FVector(0.0f, 0.0f, 15.0f));
	FRopeSDFCollider BodyCollider(
		&BodyVolume, BodyTransform, BodyTransform, 0.0f, BodyVolume.Bone, Mesh);
	FRopeSDFCollider ArmCollider(
		&ArmVolume, ArmTransform, ArmTransform, 0.0f, ArmVolume.Bone, Mesh);
	TArray<IRopeCollider*> Colliders = { &BodyCollider, &ArmCollider };

	const auto MakeStraightSim = [](const FVector& SpanDirection)
	{
		FRopeSimState Sim;
		Sim.SegmentLength = 8.0f;
		for (int32 NodeIndex = 0; NodeIndex < 6; ++NodeIndex)
		{
			Sim.Positions.Add(FVector(10.0f, 0.0f, 0.0f) +
				SpanDirection * (static_cast<float>(NodeIndex) * Sim.SegmentLength));
			Sim.PrevPositions.Add(Sim.Positions.Last());
			Sim.InvMass.Add(1.0f);
		}
		return Sim;
	};

	FRopeSurfaceAnchor Latch;
	Latch.NodeIndex = 0;
	Latch.Bone = BodyVolume.Bone;
	Latch.Mesh = Mesh;
	Latch.LocalSurfacePosition = FVector(10.0f, 0.0f, 0.0f);
	Latch.LocalNormal = FVector::ForwardVector;
	Latch.LocalTangent = FVector::RightVector;

	FRopeWrapConfig Config;
	Config.ContactQueryRadius = 3.0f;
	Config.bEnableMultiBoneWrapping = true;
	Config.WrappingAxisSource = ERopeWrappingAxisSource::CaptureTravelPlane;
	Config.WrappingHelixPitchScale = 0.25f;

	FRopeCaptureTravelFrame ParallelFrame;
	ParallelFrame.bValid = true;
	ParallelFrame.RegionCenter = FVector::ZeroVector;
	ParallelFrame.SpanDirection = FVector::RightVector;
	ParallelFrame.AverageVelocity = FVector::RightVector;
	FRopeSimState ParallelSim = MakeStraightSim(ParallelFrame.SpanDirection);
	FRopeWrappingPhase::FContext ParallelCtx{ Config, Colliders,
		3.0f, TEXT("ParallelIslandAxisPitchTest"), true,
		/*bHasGuidePlaneNormal*/ true, /*GuidePlaneNormal*/ FVector::UpVector, &ParallelFrame };
	ParallelCtx.ResolveMode = ERopeWrapResolveMode::FullSimulation;

	FRopeWrappingPhase ParallelWrapping;
	TestTrue(TEXT("parallel-axis composite wrapping initializes"),
		ParallelWrapping.Begin(Latch, 0.5f, ParallelSim, ParallelCtx));
	TestTrue(TEXT("parallel-axis geometry selects the composite SDF island"),
		ParallelWrapping.State.bPathUsesPoseSpaceIsland);
	const float ParallelPitch = FMath::Abs(
		ParallelWrapping.State.PathCompositeHelixPitchScale);
	TestTrue(FString::Printf(TEXT("parallel rope-plane and island axes suppress pitch (pitch=%.3f)"),
			ParallelPitch), ParallelPitch <= KINDA_SMALL_NUMBER);

	const FVector DiagonalPlaneNormal = FVector(0.0f, 1.0f, 1.0f).GetSafeNormal();
	FRopeCaptureTravelFrame DiagonalFrame;
	DiagonalFrame.bValid = true;
	DiagonalFrame.RegionCenter = FVector::ZeroVector;
	DiagonalFrame.SpanDirection = FVector(0.0f, 1.0f, -1.0f).GetSafeNormal();
	DiagonalFrame.AverageVelocity = DiagonalFrame.SpanDirection;
	FRopeSimState DiagonalSim = MakeStraightSim(DiagonalFrame.SpanDirection);
	FRopeWrappingPhase::FContext DiagonalCtx{ Config, Colliders,
		3.0f, TEXT("DiagonalIslandAxisPitchTest"), true,
		/*bHasGuidePlaneNormal*/ true, DiagonalPlaneNormal, &DiagonalFrame };
	DiagonalCtx.ResolveMode = ERopeWrapResolveMode::FullSimulation;

	FRopeWrappingPhase DiagonalWrapping;
	TestTrue(TEXT("diagonal-axis composite wrapping initializes"),
		DiagonalWrapping.Begin(Latch, 0.5f, DiagonalSim, DiagonalCtx));
	const float DiagonalPitch = FMath::Abs(
		DiagonalWrapping.State.PathCompositeHelixPitchScale);
	const float ExpectedDiagonalPitch = FMath::Abs(Config.WrappingHelixPitchScale) *
		FMath::Sqrt(0.5f);
	TestTrue(FString::Printf(TEXT("45-degree axis angle scales pitch continuously (%.3f vs %.3f)"),
			DiagonalPitch, ExpectedDiagonalPitch),
		FMath::IsNearlyEqual(DiagonalPitch, ExpectedDiagonalPitch, 0.01f));

	FRopeCaptureTravelFrame PerpendicularFrame;
	PerpendicularFrame.bValid = true;
	PerpendicularFrame.RegionCenter = FVector::ZeroVector;
	PerpendicularFrame.SpanDirection = -FVector::UpVector;
	PerpendicularFrame.AverageVelocity = -FVector::UpVector;
	FRopeSimState PerpendicularSim = MakeStraightSim(PerpendicularFrame.SpanDirection);
	FRopeWrappingPhase::FContext PerpendicularCtx{ Config, Colliders,
		3.0f, TEXT("PerpendicularIslandAxisPitchTest"), true,
		/*bHasGuidePlaneNormal*/ true, /*GuidePlaneNormal*/ FVector::RightVector,
		&PerpendicularFrame };
	PerpendicularCtx.ResolveMode = ERopeWrapResolveMode::FullSimulation;

	FRopeWrappingPhase PerpendicularWrapping;
	TestTrue(TEXT("perpendicular-axis composite wrapping initializes"),
		PerpendicularWrapping.Begin(Latch, 0.5f, PerpendicularSim, PerpendicularCtx));
	TestTrue(TEXT("perpendicular-axis geometry selects the composite SDF island"),
		PerpendicularWrapping.State.bPathUsesPoseSpaceIsland);
	const float PerpendicularPitch = FMath::Abs(
		PerpendicularWrapping.State.PathCompositeHelixPitchScale);
	TestTrue(FString::Printf(TEXT("perpendicular axes retain configured maximum pitch (%.3f vs %.3f)"),
			PerpendicularPitch, FMath::Abs(Config.WrappingHelixPitchScale)),
		FMath::IsNearlyEqual(PerpendicularPitch,
			FMath::Abs(Config.WrappingHelixPitchScale), 0.01f));
	TestTrue(TEXT("island-axis angle materially changes straight-contact pitch"),
		ParallelPitch < DiagonalPitch && DiagonalPitch < PerpendicularPitch);
	return true;
}

// After the composite analytic helix fails, the path must not continue from the point of failure: the path and the
// projection targets are reset entirely to the single initial latch bone.
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
		Wrapping.ProjectWrapPointToLatchBone(Mesh, Sim, Ctx,
			Surface, Normal, Tangent, Circumference, SelectedBone, SelectedMesh));
	TestEqual(TEXT("single-bone projection ignores the touching arm collider"),
		SelectedBone, BodyVolume.Bone);

	// When the analytic helix itself ends in a terminal failure, it has to switch automatically to the real
	// single-bone surface vector field fallback within the same throw.
	FRopeWrappingPhase AutomaticFallbackWrapping;
	TestTrue(TEXT("automatic fallback scenario initializes as a valid wrap"),
		AutomaticFallbackWrapping.Begin(Latch, 0.5f, Sim, Ctx));
	TestTrue(TEXT("full simulation enables composite multi-bone"),
		AutomaticFallbackWrapping.State.bPathUsesPoseSpaceIsland);
	// Breaking the island invariant induces a terminal failure. The public dispatcher has to detect it and
	// reinitialize the fallback without passing through the composite surface vector field.
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

	// Composite multi-bone is for FullSimulation alone. With the same colliders and configuration, an assisted throw
	// must not build a pose-space island and has to use the existing sequential multi-bone path.
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
