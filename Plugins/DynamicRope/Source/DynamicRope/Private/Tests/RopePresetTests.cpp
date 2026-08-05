// Copyright 2026 TeamKeno. All Rights Reserved.
//
// Unit tests for URopePreset, the data asset, and URopeComponent::ApplyPreset, verified with NewObject and no world.
// That EnterLoaded and ApplyPreset have no world dependency is the same premise as the phase contract tests in
// RopePierceTests.
//
// A limit of this test's scope, for the same reason as in RopePierceTests: neither the firing of OnPresetApplied, a
// dynamic delegate, nor the chain into the wielder's RefreshModeDerivedState can be verified here. A dynamic delegate
// needs a UObject listener with a UFUNCTION and there is no UCLASS under Private/Tests, and the wielder update
// presumes a world and BeginPlay. Verifying the events, the HUD and the preview update is the job of the
// play-in-editor checklist, through Rope.Preset.Cycle.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Preset/RopePreset.h"
#include "RopeComponent.h"
#include "Core/RopeThrowTypes.h"
#include "Components/StaticMeshComponent.h"
#include "GameFramework/Actor.h"
#if WITH_EDITOR
#include "Misc/DataValidation.h"
#endif

// The friend seam in RopeComponent.h: the only way for the negative test of ApplyPreset's phase gate to force a
// forbidden phase, since the public API cannot produce Wrapped with no world.
struct FRopePresetTestSeam
{
	static void ForcePhase(URopeComponent& Rope, ERopePhase Phase)
	{
		Rope.Phase = Phase;
	}

	static void SeedTransientSimState(URopeComponent& Rope)
	{
		Rope.Sim.SegmentTension = { 100.0f, 200.0f };
		Rope.Sim.TimeAccumulator = 0.5f;
	}

	static int32 GetSegmentTensionNum(const URopeComponent& Rope)
	{
		return Rope.Sim.SegmentTension.Num();
	}

	static float GetTimeAccumulator(const URopeComponent& Rope)
	{
		return Rope.Sim.TimeAccumulator;
	}

	// Loaded tip placement: the private compose helper plus the protected socket virtual it builds on,
	// so the offset contract can be checked without a world.
	static FTransform GetLoadedTipBaseWorld(const URopeComponent& Rope)
	{
		return Rope.MakeLoadedTipBaseWorld();
	}

	static FTransform GetLoadedSocketWorld(const URopeComponent& Rope)
	{
		return Rope.GetLoadedTipTransform();
	}

	// The throw speed contract gate, which is private. It is the single point every throw path calls before changing any state.
	static bool TryThrowSpeed(const URopeComponent& Rope, const FRopeThrowContext& Ctx, float& Out)
	{
		return Rope.TryResolveValidThrowSpeed(Ctx, Out);
	}

	// StartFreshThrow, which is private, used to verify that the rejection path for an invalid speed exits before any state changes, preserving the state.
	static void CallStartFreshThrow(URopeComponent& Rope, const FRopeThrowContext& Ctx)
	{
		Rope.StartFreshThrow(Ctx);
	}
};

namespace
{
	/** An AssistedJudged preset whose values are certainly non-default, for verifying the stamp. */
	URopePreset* MakeStampTestPreset()
	{
		URopePreset* Preset = NewObject<URopePreset>();
		Preset->ResolveMode = ERopeWrapResolveMode::AssistedJudged;
		Preset->NumParticles = 32;
		Preset->RopeLength = 555.0f;
		Preset->MinRopeLength = 111.0f;
		Preset->ReelSpeed = 321.0f;
		Preset->SolverConfig.Substeps = 7;
		Preset->SolverConfig.Iterations = 3;
		Preset->Radius = 3.25f;
		Preset->NumSides = 12;
		Preset->bIncludeOwnerColliders = true;
		Preset->bUseWorldGDF = false;
		return Preset;
	}
}

// A default preset gives the default rope: the combination has to be valid and the essential defaults have to match
// the component's CDO for the "an empty preset means no regression" contract to hold. The preset's fields mirror the
// component's; see the header comment.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePresetDefaultsValidTest,
	"DynamicRope.Preset.DefaultsValid",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePresetDefaultsValidTest::RunTest(const FString& Parameters)
{
	const URopePreset* Preset = GetDefault<URopePreset>();
	const URopeComponent* Rope = GetDefault<URopeComponent>();

	TestEqual(TEXT("the default resolve mode matches"), Preset->ResolveMode, Rope->ResolveMode);
	TestEqual(TEXT("the default particle count matches"), Preset->NumParticles, Rope->NumParticles);
	TestEqual(TEXT("the default rope length matches"), Preset->RopeLength, Rope->RopeLength);
	TestEqual(TEXT("the default minimum rope length matches"), Preset->MinRopeLength, Rope->MinRopeLength);
	TestEqual(TEXT("the default reel speed matches"), Preset->ReelSpeed, Rope->ReelSpeed);
	TestEqual(TEXT("the default radius matches"), Preset->Radius, Rope->Radius);
	TestEqual(TEXT("the default side count matches"), Preset->NumSides, Rope->NumSides);
	TestEqual(TEXT("the default substep count matches"), Preset->SolverConfig.Substeps, Rope->SolverConfig.Substeps);
	TestEqual(TEXT("the default world distance field flag matches"), Preset->bUseWorldGDF, Rope->bUseWorldGDF);
	TestEqual(TEXT("the default tip mesh flag matches"), Preset->bUseTipMesh, Rope->bUseTipMesh);
	TestEqual(TEXT("the default show-when-loaded flag matches"),
		Preset->bShowRopeWhenLoaded, Rope->IsShowRopeWhenLoaded());
	TestEqual(TEXT("the default loaded hand socket matches"), Preset->LoadedHandSocket, Rope->LoadedHandSocket);
	TestTrue(TEXT("the default loaded tip relative transform matches"),
		Preset->LoadedTipRelativeTransform.Equals(Rope->LoadedTipRelativeTransform));
	// The socket override has to default to off so that applying an existing preset does not erase the instance plumbing.
	TestFalse(TEXT("the loaded hand socket override is off by default"), Preset->bOverrideLoadedHandSocket);
	// The default material mirrors too: with none on the preset the stamp would strip the default material and leave the grey fallback. Kept synchronized through the constructor's FObjectFinder.
	TestEqual(TEXT("the default rope material matches"), Preset->RopeMaterial.Get(), Rope->RopeMaterial.Get());
	return true;
}

#if WITH_EDITOR
// Asset save validation through IsDataValid: an inverted length range is reported as a warning but does not block the
// save, since SetRopeLength clamps to the range from the minimum to the maximum and an inversion is an authoring
// mistake rather than invalid data.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePresetDataValidationTest,
	"DynamicRope.Preset.DataValidationWarnsOnLengthInversion",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePresetDataValidationTest::RunTest(const FString& Parameters)
{
	URopePreset* Preset = NewObject<URopePreset>();

	Preset->RopeLength = 300.0f;
	Preset->MinRopeLength = 500.0f;
	{
		FDataValidationContext Context;
		TestNotEqual(TEXT("an inverted length range is a warning rather than invalid"),
			Preset->IsDataValid(Context), EDataValidationResult::Invalid);
		TestEqual(TEXT("an inverted length range gives one warning"), Context.GetNumWarnings(), 1);
	}

	Preset->MinRopeLength = 100.0f;
	{
		FDataValidationContext Context;
		TestEqual(TEXT("a valid length range gives no warning"), Context.GetNumWarnings(), 0);
		TestNotEqual(TEXT("a valid length range is not invalid"),
			Preset->IsDataValid(Context), EDataValidationResult::Invalid);
	}
	return true;
}
#endif // WITH_EDITOR

// Stamping the values and reinitializing: on a successful application the fields are copied and InitRope actually
// runs, so the node count and the length are reflected in the simulation. Applying an AssistedJudged preset to a Free
// rope leaves it Free.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePresetApplyStampsValuesTest,
	"DynamicRope.Preset.ApplyStampsValues",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePresetApplyStampsValuesTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();
	URopePreset* Preset = MakeStampTestPreset();
	FRopePresetTestSeam::SeedTransientSimState(*Rope);

	TestTrue(TEXT("it applies successfully while Free"), Rope->ApplyPreset(Preset));

	TestEqual(TEXT("the resolve mode is stamped"), Rope->ResolveMode, ERopeWrapResolveMode::AssistedJudged);
	TestEqual(TEXT("the particle count is stamped"), Rope->NumParticles, 32);
	TestEqual(TEXT("the rope length is stamped"), Rope->RopeLength, 555.0f);
	TestEqual(TEXT("the minimum rope length is stamped"), Rope->MinRopeLength, 111.0f);
	TestEqual(TEXT("the reel speed is stamped"), Rope->ReelSpeed, 321.0f);
	TestEqual(TEXT("the solver config struct is stamped"), Rope->SolverConfig.Substeps, 7);
	TestEqual(TEXT("the radius is stamped"), Rope->Radius, 3.25f);
	TestEqual(TEXT("the side count is stamped"), Rope->NumSides, 12);
	TestEqual(TEXT("the include-owner-colliders flag is stamped"), Rope->bIncludeOwnerColliders, true);
	TestEqual(TEXT("the world distance field flag is stamped"), Rope->bUseWorldGDF, false);

	// Proof that InitRope ran: the simulation's topology and length were reseeded to the new values.
	TestEqual(TEXT("the node count is reseeded"), Rope->GetNodeCount(), 32);
	TestEqual(TEXT("the current length is reseeded"), Rope->GetCurrentRopeLength(), 555.0f);
	TestEqual(TEXT("reinitializing clears the previous tension"), FRopePresetTestSeam::GetSegmentTensionNum(*Rope), 0);
	TestEqual(TEXT("reinitializing clears the fixed-step time accumulator"), FRopePresetTestSeam::GetTimeAccumulator(*Rope), 0.0f);

	// An AssistedJudged preset does not move the phase.
	TestEqual(TEXT("it stays Free"), Rope->GetPhase(), ERopePhase::Free);
	return true;
}

// The stored setting is the runtime value, with no intermediate interpretation. The accuracy and cost defaults have
// to stay at their documented values, which this guards, and a preset stamps those values as they are.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeStoredConfigIsRuntimeConfigTest,
	"DynamicRope.Preset.StoredConfigIsRuntimeConfig",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeStoredConfigIsRuntimeConfigTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();

	// The solver's accuracy and sweep defaults.
	TestEqual(TEXT("the default substep count is 12"), Rope->SolverConfig.Substeps, 12);
	TestEqual(TEXT("the default iteration count is 4"), Rope->SolverConfig.Iterations, 4);
	TestEqual(TEXT("the default sweep step is 2"), Rope->SolverConfig.SweepStep, 2.0f);
	TestEqual(TEXT("the default maximum sweep samples is 16"), Rope->SolverConfig.MaxSweepSamples, 16);

	// The contact detection sweep and the wrap path build budget defaults.
	TestEqual(TEXT("the default contact sweep step is 2"), Rope->WrapConfig.ContactSweepStep, 2.0f);
	TestEqual(TEXT("the default contact maximum sweep samples is 16"), Rope->WrapConfig.ContactMaxSweepSamples, 16);
	TestEqual(TEXT("the default wrapping path build steps per frame is 8"), Rope->WrapConfig.WrappingPathBuildStepsPerFrame, 8);

	// The stored values survive a preset stamp as well, with no reinterpretation after application.
	URopePreset* Preset = NewObject<URopePreset>();
	Preset->SolverConfig.Substeps = 9;
	Preset->SolverConfig.Iterations = 5;
	TestTrue(TEXT("the preset applies successfully"), Rope->ApplyPreset(Preset));
	TestEqual(TEXT("the substep count is stamped"), Rope->SolverConfig.Substeps, 9);
	TestEqual(TEXT("the iteration count is stamped"), Rope->SolverConfig.Iterations, 5);

	return true;
}

// The throw speed contract: the throw speed has to be positive, in centimetres per second, and zero or negative falls
// back to the throw parameters. An invalid speed, meaning below one after resolution, is rejected before any state
// changes, preserving the phase and the transient state, which removes the contradiction of a slow throw suddenly being fast.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeThrowInvalidSpeedTest,
	"DynamicRope.Throw.InvalidSpeedPreservesState",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeThrowInvalidSpeedTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();

	// 1) Zero on the context and zero in the throw parameters is rejected.
	Rope->ThrowParams.ThrowSpeed = 0.0f;
	FRopeThrowContext Ctx;
	Ctx.ThrowSpeed = 0.0f;
	float Out = -1.0f;
	TestFalse(TEXT("a speed of zero is rejected"), FRopePresetTestSeam::TryThrowSpeed(*Rope, Ctx, Out));

	// 2) Zero on the context falls back to the throw parameters, at 1500, and passes.
	Rope->ThrowParams.ThrowSpeed = 1500.0f;
	TestTrue(TEXT("a zero context falls back to the throw parameters"), FRopePresetTestSeam::TryThrowSpeed(*Rope, Ctx, Out));
	TestEqual(TEXT("the fallback speed is 1500"), Out, 1500.0f);

	// 3) A speed above zero but below one is rejected, per the lower bound contract.
	Ctx.ThrowSpeed = 0.5f;
	TestFalse(TEXT("a speed of 0.5 is rejected"), FRopePresetTestSeam::TryThrowSpeed(*Rope, Ctx, Out));

	// 4) A positive speed passes and keeps its value.
	Ctx.ThrowSpeed = 900.0f;
	TestTrue(TEXT("a speed of 900 passes"), FRopePresetTestSeam::TryThrowSpeed(*Rope, Ctx, Out));
	TestEqual(TEXT("the resolved speed is 900"), Out, 900.0f);

	// 5) State preservation: on an invalid speed StartFreshThrow rejects before changing any state in
	//    ResetStateForNewThrow, so the phase is preserved along with the previous transient tension, which a normal throw would have cleared through InitRope.
	Rope->ThrowParams.ThrowSpeed = 0.0f; // So the fallback is invalid too.
	FRopePresetTestSeam::ForcePhase(*Rope, ERopePhase::Free);
	FRopePresetTestSeam::SeedTransientSimState(*Rope);
	FRopeThrowContext ZeroCtx;
	ZeroCtx.ThrowSpeed = 0.0f;
	FRopePresetTestSeam::CallStartFreshThrow(*Rope, ZeroCtx);
	TestEqual(TEXT("the phase is unchanged, staying Free, on a rejection"), Rope->GetPhase(), ERopePhase::Free);
	TestEqual(TEXT("the transient tension is preserved on a rejection"), FRopePresetTestSeam::GetSegmentTensionNum(*Rope), 2);

	return true;
}

// Taut sensitivity geometrically interpolates the slack ratio and the maximum sag; both the direction and the values are verified. Zero is loose, permitting more, and one is strict, permitting less.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeTautSensitivityMappingTest,
	"DynamicRope.Taut.SensitivityMapping",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeTautSensitivityMappingTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();

	Rope->HoldConfig.TautSensitivity = 0.5f;
	TestEqual(TEXT("at 0.5 the slack is about 0.03"), Rope->GetEffectiveTautSlackRatio(), 0.03f, 0.002f);
	TestEqual(TEXT("at 0.5 the sag is about 20"), Rope->GetEffectiveTautMaxSag(), 20.0f, 0.5f);

	Rope->HoldConfig.TautSensitivity = 0.0f;
	TestEqual(TEXT("at 0 the slack is about 0.09"), Rope->GetEffectiveTautSlackRatio(), 0.09f, 0.002f);
	TestEqual(TEXT("at 0 the sag is about 80"), Rope->GetEffectiveTautMaxSag(), 80.0f, 0.5f);

	Rope->HoldConfig.TautSensitivity = 1.0f;
	TestEqual(TEXT("at 1 the slack is about 0.01"), Rope->GetEffectiveTautSlackRatio(), 0.01f, 0.002f);
	TestEqual(TEXT("at 1 the sag is about 5"), Rope->GetEffectiveTautMaxSag(), 5.0f, 0.5f);

	// The direction: greater sensitivity permits less, meaning it is stricter.
	Rope->HoldConfig.TautSensitivity = 0.3f;
	const float Slack03 = Rope->GetEffectiveTautSlackRatio();
	Rope->HoldConfig.TautSensitivity = 0.7f;
	const float Slack07 = Rope->GetEffectiveTautSlackRatio();
	TestTrue(TEXT("higher sensitivity permits less slack"), Slack07 < Slack03);

	return true;
}

// Mode and phase consistency: applying a GuaranteedWrap preset while Free enters Loaded immediately and passes the
// throw gate. Applying a FullSimulation preset from Loaded then unloads and returns to Free, since FullSimulation has no gate.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePresetModePhaseReconciliationTest,
	"DynamicRope.Preset.ModePhaseReconciliation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePresetModePhaseReconciliationTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();

	URopePreset* Guaranteed = NewObject<URopePreset>();
	Guaranteed->ResolveMode = ERopeWrapResolveMode::GuaranteedWrap;

	TestTrue(TEXT("a GuaranteedWrap preset applies successfully while Free"), Rope->ApplyPreset(Guaranteed));
	TestEqual(TEXT("applying GuaranteedWrap enters Loaded"), Rope->GetPhase(), ERopePhase::Loaded);
	TestTrue(TEXT("the throw gate passes while Loaded"), Rope->CanThrowNow());

	URopePreset* FreeSim = NewObject<URopePreset>();
	FreeSim->ResolveMode = ERopeWrapResolveMode::FullSimulation;

	TestTrue(TEXT("a FullSimulation preset applies successfully from Loaded, which is a permitted phase"), Rope->ApplyPreset(FreeSim));
	TestEqual(TEXT("switching to FullSimulation returns to Free"), Rope->GetPhase(), ERopePhase::Free);
	TestTrue(TEXT("FullSimulation has no phase gate"), Rope->CanThrowNow());
	return true;
}

// Stamping the loaded display switch: a GuaranteedWrap preset's show-when-loaded flag is reflected in the loaded
// visibility. The second application, from Loaded to Loaded, is the essential one: EnterLoaded is an entry edge and
// does not call OnEnterLoaded again, so unless the stamp goes through the setter the new value stays buried until the next load.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePresetStampsShowRopeWhenLoadedTest,
	"DynamicRope.Preset.StampsShowRopeWhenLoaded",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePresetStampsShowRopeWhenLoadedTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();

	URopePreset* Hidden = NewObject<URopePreset>();
	Hidden->ResolveMode = ERopeWrapResolveMode::GuaranteedWrap;
	Hidden->bShowRopeWhenLoaded = false;

	TestTrue(TEXT("the GuaranteedWrap preset applies successfully while Free"), Rope->ApplyPreset(Hidden));
	TestEqual(TEXT("applying GuaranteedWrap enters Loaded"), Rope->GetPhase(), ERopePhase::Loaded);
	TestFalse(TEXT("a preset with it off hides the rope tube while loaded"), Rope->GetVisibleFlag());

	URopePreset* Shown = NewObject<URopePreset>();
	Shown->ResolveMode = ERopeWrapResolveMode::GuaranteedWrap;
	Shown->bShowRopeWhenLoaded = true;

	TestTrue(TEXT("a GuaranteedWrap preset reapplies successfully from Loaded"), Rope->ApplyPreset(Shown));
	TestTrue(TEXT("a preset with it on stamps the value"), Rope->IsShowRopeWhenLoaded());
	TestTrue(TEXT("reapplying while Loaded reflects the visibility immediately"), Rope->GetVisibleFlag());
	return true;
}

// The negative case of the phase gate: outside Free and Loaded, meaning Wrapped, it returns false and changes no
// field at all, which proves the ordering contract that the gate comes before the stamp.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePresetRejectsOutsideFreeReelTest,
	"DynamicRope.Preset.RejectsOutsideFreeReel",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePresetRejectsOutsideFreeReelTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();
	const int32 OldNumParticles = Rope->NumParticles;
	const float OldRopeLength = Rope->RopeLength;
	const ERopeWrapResolveMode OldMode = Rope->ResolveMode;

	FRopePresetTestSeam::ForcePhase(*Rope, ERopePhase::Wrapped);

	URopePreset* Preset = MakeStampTestPreset();
	TestFalse(TEXT("it is refused while Wrapped"), Rope->ApplyPreset(Preset));
	TestEqual(TEXT("the particle count is unchanged on a refusal"), Rope->NumParticles, OldNumParticles);
	TestEqual(TEXT("the rope length is unchanged on a refusal"), Rope->RopeLength, OldRopeLength);
	TestEqual(TEXT("the resolve mode is unchanged on a refusal"), Rope->ResolveMode, OldMode);
	TestEqual(TEXT("the phase is preserved on a refusal"), Rope->GetPhase(), ERopePhase::Wrapped);

	// A null preset is refused too.
	TestFalse(TEXT("a null preset is refused"), Rope->ApplyPreset(nullptr));
	return true;
}

// The hand socket is instance plumbing bound to the owning skeleton, so only a preset that opts in overwrites it.
// A preset with it off, the default, applies with the socket left empty and must not erase the plumbing.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePresetLoadedHandSocketOverrideGateTest,
	"DynamicRope.Preset.LoadedHandSocketOverrideGate",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePresetLoadedHandSocketOverrideGateTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();
	Rope->LoadedHandSocket = FName(TEXT("hand_r"));

	// With the gate off, the default, the instance value is preserved even when the socket is filled in.
	URopePreset* KeepPreset = NewObject<URopePreset>();
	KeepPreset->bUseTipMesh = true;
	KeepPreset->LoadedHandSocket = FName(TEXT("hand_l"));
	TestTrue(TEXT("a preset with the gate off applies successfully"), Rope->ApplyPreset(KeepPreset));
	TestEqual(TEXT("the instance socket is preserved without the opt-in"), Rope->LoadedHandSocket, FName(TEXT("hand_r")));

	// With the gate on it is replaced by the preset's socket.
	URopePreset* OverridePreset = NewObject<URopePreset>();
	OverridePreset->bUseTipMesh = true;
	OverridePreset->bOverrideLoadedHandSocket = true;
	OverridePreset->LoadedHandSocket = FName(TEXT("hand_l"));
	TestTrue(TEXT("a preset with the gate on applies successfully"), Rope->ApplyPreset(OverridePreset));
	TestEqual(TEXT("the socket is replaced on opting in"), Rope->LoadedHandSocket, FName(TEXT("hand_l")));

	// A preset that opts in stamps an empty socket as it stands, which is the explicit clearing path.
	URopePreset* ClearPreset = NewObject<URopePreset>();
	ClearPreset->bUseTipMesh = true;
	ClearPreset->bOverrideLoadedHandSocket = true;
	TestTrue(TEXT("a preset opting in with an empty socket applies successfully"), Rope->ApplyPreset(ClearPreset));
	TestEqual(TEXT("opting in with an empty socket clears it"), Rope->LoadedHandSocket, FName(NAME_None));
	return true;
}

// The loaded placement offset is stamped unconditionally, where the identity is the existing behaviour, and is
// composed in the socket's frame. Both consumers, the tip mesh placement and the last node's pin, share this one
// value, so the composition itself is pinned.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePresetLoadedTipOffsetStampedTest,
	"DynamicRope.Preset.LoadedTipOffsetStamped",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePresetLoadedTipOffsetStampedTest::RunTest(const FString& Parameters)
{
	URopeComponent* Rope = NewObject<URopeComponent>();

	// The socket fallback is the component transform, and an identity socket would make Offset*Socket
	// and Socket*Offset agree - which is exactly the mistake this test exists to catch. Push the
	// component somewhere non-trivial first, and fail loudly if that did not take.
	Rope->SetRelativeTransform(FTransform(FQuat(FVector(1, 0, 0), HALF_PI), FVector(-40.0f, 7.0f, 100.0f)));
	Rope->UpdateComponentToWorld();
	TestFalse(TEXT("the socket fallback has to be non-identity for the ordering check to mean anything"),
		FRopePresetTestSeam::GetLoadedSocketWorld(*Rope).Equals(FTransform::Identity));

	// With the default identity offset the composition has to give the socket transform exactly, which this guards.
	TestTrue(TEXT("an identity offset gives the socket transform as it stands"),
		FRopePresetTestSeam::GetLoadedTipBaseWorld(*Rope).Equals(
			FRopePresetTestSeam::GetLoadedSocketWorld(*Rope)));

	const FTransform Offset(FQuat(FVector(0, 0, 1), HALF_PI), FVector(12.0f, -3.0f, 5.0f));
	URopePreset* Preset = NewObject<URopePreset>();
	Preset->bUseTipMesh = true;
	Preset->LoadedTipRelativeTransform = Offset;
	TestTrue(TEXT("the preset applies successfully"), Rope->ApplyPreset(Preset));
	TestTrue(TEXT("the loaded offset is stamped"), Rope->LoadedTipRelativeTransform.Equals(Offset));

	// In the socket's local frame it is the offset composed with the socket's world transform, so the offset applies first.
	const FTransform SocketWorld = FRopePresetTestSeam::GetLoadedSocketWorld(*Rope);
	TestTrue(TEXT("it is composed in the socket's frame"),
		FRopePresetTestSeam::GetLoadedTipBaseWorld(*Rope).Equals(Offset * SocketWorld));
	// The reverse order must not hold, which pins that the composition order really is the contract.
	TestFalse(TEXT("the reverse composition order does not match"),
		FRopePresetTestSeam::GetLoadedTipBaseWorld(*Rope).Equals(SocketWorld * Offset));
	return true;
}

// A tag-reused tip across a preset switch: the authored relative transform is restored on teardown, so the previous
// preset's placement scale of 0.2 does not contaminate the authored baseline of the next acquisition, which guards
// against the scale accumulating.
// The per-frame placement in FinalizeSimFrame does not run in a test, so the state the placement would have overwritten is written directly to reproduce it.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePresetTagTipTransformRestoreTest,
	"DynamicRope.Preset.TagTipTransformRestoredAcrossPresets",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePresetTagTipTransformRestoreTest::RunTest(const FString& Parameters)
{
	AActor* Owner = NewObject<AActor>();
	UStaticMeshComponent* TagTip = NewObject<UStaticMeshComponent>(Owner);
	TagTip->ComponentTags.Add(FName(TEXT("RopeTipTagTest")));
	TagTip->SetRelativeScale3D(FVector(2.0f));		// The authored scale.

	URopeComponent* Rope = NewObject<URopeComponent>(Owner);
	Rope->TipMeshComponentTag = FName(TEXT("RopeTipTagTest"));

	// Preset one: a tip in use with a placement scale of 0.2.
	URopePreset* SmallTipPreset = NewObject<URopePreset>();
	SmallTipPreset->bUseTipMesh = true;
	SmallTipPreset->TipMeshRelativeTransform.SetScale3D(FVector(0.2f));
	TestTrue(TEXT("preset one applies"), Rope->ApplyPreset(SmallTipPreset));
	TestEqual(TEXT("the tagged component is acquired"), Rope->GetTipMeshComponent(), TagTip);

	// Reproduces the state where the frame's placement overwrote the world scale with the authored 2 times the preset's 0.2.
	TagTip->SetWorldScale3D(FVector(2.0f * 0.2f));

	// Preset two: a tip in use with the default transform, at a scale of one. The teardown restores it, so the baseline for reacquisition has to be the authored value.
	URopePreset* DefaultTipPreset = NewObject<URopePreset>();
	DefaultTipPreset->bUseTipMesh = true;
	TestTrue(TEXT("preset two applies"), Rope->ApplyPreset(DefaultTipPreset));
	TestEqual(TEXT("it stays acquired"), Rope->GetTipMeshComponent(), TagTip);
	TestEqual(TEXT("the authored scale is restored, with no 0.2 contamination"), TagTip->GetRelativeScale3D(), FVector(2.0f));
	return true;
}

// A tag-reused tip against a preset with the tip turned off: the external component is not destroyed, not being
// owned, only released, and is restored to its authored transform. The rope's tip pointer is null.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopePresetTagTipSurvivesTipOffTest,
	"DynamicRope.Preset.TagTipSurvivesTipOffPreset",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopePresetTagTipSurvivesTipOffTest::RunTest(const FString& Parameters)
{
	AActor* Owner = NewObject<AActor>();
	UStaticMeshComponent* TagTip = NewObject<UStaticMeshComponent>(Owner);
	TagTip->ComponentTags.Add(FName(TEXT("RopeTipTagTest2")));
	TagTip->SetRelativeScale3D(FVector(3.0f));

	URopeComponent* Rope = NewObject<URopeComponent>(Owner);
	Rope->TipMeshComponentTag = FName(TEXT("RopeTipTagTest2"));

	URopePreset* TipOnPreset = NewObject<URopePreset>();
	TipOnPreset->bUseTipMesh = true;
	TipOnPreset->TipMeshRelativeTransform.SetScale3D(FVector(0.5f));
	TestTrue(TEXT("the preset with the tip on applies"), Rope->ApplyPreset(TipOnPreset));
	TagTip->SetWorldScale3D(FVector(3.0f * 0.5f));	// Reproduces the placement overwrite.

	URopePreset* TipOffPreset = NewObject<URopePreset>();	// The default has the tip mesh off.
	TestTrue(TEXT("the preset with the tip off applies"), Rope->ApplyPreset(TipOffPreset));
	TestNull(TEXT("the rope's tip reference is released"), Rope->GetTipMeshComponent());
	TestTrue(TEXT("the external component survives"), IsValid(TagTip));
	TestEqual(TEXT("the authored transform is restored"), TagTip->GetRelativeScale3D(), FVector(3.0f));
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
