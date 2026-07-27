// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Collision/SDF/RopeSDFSampler.h"
#include "RopeSDFSynthetic.h"
#include "Collision/SDF/RopeSDFData.h"
#include "Collision/SDF/RopeSDFCollider.h"

// Whether a trilinear sample matches the analytic sphere SDF: zero at the surface, negative inside and positive outside.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeSDFSamplerSphereTest,
	"DynamicRope.SDF.SamplerMatchesAnalyticSphere",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeSDFSamplerSphereTest::RunTest(const FString& Parameters)
{
	const FRopeBoneSDFVolume V =
		RopeSDFSynthetic::MakeSphere(FName("test"), FVector::ZeroVector, 20.0f, FIntVector(31), 10.0f);

	TestTrue(TEXT("synthetic volume is baked"), V.IsBaked());
	// The surface, at about zero.
	TestTrue(TEXT("surface ~ 0"),
		FMath::Abs(RopeSDFSampler::SampleTrilinear(V, FVector(20, 0, 0))) < 1.0f);
	// Inside, at about minus fifteen.
	TestTrue(TEXT("inside ~ -15"),
		FMath::Abs(RopeSDFSampler::SampleTrilinear(V, FVector(5, 0, 0)) + 15.0f) < 1.5f);
	// Outside, at about plus eight.
	TestTrue(TEXT("outside ~ +8"),
		FMath::Abs(RopeSDFSampler::SampleTrilinear(V, FVector(28, 0, 0)) - 8.0f) < 1.5f);
	return true;
}

// Whether the gradient at the surface points outwards along positive X as a unit vector, which is the query normal contract.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeSDFGradientTest,
	"DynamicRope.SDF.GradientPointsOutward",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeSDFGradientTest::RunTest(const FString& Parameters)
{
	const FRopeBoneSDFVolume V =
		RopeSDFSynthetic::MakeSphere(FName("test"), FVector::ZeroVector, 20.0f, FIntVector(31), 10.0f);

	const FVector G = RopeSDFSampler::SampleGradient(V, FVector(20, 0, 0));
	TestTrue(TEXT("gradient is unit-length"), FMath::Abs(static_cast<float>(G.Size()) - 1.0f) < 0.05f);
	TestTrue(TEXT("gradient points +X (outward)"),
		static_cast<float>(FVector::DotProduct(G, FVector(1, 0, 0))) > 0.9f);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeSDFProjectionOutsideBoundsTest,
	"DynamicRope.SDF.ProjectionOutsideBounds",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeSDFProjectionOutsideBoundsTest::RunTest(const FString& Parameters)
{
	const FRopeBoneSDFVolume V =
		RopeSDFSynthetic::MakeSphere(FName("test"), FVector::ZeroVector, 20.0f, FIntVector(31), 10.0f);
	const FRopeSDFCollider Collider(
		&V, FTransform::Identity, FTransform::Identity, 0.0f, FName("test"), nullptr);

	const FVector QueryPoint(34.0, 0.0, 0.0);
	const FRopeSurfaceProjection Hit = Collider.ProjectToSurface(QueryPoint, 15.0f);
	TestTrue(TEXT("query outside bounds projects within MaxDistance"), Hit.bHit);
	TestTrue(TEXT("projected point lies near sphere surface"),
		FVector::Distance(Hit.SurfacePoint, FVector(20.0, 0.0, 0.0)) < 1.0f);
	TestTrue(TEXT("projection reports distance from original query"),
		FMath::Abs(Hit.Distance - 14.0f) < 1.0f);
	TestTrue(TEXT("projection normal points outward"),
		FVector::DotProduct(Hit.Normal, FVector::XAxisVector) > 0.9f);

	const FRopeSurfaceProjection Miss = Collider.ProjectToSurface(QueryPoint, 12.0f);
	TestFalse(TEXT("surface beyond MaxDistance is rejected"), Miss.bHit);
	return true;
}

// Whether both the query coordinate and the normal follow the flipped axis on a mirrored, meaning negatively scaled,
// target. Pushing the sphere from the origin along positive X makes it asymmetric, so after flipping the X axis the
// world surface appears on the negative X side and the outward normal has to be negative X too. Losing the sign gives
// a normal along positive X, pointing inwards, which violates the FRopeContact contract and sucks the rope into the
// body, while on the GPU side the inverse scale is caught by a maximum clamp and the coordinate explodes by a factor
// of a million.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeSDFNegativeScaleTest,
	"DynamicRope.SDF.NegativeScaleMirrorsQueryAndNormal",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeSDFNegativeScaleTest::RunTest(const FString& Parameters)
{
	// A local centre at ten along X with a radius of twenty puts the local surface at x = 30, so after mirroring X the world surface is at x = -30.
	const FRopeBoneSDFVolume V =
		RopeSDFSynthetic::MakeSphere(FName("test"), FVector(10, 0, 0), 20.0f, FIntVector(31), 10.0f);

	const FTransform Mirrored(FQuat::Identity, FVector::ZeroVector, FVector(-1.0, 1.0, 1.0));
	const FRopeSDFCollider Collider(&V, Mirrored, Mirrored, 0.0f, FName("test"), nullptr);

	const FRopeContact Hit = Collider.Query(FVector(-30.0, 0.0, 0.0), 2.0f);
	TestTrue(TEXT("mirrored surface is hit on the -X side"), Hit.bHit);
	TestTrue(TEXT("outward normal follows the mirrored axis (-X)"),
		FVector::DotProduct(Hit.Normal, FVector(-1, 0, 0)) > 0.9);
	// The query is on the surface, so the penetration is the whole node radius at a distance of about zero.
	TestTrue(TEXT("penetration equals the node radius at the surface"),
		FMath::Abs(Hit.Penetration - 2.0f) < 0.5f);

	// There is nothing on the unmirrored positive X side, which is where a lost coordinate sign would have hit.
	const FRopeContact Miss = Collider.Query(FVector(30.0, 0.0, 0.0), 2.0f);
	TestFalse(TEXT("nothing on the unmirrored +X side"), Miss.bHit);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeSDFProjectionBoundaryGradientTest,
	"DynamicRope.SDF.ProjectionGradientAtBounds",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeSDFProjectionBoundaryGradientTest::RunTest(const FString& Parameters)
{
	const FRopeBoneSDFVolume V =
		RopeSDFSynthetic::MakeSphere(FName("test"), FVector::ZeroVector, 20.0f, FIntVector(31), 10.0f);
	const FVector Gradient = RopeSDFSampler::SampleProjectionGradient(V, FVector(30.0, 0.0, 0.0));

	TestTrue(TEXT("boundary gradient is unit-length"),
		FMath::Abs(static_cast<float>(Gradient.Size()) - 1.0f) < 0.05f);
	TestTrue(TEXT("boundary gradient points outward"),
		FVector::DotProduct(Gradient, FVector::XAxisVector) > 0.9f);
	return true;
}

// Whether the optimised trilinear sample is bit-for-bit identical to a slow but obvious reference implementation.
//
// The sampler folds the decode and indexing constants it used to compute per tap into one computation per call, and
// takes a fast path skipping the per-axis clamp for cells inside the grid. Neither may change a value: the reference
// implementation uses the public API alone, meaning DecodeDistance, and is a transcription of the code before the
// optimisation, so this is pinned with equality rather than an approximate comparison.
//
// In particular the sample coordinates are chosen to pass through the grid's maximum faces, which is where the fast
// and slow paths diverge; a difference at the boundary is what makes normals jump at bone seams.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeSDFSamplerBitExactTest,
	"DynamicRope.SDF.SamplerMatchesReferenceExactly",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeSDFSamplerBitExactTest::RunTest(const FString& Parameters)
{
	// Exactly as the implementation was before the optimisation, using the public DecodeDistance.
	auto RefSampleAt = [](const FRopeBoneSDFVolume& V, int32 X, int32 Y, int32 Z) -> float
	{
		X = FMath::Clamp(X, 0, V.Resolution.X - 1);
		Y = FMath::Clamp(Y, 0, V.Resolution.Y - 1);
		Z = FMath::Clamp(Z, 0, V.Resolution.Z - 1);
		return V.DecodeDistance(X + Y * V.Resolution.X + Z * V.Resolution.X * V.Resolution.Y);
	};
	auto RefGridCoord = [](double P, double Mn, double Size, int32 Res) -> double
	{
		if (Size <= KINDA_SMALL_NUMBER || Res < 2)
		{
			return 0.0;
		}
		return FMath::Clamp((P - Mn) / Size, 0.0, 1.0) * (Res - 1);
	};
	auto RefTrilinear = [&](const FRopeBoneSDFVolume& V, const FVector& L) -> float
	{
		if (!V.IsBaked())
		{
			return 0.0f;
		}
		const FVector Min = V.LocalBounds.Min;
		const FVector Size = V.LocalBounds.GetSize();
		const double Gx = RefGridCoord(L.X, Min.X, Size.X, V.Resolution.X);
		const double Gy = RefGridCoord(L.Y, Min.Y, Size.Y, V.Resolution.Y);
		const double Gz = RefGridCoord(L.Z, Min.Z, Size.Z, V.Resolution.Z);
		const int32 X0 = FMath::FloorToInt(Gx);
		const int32 Y0 = FMath::FloorToInt(Gy);
		const int32 Z0 = FMath::FloorToInt(Gz);
		const float Fx = static_cast<float>(Gx - X0);
		const float Fy = static_cast<float>(Gy - Y0);
		const float Fz = static_cast<float>(Gz - Z0);
		const float X00 = FMath::Lerp(RefSampleAt(V, X0, Y0, Z0),         RefSampleAt(V, X0 + 1, Y0, Z0),         Fx);
		const float X10 = FMath::Lerp(RefSampleAt(V, X0, Y0 + 1, Z0),     RefSampleAt(V, X0 + 1, Y0 + 1, Z0),     Fx);
		const float X01 = FMath::Lerp(RefSampleAt(V, X0, Y0, Z0 + 1),     RefSampleAt(V, X0 + 1, Y0, Z0 + 1),     Fx);
		const float X11 = FMath::Lerp(RefSampleAt(V, X0, Y0 + 1, Z0 + 1), RefSampleAt(V, X0 + 1, Y0 + 1, Z0 + 1), Fx);
		return FMath::Lerp(FMath::Lerp(X00, X10, Fy), FMath::Lerp(X01, X11, Fy), Fz);
	};

	// Both an even and an odd resolution, since the fast path's indexing depends on the stride.
	const FRopeBoneSDFVolume Volumes[] = {
		RopeSDFSynthetic::MakeSphere(FName("odd"),  FVector::ZeroVector, 20.0f, FIntVector(31), 10.0f),
		RopeSDFSynthetic::MakeSphere(FName("even"), FVector::ZeroVector, 14.0f, FIntVector(16), 6.0f),
	};

	int32 Compared = 0;
	int32 BoundaryHits = 0;
	for (const FRopeBoneSDFVolume& V : Volumes)
	{
		TestTrue(TEXT("synthetic volume is baked"), V.IsBaked());
		const FVector Min = V.LocalBounds.Min;
		const FVector Size = V.LocalBounds.GetSize();

		// Walks the grid finely while including exactly t = 0 and t = 1, the minimum and maximum faces, which forces the slow clamping path.
		constexpr int32 Steps = 17;
		for (int32 ix = 0; ix <= Steps; ++ix)
		for (int32 iy = 0; iy <= Steps; ++iy)
		for (int32 iz = 0; iz <= Steps; ++iz)
		{
			const FVector T(
				static_cast<double>(ix) / Steps,
				static_cast<double>(iy) / Steps,
				static_cast<double>(iz) / Steps);
			const FVector L = Min + Size * T;
			if (ix == Steps || iy == Steps || iz == Steps)
			{
				++BoundaryHits;
			}

			const float Got = RopeSDFSampler::SampleTrilinear(V, L);
			const float Want = RefTrilinear(V, L);
			++Compared;
			if (Got != Want)
			{
				TestTrue(FString::Printf(
					TEXT("trilinear must match the reference bit-for-bit at local %s (got %.9g, want %.9g)"),
					*L.ToString(), Got, Want), false);
				return false;
			}
		}

	// Outside the volume, on the clamping path, is checked too.
		for (const FVector& Outside : { FVector(1000, 0, 0), FVector(-1000, -1000, -1000), Min - Size })
		{
			const float Got = RopeSDFSampler::SampleTrilinear(V, Outside);
			const float Want = RefTrilinear(V, Outside);
			++Compared;
			TestTrue(FString::Printf(TEXT("outside-bounds sample matches reference at %s"), *Outside.ToString()),
				Got == Want);
		}
	}

	TestTrue(FString::Printf(TEXT("compared %d samples"), Compared), Compared > 10000);
	TestTrue(FString::Printf(TEXT("exercised the clamped slow path (%d boundary samples)"), BoundaryHits),
		BoundaryHits > 0);
	return true;
}

// Pins the grid boundary fallback in SampleGradient directly.
//
// With a per-axis forward difference, a probe at plus H leaving the grid's maximum face makes the trilinear sample
// clamp to the boundary face and that axis's difference dies at zero, so the axis component disappears from the
// normal entirely. Outside a bone seam, meaning a cut face, the normal then lies along the tangent and the node is
// pushed sideways rather than out of the surface. The fix replaces that axis alone with a backward difference, and
// there was no test verifying it directly.
//
// The threshold is chosen so the test necessarily fails without the fallback: a dead axis makes the gradient
// degenerate and the sampler falls back to positive Z, while at a corner the true normal is (1,1,1)/sqrt(3), whose
// dot product with positive Z is 0.577. Hence 0.9, since 0.5 would let the fallback vector pass and the test would
// catch nothing.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeSDFGradientAtGridMaxFaceTest,
	"DynamicRope.SDF.GradientAtGridMaxFace",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeSDFGradientAtGridMaxFaceTest::RunTest(const FString& Parameters)
{
	// MakeSphere fits the quantization band to the data's maximum with no clamping, so the distance does not saturate
	// even at a boundary or corner, which means the gradient measured here carries real information.
	const FRopeBoneSDFVolume V =
		RopeSDFSynthetic::MakeSphere(FName("face"), FVector::ZeroVector, 20.0f, FIntVector(31), 10.0f);
	TestTrue(TEXT("synthetic volume is baked"), V.IsBaked());
	const FVector Max = V.LocalBounds.Max;

	// One: exactly on the positive X maximum face, where the true normal is positive X. With no fallback the X component dies and this fails.
	{
		const FVector N = RopeSDFSampler::SampleGradient(V, FVector(Max.X, 0.0, 0.0));
		TestTrue(FString::Printf(TEXT("+X max-face normal is unit (len=%.4f)"), N.Size()),
			FMath::Abs(static_cast<float>(N.Size()) - 1.0f) < 0.05f);
		TestTrue(FString::Printf(TEXT("+X max-face normal keeps its X component (n=%s)"), *N.ToString()),
			FVector::DotProduct(N, FVector::XAxisVector) > 0.9);
	}

	// Two: a corner where all three axes are at their maximum, so all three have to take the fallback.
	{
		const FVector N = RopeSDFSampler::SampleGradient(V, Max);
		const FVector Truth = Max.GetSafeNormal();
		TestTrue(FString::Printf(TEXT("max-corner normal is unit (len=%.4f)"), N.Size()),
			FMath::Abs(static_cast<float>(N.Size()) - 1.0f) < 0.05f);
		TestTrue(FString::Printf(TEXT("max-corner normal points along the true radial normal (n=%s)"), *N.ToString()),
			FVector::DotProduct(N, Truth) > 0.9);
	}

	// Three: on a minimum face the plus H probe stays inside the boundary, so the forward difference stands and this has to pass regardless of the fallback, as a control.
	{
		const FVector N = RopeSDFSampler::SampleGradient(V, FVector(V.LocalBounds.Min.X, 0.0, 0.0));
		TestTrue(FString::Printf(TEXT("-X min-face normal points -X (n=%s)"), *N.ToString()),
			FVector::DotProduct(N, -FVector::XAxisVector) > 0.9);
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
