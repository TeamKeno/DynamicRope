// Copyright Epic Games, Inc. All Rights Reserved.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Collision/SDF/RopeSDFSampler.h"
#include "Collision/SDF/RopeSDFSynthetic.h"
#include "Collision/SDF/RopeSDFData.h"
#include "Collision/SDF/RopeSDFCollider.h"

// trilinear 샘플이 해석적 구 SDF와 일치하는가(표면 0, 안쪽 음수, 바깥 양수).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeSDFSamplerSphereTest,
	"DynamicRope.SDF.SamplerMatchesAnalyticSphere",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeSDFSamplerSphereTest::RunTest(const FString& Parameters)
{
	const FRopeBoneSDFVolume V =
		RopeSDFSynthetic::MakeSphere(FName("test"), FVector::ZeroVector, 20.0f, FIntVector(31), 10.0f);

	TestTrue(TEXT("synthetic volume is baked"), V.IsBaked());
	// 표면 (20,0,0) ~ 0
	TestTrue(TEXT("surface ~ 0"),
		FMath::Abs(RopeSDFSampler::SampleTrilinear(V, FVector(20, 0, 0))) < 1.0f);
	// 안쪽 (5,0,0) ~ -15
	TestTrue(TEXT("inside ~ -15"),
		FMath::Abs(RopeSDFSampler::SampleTrilinear(V, FVector(5, 0, 0)) + 15.0f) < 1.5f);
	// 바깥 (28,0,0) ~ +8
	TestTrue(TEXT("outside ~ +8"),
		FMath::Abs(RopeSDFSampler::SampleTrilinear(V, FVector(28, 0, 0)) - 8.0f) < 1.5f);
	return true;
}

// gradient가 표면에서 바깥쪽(+X)을 단위 벡터로 가리키는가(= Query 법선 계약).
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

#endif // WITH_DEV_AUTOMATION_TESTS
