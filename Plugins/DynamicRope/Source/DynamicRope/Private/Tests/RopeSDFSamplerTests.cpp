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

// 최적화된 trilinear가 "느리지만 명백한" 참조 구현과 *비트 단위로* 같은가.
//
// 샘플러는 탭마다 하던 디코드/인덱싱 상수 계산을 호출당 1회로 접고, 격자 내부 셀에서는 축별 클램프를
// 건너뛰는 고속 경로를 탄다. 어느 쪽도 값을 바꾸면 안 된다 — 참조 구현은 공개 API(DecodeDistance)만
// 써서 최적화 전 코드를 그대로 옮긴 것이고, 여기서는 근사 비교가 아니라 == 로 못박는다.
//
// 특히 고속/느린 경로의 분기점인 격자 Max 면을 반드시 지나도록 샘플 좌표를 잡는다(경계에서 갈리면
// 본 이음매에서 법선이 튀는 회귀 #4가 재발한다).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeSDFSamplerBitExactTest,
	"DynamicRope.SDF.SamplerMatchesReferenceExactly",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeSDFSamplerBitExactTest::RunTest(const FString& Parameters)
{
	// 최적화 이전 구현 그대로(공개 DecodeDistance 사용).
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

	// 짝수/홀수 해상도 둘 다(고속 경로 인덱싱이 스트라이드에 의존한다).
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

		// 격자를 촘촘히 훑되 t=0 / t=1(Min·Max 면)을 정확히 포함시킨다 → 느린(클램프) 경로를 반드시 탄다.
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

		// 볼륨 밖(클램프 경로)도 확인한다.
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

#endif // WITH_DEV_AUTOMATION_TESTS
