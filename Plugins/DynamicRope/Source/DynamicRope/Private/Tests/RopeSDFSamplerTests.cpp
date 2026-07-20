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

// SampleGradient의 격자 경계 폴백(#4, CL 463)을 직접 못박는다.
//
// 축당 순방향 차분은 +H 프로브가 그리드 Max 면을 벗어나면 SampleTrilinear가 경계면으로 클램프해
// 그 축 차분이 0으로 죽는다 → 법선에서 그 축 성분이 통째로 사라져 본 이음매(절단면) 밖에서 법선이
// 접선 방향으로 눕고, 노드가 표면 밖이 아니라 옆으로 밀린다. CL 463이 그 축만 후방 차분으로
// 대체해 고쳤지만 그 수정을 직접 검증하는 테스트가 없었다.
//
// 임계값은 폴백이 빠졌을 때 반드시 실패하도록 잡았다: 축이 죽으면 gradient가 축퇴해 샘플러가 +Z로
// 폴백하는데, 모서리에서 참 법선은 (1,1,1)/sqrt(3)이라 +Z와의 내적이 0.577이다. 그래서 0.9를 쓴다
// (0.5로 두면 폴백 벡터도 통과해 버려 테스트가 아무것도 못 잡는다).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRopeSDFGradientAtGridMaxFaceTest,
	"DynamicRope.SDF.GradientAtGridMaxFace",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRopeSDFGradientAtGridMaxFaceTest::RunTest(const FString& Parameters)
{
	// MakeSphere는 양자화 밴드를 데이터 최댓값에 맞추므로(클램프 없음) 경계/모서리에서도 거리가
	// 포화되지 않는다 = 여기서 재는 gradient는 실제 정보를 담고 있다.
	const FRopeBoneSDFVolume V =
		RopeSDFSynthetic::MakeSphere(FName("face"), FVector::ZeroVector, 20.0f, FIntVector(31), 10.0f);
	TestTrue(TEXT("synthetic volume is baked"), V.IsBaked());
	const FVector Max = V.LocalBounds.Max;

	// (1) +X Max 면 위(정확히 경계). 참 법선 = +X. 폴백이 없으면 X 성분이 죽어 실패한다.
	{
		const FVector N = RopeSDFSampler::SampleGradient(V, FVector(Max.X, 0.0, 0.0));
		TestTrue(FString::Printf(TEXT("+X max-face normal is unit (len=%.4f)"), N.Size()),
			FMath::Abs(static_cast<float>(N.Size()) - 1.0f) < 0.05f);
		TestTrue(FString::Printf(TEXT("+X max-face normal keeps its X component (n=%s)"), *N.ToString()),
			FVector::DotProduct(N, FVector::XAxisVector) > 0.9);
	}

	// (2) 세 축이 동시에 Max인 모서리 — 세 축 모두 폴백을 타야 한다.
	{
		const FVector N = RopeSDFSampler::SampleGradient(V, Max);
		const FVector Truth = Max.GetSafeNormal();
		TestTrue(FString::Printf(TEXT("max-corner normal is unit (len=%.4f)"), N.Size()),
			FMath::Abs(static_cast<float>(N.Size()) - 1.0f) < 0.05f);
		TestTrue(FString::Printf(TEXT("max-corner normal points along the true radial normal (n=%s)"), *N.ToString()),
			FVector::DotProduct(N, Truth) > 0.9);
	}

	// (3) Min 면은 +H 프로브가 경계 안이라 순방향 차분 그대로 — 폴백과 무관하게 정상이어야 한다(대조).
	{
		const FVector N = RopeSDFSampler::SampleGradient(V, FVector(V.LocalBounds.Min.X, 0.0, 0.0));
		TestTrue(FString::Printf(TEXT("-X min-face normal points -X (n=%s)"), *N.ToString()),
			FVector::DotProduct(N, -FVector::XAxisVector) > 0.9);
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
