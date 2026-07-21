// Copyright Epic Games, Inc. All Rights Reserved.

#include "Logic/RopeTractionSolver.h"

namespace RopeTraction
{
	float ComputeAxisDeltaV(float CurAlong, const FRopeAxisServo& Servo)
	{
		// 상쇄 기준: 바깥으로 가고 있으면 0에서 출발한 것으로 본다(그 상쇄분은 아래 ΔV에 포함돼 함께 인가된다).
		const float Base = Servo.bCancelOutward ? FMath::Max(CurAlong, 0.0f) : CurAlong;
		// 가속만 모드에서 이미 목표 이상이면 Base 유지(= 제동 없음).
		const bool bApproach = Servo.bBidirectional || (Servo.TargetSpeed > Base);
		const float Final = bApproach ? (Base + (Servo.TargetSpeed - Base) * Servo.Alpha) : Base;
		return Final - CurAlong;
	}

	float ClampAxisImpulse(float DeltaV, float Mass, float MaxImpulse)
	{
		const float J = Mass * DeltaV;
		return (MaxImpulse > 0.0f) ? FMath::Clamp(J, -MaxImpulse, MaxImpulse) : J;
	}

	FVector ClampInjectedVelocity(const FVector& NewVel, const FVector& OldVel, float SpeedCap)
	{
		if (SpeedCap <= 0.0f)
		{
			return NewVel;
		}
		return NewVel.GetClampedToMaxSize(FMath::Max(SpeedCap, static_cast<float>(OldVel.Size())));
	}

	float InvMassFromMass(float Mass)
	{
		return (Mass > KINDA_SMALL_NUMBER) ? (1.0f / Mass) : 0.0f;
	}

	float ExpSmoothAlpha(float Tau, float DeltaTime)
	{
		return (Tau > KINDA_SMALL_NUMBER) ? (1.0f - FMath::Exp(-DeltaTime / Tau)) : 1.0f;
	}

	FVector SmoothDirection(const FVector& Current, const FVector& Target, float Alpha)
	{
		if (Current.IsNearlyZero())
		{
			return Target; // 미시드 — 측정값으로 시드(래그 없음).
		}
		const FVector Smoothed = FMath::Lerp(Current, Target, Alpha).GetSafeNormal();
		// 정반대 방향 상쇄 축퇴(180° 반전 순간): Lerp가 0이 되면 raw로 재시드한다.
		return Smoothed.IsNearlyZero() ? Target : Smoothed;
	}

	FVector SampleFractionalAim(const TArray<FVector>& Positions, float AimF, int32 AnchorNode)
	{
		const int32 A0 = FMath::FloorToInt(AimF);
		const int32 A1 = FMath::Min(A0 + 1, AnchorNode);
		if (!Positions.IsValidIndex(A0) || !Positions.IsValidIndex(A1))
		{
			return FVector::ZeroVector;
		}
		return FMath::Lerp(Positions[A0], Positions[A1], AimF - static_cast<float>(A0));
	}

	bool EvaluateTautGate(float Tension, float Threshold, float ReleaseRatio, bool bWasTaut)
	{
		// 진입/유지 임계 분리(히스테리시스). Threshold ≤ 0이면 둘 다 "장력 > ~0"으로 수렴한다(종전 게이트).
		const float EnterAbove = FMath::Max(Threshold, KINDA_SMALL_NUMBER);
		const float StayAbove = FMath::Max(Threshold * FMath::Clamp(ReleaseRatio, 0.0f, 1.0f), KINDA_SMALL_NUMBER);
		return Tension > (bWasTaut ? StayAbove : EnterAbove);
	}

	bool EvaluateChainTautGate(float ChordLen, float RestLen, float SlackRatio, float ReleaseScale, bool bWasTaut)
	{
		if (RestLen <= KINDA_SMALL_NUMBER)
		{
			// 자유 구간 없음(앵커=손) 등 판정 불능 — 팽팽 아님.
			return false;
		}
		// 진입/유지 임계 분리(히스테리시스): 유지는 슬랙 허용을 ReleaseScale배로 완화한다. 곱이 1 이상이면
		// "슬랙 전량 허용"(래치된 게이트가 chord와 무관하게 유지)으로 수렴한다 — 1로 캡해 음수 임계를 막는다.
		const float Ratio = FMath::Clamp(SlackRatio, 0.0f, 1.0f);
		const float EffRatio = bWasTaut ? FMath::Min(Ratio * FMath::Max(ReleaseScale, 1.0f), 1.0f) : Ratio;
		return ChordLen >= RestLen * (1.0f - EffRatio);
	}

	float SolveTetherLambda(const FRopeTetherConstraint& In, float DeltaTime)
	{
		if (In.C <= 0.0f || DeltaTime <= 1e-4f)
		{
			return 0.0f; // 슬랙(또는 dt 축퇴) — 로프는 밀지 못한다. 이게 슬랙 게이트의 전부다.
		}
		const float WSum = FMath::Max(In.InvMassTarget, 0.0f) + FMath::Max(In.InvMassWielder, 0.0f);
		const float Denom = WSum + FMath::Max(In.Compliance, 0.0f) / (DeltaTime * DeltaTime);
		if (Denom <= KINDA_SMALL_NUMBER)
		{
			return 0.0f; // 양끝 다 앵커 — 아무도 못 움직인다(한계 이탈은 거리 release가 처리).
		}

		// 위치 회수 명령(접근 속도): 이번 프레임 C의 β 비율을 닫는다. C가 큰 프레임(커밋 직후 이미 초과 등)의
		// 스파이크는 MaxBiasSpeed로 상한 — 벌어짐 상쇄와 달리 이 항만 운동량에 바이어스로 남으므로
		// (슬랙 전환 후 접근 코스팅 상한 = 이 값), 상한이 곧 테더가 만들 수 있는 최대 접근 속도다.
		const float Beta = FMath::Clamp(In.SettleAlpha, 0.0f, 1.0f);
		float BiasSpeed = Beta * In.C / DeltaTime;
		if (In.MaxBiasSpeed > 0.0f)
		{
			BiasSpeed = FMath::Min(BiasSpeed, In.MaxBiasSpeed);
		}

		// 필요한 접근 속도 변화 = 목표(−BiasSpeed) − 현재(SepSpeed). 이미 목표 이상으로 접근 중이면
		// 무동작(단방향 — 로프는 접근을 제동하지 않는다).
		const float DeltaS = -BiasSpeed - In.SepSpeed;
		if (DeltaS >= 0.0f)
		{
			return 0.0f;
		}
		const float Lambda = -DeltaS / Denom;
		return (In.MaxTension > 0.0f) ? FMath::Min(Lambda, In.MaxTension * DeltaTime) : Lambda;
	}
}
