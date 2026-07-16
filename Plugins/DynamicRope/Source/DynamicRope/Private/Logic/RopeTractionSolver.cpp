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

	float ComputeReelTargetSpeed(float Overshoot, float ReelSpeed, float TaperDist, float DeltaTime)
	{
		const float CloseSpeed = Overshoot / FMath::Max(DeltaTime, 1e-4f); // 이번 프레임에 전량 회수할 속도.
		const float Taper = FMath::Max(TaperDist, 0.01f);
		const float Tapered = FMath::Max(ReelSpeed, 0.0f) * FMath::Clamp(Overshoot / Taper, 0.0f, 1.0f);
		return FMath::Min(Tapered, CloseSpeed);
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

	float ComputeRawTargetShare(float InvMassTarget, float InvMassWielder, float MassBias)
	{
		const float Total = InvMassTarget + InvMassWielder;
		if (Total <= KINDA_SMALL_NUMBER)
		{
			return 0.0f; // 양끝 다 앵커 — 아무도 안 움직인다(호출자가 wielder 몫도 0으로 둔다).
		}
		const float Bias = FMath::Max(MassBias, 0.0f);
		if (FMath::IsNearlyEqual(Bias, 1.0f))
		{
			return InvMassTarget / Total; // 선형 역질량: 무거운 쪽 = 작은 w → 작은 몫.
		}
		// w=0(앵커)은 지수와 무관하게 0 — Pow(0,0)=1이라 Bias=0에서 앵커가 몫을 받는 것을 막는다.
		const float PT = (InvMassTarget > 0.0f) ? FMath::Pow(InvMassTarget, Bias) : 0.0f;
		const float PW = (InvMassWielder > 0.0f) ? FMath::Pow(InvMassWielder, Bias) : 0.0f;
		const float PTotal = PT + PW;
		return (PTotal > KINDA_SMALL_NUMBER) ? (PT / PTotal) : 0.0f;
	}

	bool EvaluateTautGate(float Tension, float Threshold, float ReleaseRatio, bool bWasTaut)
	{
		// 진입/유지 임계 분리(히스테리시스). Threshold ≤ 0이면 둘 다 "장력 > ~0"으로 수렴한다(종전 게이트).
		const float EnterAbove = FMath::Max(Threshold, KINDA_SMALL_NUMBER);
		const float StayAbove = FMath::Max(Threshold * FMath::Clamp(ReleaseRatio, 0.0f, 1.0f), KINDA_SMALL_NUMBER);
		return Tension > (bWasTaut ? StayAbove : EnterAbove);
	}
}
