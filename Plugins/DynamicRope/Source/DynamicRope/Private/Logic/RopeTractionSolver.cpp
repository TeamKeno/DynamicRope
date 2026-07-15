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
}
