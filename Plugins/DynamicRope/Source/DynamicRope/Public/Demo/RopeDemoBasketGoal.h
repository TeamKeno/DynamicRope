// Copyright Epic Games, Inc. All Rights Reserved.
//
// Demo basket goal. Place it so its trigger sits just under the rim of a hoop already built in the
// level, and the hoop's own materials pulse whenever the ball drops through.
//
// Only a downward pass counts (bRequireDownwardEntry). The ball's velocity is projected onto this
// actor's own down axis, so pushing the ball up through the rim from below is not a goal, and a hoop
// tilted in the level still works as long as the goal actor is rotated with it. A ball rattling on
// the rim raises several overlaps in a row, so each ball is locked out for ScoreCooldown seconds
// after it scores.
//
// The goal owns no hoop or ball geometry; those are level actors the map author has already placed.
// Balls are recognised in two ways, on the same both-ways basis as ARopeDemoDoor's plates:
//  1) Assign the level's ball actors directly in the Balls array.
//  2) Set BallTag and anything carrying that tag counts.
// With neither set, any actor that simulates physics counts, which is the least wiring for a
// playground. Pawns never count. The trigger only overlaps the PhysicsBody and WorldDynamic
// channels, so a ball on a collision preset outside those is not seen.
//
// The whole reaction is that emissive pulse, which keeps the demo map free of extra assets so it
// still opens as-is in a distributed build. Anything further — a sound, a score counter, returning
// the ball to where it was thrown from — hangs off OnGoalScored in Blueprint.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RopeDemoBasketGoal.generated.h"

class UBoxComponent;
class UMaterialInstanceDynamic;

/** Fired on a goal, carrying the ball that scored. */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FRopeDemoGoalScoredSignature,
	ARopeDemoBasketGoal*, Goal, AActor*, Ball);

UCLASS(Blueprintable, ClassGroup = (DynamicRope), meta = (DisplayName = "Rope Demo Basket Goal"))
class DYNAMICROPE_API ARopeDemoBasketGoal : public AActor
{
	GENERATED_BODY()

public:
	ARopeDemoBasketGoal();

	//~ AActor
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;

	/** Broadcast on a goal. */
	UPROPERTY(BlueprintAssignable, Category = "Rope|Demo")
	FRopeDemoGoalScoredSignature OnGoalScored;

	/** Actors carrying this tag count as balls. Leave it empty to rely on Balls alone, or on the
	 *  simulates-physics fallback when that is empty too. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	FName BallTag = TEXT("RopeDemoBall");

	/** The level's ball actors, assigned explicitly. Merged with whatever BallTag collects. */
	UPROPERTY(EditInstanceOnly, BlueprintReadOnly, Category = "Rope|Demo")
	TArray<TObjectPtr<AActor>> Balls;

	/** Count only a ball travelling downwards through the rim, which stops a ball pushed up from
	 *  below scoring. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo")
	bool bRequireDownwardEntry = false;

	/** How fast the ball must be moving down the goal's own down axis to count (cm/s). Raise it if a
	 *  ball merely resting on the rim slips through and scores. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo",
		meta = (ClampMin = "0.0", Units = "cm/s", EditCondition = "bRequireDownwardEntry"))
	float MinDownwardSpeed = 50.0f;

	/** How long a ball is locked out from scoring again after a goal (s). This is what stops a ball
	 *  rattling in the rim counting several times. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo", meta = (ClampMin = "0.0", Units = "s"))
	float ScoreCooldown = 1.0f;

	/**
	 * The actor whose static mesh materials pulse on a goal. Point it at the hoop or the backboard;
	 * with nothing assigned the goal still fires OnGoalScored but shows nothing.
	 *
	 * The materials must expose HoopEmissiveParam as a vector parameter, otherwise the pulse does
	 * nothing; that case is logged as a warning at BeginPlay rather than failing silently.
	 */
	UPROPERTY(EditInstanceOnly, BlueprintReadWrite, Category = "Rope|Demo|Presentation")
	TObjectPtr<AActor> HoopFlashActor = nullptr;

	/** The vector parameter driven on the hoop's materials. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Presentation")
	FName HoopEmissiveParam = TEXT("EmissiveColor");

	/** Pulse colour. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Presentation")
	FLinearColor FlashColor = FLinearColor(0.15f, 1.0f, 0.35f);

	/** Multiplier on FlashColor written into the parameter at the peak of the pulse. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Presentation", meta = (ClampMin = "0.0"))
	float HoopFlashBrightness = 20.0f;

	/** How long the pulse takes to decay (s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rope|Demo|Presentation", meta = (ClampMin = "0.01", Units = "s"))
	float FlashDuration = 0.6f;

protected:
	/** The scoring volume, to be placed just under the rim. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rope|Demo")
	TObjectPtr<UBoxComponent> Trigger = nullptr;

private:
	UFUNCTION()
	void HandleBeginOverlap(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor,
		UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult);

	/** Whether this actor counts as a ball, by explicit assignment, by tag, or by the
	 *  simulates-physics fallback when neither is configured. */
	bool IsQualifyingBall(AActor* Candidate) const;

	/** Whether the ball passed downwards fast enough, measured along the goal's own down axis. */
	bool IsDownwardEntry(AActor* Ball, UPrimitiveComponent* BallComponent) const;

	/** Builds the dynamic materials for the pulse and warns if the parameter is missing. */
	void PrepareHoopMaterials();

	/** Writes the pulse at this strength, 0 being idle and 1 the peak. */
	void ApplyFlash(float Alpha);

	/** World time of each ball's last goal, which drives the ScoreCooldown lockout. */
	TMap<TWeakObjectPtr<AActor>, float> LastScoreTimes;

	/** The hoop's dynamic materials, held as a UPROPERTY so they survive collection. */
	UPROPERTY(Transient)
	TArray<TObjectPtr<UMaterialInstanceDynamic>> HoopMaterials;

	/** Seconds left in the pulse. The actor ticks only while this is above zero. */
	float FlashRemaining = 0.0f;
};
