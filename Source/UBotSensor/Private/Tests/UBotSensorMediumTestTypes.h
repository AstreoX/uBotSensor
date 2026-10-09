#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "UBotPerceptionMedium.h"
#include "UBotSensorMediumTestTypes.generated.h"

// Test-only reflected types for UBotSensor.Medium.* (UHT cannot see types declared in a .cpp, and it does
// not allow WITH_DEV_AUTOMATION_TESTS guards, so these are compiled in every configuration).

/** Same reflected field names as the AgentCombatCore smoke cell (only the fields the sensors read). */
USTRUCT()
struct FUBotSensorMediumTestLegacyCell
{
    GENERATED_BODY()

    UPROPERTY()
    FVector WorldPosition = FVector::ZeroVector;

    UPROPERTY()
    float Density = 0.0f;

    UPROPERTY()
    float EdgeFactor = 1.0f;
};

/**
 * Stand-in for a legacy smoke actor: found through the legacy tag and read through reflection only
 * (IsSmokeFieldActive, GetSmokeCellSnapshots, CellVisualRadiusMeters, CellSizeMeters).
 */
UCLASS(Transient, NotBlueprintable, NotPlaceable, HideDropdown)
class AUBotSensorMediumTestLegacyActor : public AActor
{
    GENERATED_BODY()

public:
    UPROPERTY()
    float CellVisualRadiusMeters = 1.5f;

    UPROPERTY()
    float CellSizeMeters = 1.0f;

    UPROPERTY()
    bool bSmokeFieldActive = true;

    UPROPERTY()
    TArray<FUBotSensorMediumTestLegacyCell> Cells;

    UFUNCTION()
    bool IsSmokeFieldActive() const
    {
        return bSmokeFieldActive;
    }

    UFUNCTION()
    TArray<FUBotSensorMediumTestLegacyCell> GetSmokeCellSnapshots() const
    {
        return Cells;
    }
};

/**
 * Medium discovered through the Core IUBotPerceptionMedium interface. It also keeps the legacy reflection
 * API, like a smoke actor that adopted the interface without removing its old functions, so tests can
 * check that such an actor is never collected twice.
 */
UCLASS(Transient, NotBlueprintable, NotPlaceable, HideDropdown)
class AUBotSensorMediumTestInterfaceActor : public AActor, public IUBotPerceptionMedium
{
    GENERATED_BODY()

public:
    UPROPERTY()
    FUBotMediumSnapshot Snapshot;

    virtual bool GetMediumSnapshot_Implementation(FUBotMediumSnapshot& OutSnapshot) const override
    {
        OutSnapshot = Snapshot;
        return true;
    }

    UFUNCTION()
    TArray<FUBotSensorMediumTestLegacyCell> GetSmokeCellSnapshots() const
    {
        TArray<FUBotSensorMediumTestLegacyCell> LegacyCells;
        for (const FUBotMediumCell& Cell : Snapshot.Cells)
        {
            FUBotSensorMediumTestLegacyCell& LegacyCell = LegacyCells.AddDefaulted_GetRef();
            LegacyCell.WorldPosition = Cell.WorldPositionCentimeters;
            LegacyCell.Density = Cell.Density;
            LegacyCell.EdgeFactor = Cell.EdgeFactor;
        }
        return LegacyCells;
    }
};
