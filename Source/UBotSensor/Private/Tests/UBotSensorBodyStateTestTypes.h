#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Components/SceneComponent.h"
#include "UBotBodyState.h"
#include "UBotBodyStateHistory.h"
#include "UBotImuSensorComponent.h"
#include "UBotSensorBodyStateTestTypes.generated.h"

// Test-only reflected types for UBotSensor.BodyState.* (UHT cannot see types declared in a .cpp, and it does
// not allow WITH_DEV_AUTOMATION_TESTS guards, so these are compiled in every configuration).

// State source of one body: provides exactly ProvidedBody and reports State for it while bHasState is
// set. The tests write State and bHasState between samples to script what a physics backend or a vehicle
// integrator would report.
UCLASS(Transient, NotBlueprintable, HideDropdown)
class UUBotSensorTestStateSource : public UActorComponent, public IUBotBodyStateSource
{
    GENERATED_BODY()

public:
    TWeakObjectPtr<const USceneComponent> ProvidedBody;
    FUBotBodyState State;
    bool bHasState = true;
    TSharedPtr<const IUBotBodyStateHistory> History;
    virtual TSharedPtr<const IUBotBodyStateHistory> AcquireBodyStateHistory(const USceneComponent* Body) override
    {
        return ProvidesBodyState(Body) ? History : nullptr;
    }

    virtual bool ProvidesBodyState(const USceneComponent* Body) const override
    {
        return Body != nullptr && ProvidedBody.Get() == Body;
    }

    virtual bool GetBodyState(const USceneComponent* Body, FUBotBodyState& OutState) const override
    {
        if (!bHasState || !ProvidesBodyState(Body))
        {
            return false;
        }
        OutState = State;
        return true;
    }
};

UCLASS(Transient, NotBlueprintable, HideDropdown)
class UUBotImuTimelineRecorder : public UObject
{
    GENERATED_BODY()
public:
    TArray<FUBotImuSensorData> Samples;
    int32 Resets = 0;
    TWeakObjectPtr<UUBotImuSensorComponent> MoveSensorOnValidSample;
    TFunction<void()> OnNextValidSample;
    UFUNCTION() void OnSample(const FUBotImuSensorData& Sample)
    {
        Samples.Add(Sample);
        if (Sample.Header.bValid && OnNextValidSample)
        {
            TFunction<void()> Once = MoveTemp(OnNextValidSample);
            Once();
        }
        if (Sample.Header.bValid && MoveSensorOnValidSample.IsValid())
        {
            auto* Sensor = MoveSensorOnValidSample.Get();
            MoveSensorOnValidSample.Reset();
            Sensor->SetRelativeLocation(FVector(60, 0, 0));
        }
    }
    UFUNCTION() void OnReset() { ++Resets; }
};
