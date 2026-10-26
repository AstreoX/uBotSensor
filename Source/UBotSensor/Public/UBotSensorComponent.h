#pragma once

#include "CoreMinimal.h"
#include "Components/StaticMeshComponent.h"
#include "UBotSampleScheduler.h"
#include "UBotSensorInterface.h"
#include "UBotSensorTypes.h"
#include "UBotTypes.h"
#include "UBotSensorComponent.generated.h"

class USceneComponent;

DECLARE_DYNAMIC_MULTICAST_DELEGATE(FUBotSensorSimpleEvent);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FUBotSensorSampledEvent, const FUBotFrameHeader&, Header);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FUBotSensorErrorEvent, const FString&, ErrorMessage);

/**
 * Base class of every uBot sensor component. The component itself is the sensor frame and shows the
 * sensor model mesh.
 *
 * Default sampling: StartSensor takes one immediate sample, then the component tick (TG_PostPhysics) samples
 * whenever the FUBotSampleScheduler says a sample is due on the UpdateRateHz grid of uBot sim time.
 * At most one sample is taken per tick; due times skipped by long frames are counted by
 * GetDroppedSampleCount instead of being sampled in a burst with identical timestamps. The tick is
 * enabled only while the sensor runs, so subclasses must not clear PrimaryComponentTick.bCanEverTick.
 * Subclasses may opt into an explicitly timed path (IMU PhysicsTimeline). SampleSensor remains
 * the manual or externally triggered request entry point.
 */
UCLASS(Abstract, Blueprintable, ClassGroup = (uBot), meta = (BlueprintSpawnableComponent))
class UBOTSENSOR_API UUBotSensorComponent : public UStaticMeshComponent, public IUBotSensor
{
    GENERATED_BODY()

public:
    UUBotSensorComponent();

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sensor")
    FString SensorId;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sensor")
    FString FrameId;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sensor", meta = (ClampMin = "0.1", ClampMax = "240.0", UIMin = "0.1", UIMax = "120.0"))
    float UpdateRateHz = 10.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sensor")
    bool bAutoStart = true;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sensor")
    bool bSensorEnabled = true;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sensor|Debug")
    bool bDebugVisualization = false;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sensor|Model")
    bool bHideSensorModelInGame = true;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sensor|Noise")
    bool bNoiseEnabled = false;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Sensor|Noise")
    int32 RandomSeed = 1337;

    UPROPERTY(BlueprintAssignable, Category = "uBot Sensor")
    FUBotSensorSimpleEvent OnSensorStarted;

    UPROPERTY(BlueprintAssignable, Category = "uBot Sensor")
    FUBotSensorSimpleEvent OnSensorStopped;

    UPROPERTY(BlueprintAssignable, Category = "uBot Sensor")
    FUBotSensorSampledEvent OnSensorSampled;

    UPROPERTY(BlueprintAssignable, Category = "uBot Sensor")
    FUBotSensorErrorEvent OnSensorError;

    UFUNCTION(BlueprintCallable, Category = "uBot Sensor")
    bool StartSensor();

    UFUNCTION(BlueprintCallable, Category = "uBot Sensor")
    void StopSensor();

    UFUNCTION(BlueprintCallable, Category = "uBot Sensor")
    bool SampleSensor();

    /** Clamped to [0.1, 240] Hz. A running sensor keeps running; the next due sample keeps its time. */
    UFUNCTION(BlueprintCallable, Category = "uBot Sensor")
    void SetUpdateRateHz(float NewUpdateRateHz);

    /**
     * Native accessor. Deliberately not a UFUNCTION: IUBotSensor declares the BlueprintNativeEvent
     * IsSensorRunning, which Blueprints and reflection reach through the interface (its node is pure
     * and returns the same value). A UFUNCTION with the same name here would not compile with UHT.
     */
    bool IsSensorRunning() const { return bSensorRunning; }

    UFUNCTION(BlueprintPure, Category = "uBot Sensor")
    FUBotFrameHeader GetLatestFrameHeader() const { return LatestFrameHeader; }

    /** Scheduler grid times skipped by frame sampling or unavailable history (reset by StartSensor). */
    UFUNCTION(BlueprintPure, Category = "uBot Sensor")
    int64 GetDroppedSampleCount() const;

    UFUNCTION(BlueprintCallable, Category = "uBot Sensor|Model")
    void SetSensorModelVisibleInGame(bool bModelVisible);

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|Model")
    bool IsSensorModelVisibleInGame() const { return !bHideSensorModelInGame; }

    /**
     * Short stable type key reported through IUBotSensor::GetSensorType: "Pose", "Odometry", "IMU",
     * "Lidar2D", "Lidar3D", "RGBCamera" or "DepthCamera". The base class returns NAME_None.
     */
    virtual FName GetSensorTypeName() const;

    //~ Begin IUBotSensor Interface
    virtual FString GetSensorId_Implementation() const override;
    virtual FString GetSensorFrameId_Implementation() const override;
    virtual FName GetSensorType_Implementation() const override;
    virtual bool IsSensorRunning_Implementation() const override;
    virtual FUBotFrameHeader GetLatestSensorHeader_Implementation() const override;
    virtual USceneComponent* GetSensorFrameComponent_Implementation() const override;
    //~ End IUBotSensor Interface

    //~ Begin UActorComponent Interface
    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
    //~ End UActorComponent Interface

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual bool CaptureSensorData(FUBotFrameHeader& Header, FString& OutErrorMessage) PURE_VIRTUAL(UUBotSensorComponent::CaptureSensorData, return false;);
    virtual void OnSamplingStarted();
    virtual void OnSamplingStopped();
    virtual bool ShouldAutoStartSensor() const;
    virtual bool CanStartSensor() const;

    /** Internal scheduling hooks. Legacy sensors keep their immediate, once-per-frame behavior. */
    virtual bool WantsImmediateStartupSample() const { return true; }
    virtual bool SampleRequested();
    virtual void SampleDue(double NowSeconds);
    bool CaptureSampleAt(double TimestampSeconds);
    FUBotSampleScheduler& GetSampleScheduler() { return Scheduler; }

    FUBotFrameHeader CreateFrameHeader();
    void MarkHeader(FUBotFrameHeader& Header, EUBotSampleStatus Status, bool bValid) const;
    void OverrideHeaderTimestamp(FUBotFrameHeader& Header, double TimestampSeconds) const;

    /** uBot sim time (UBot::Clock::GetSimTimeSeconds); stamped into every header by CreateFrameHeader. */
    double GetSensorTimestampSeconds() const;

private:
    /** SensorId, or the component name when SensorId is empty. */
    FString ResolveSensorId() const;

    /** FrameId, or the resolved sensor id when FrameId is empty. */
    FString ResolveFrameId(const FString& ResolvedSensorId) const;

    UPROPERTY(Transient)
    FUBotFrameHeader LatestFrameHeader;

    FUBotSampleScheduler Scheduler;
    int64 NextSequence = 1;
    bool bSensorRunning = false;
};
