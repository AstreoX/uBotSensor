#pragma once

#include "CoreMinimal.h"
#include "Engine/EngineTypes.h"
#include "Math/RandomStream.h"
#include "UBotSensorComponent.h"
#include "UBotSensorTypes.h"
#include "UBotTypes.h"
#include "UBotOdometrySensorComponent.generated.h"

class USceneComponent;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FUBotOdometryDataEvent, const FUBotOdometrySensorData&, Data);

UCLASS(Blueprintable, ClassGroup = (uBot), meta = (BlueprintSpawnableComponent, DisplayName = "uBot Odometry Sensor"))
class UBOTSENSOR_API UUBotOdometrySensorComponent : public UUBotSensorComponent
{
    GENERATED_BODY()

public:
    UUBotOdometrySensorComponent();

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Odometry|Frames")
    FString OdometryFrameId = TEXT("odom");

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Odometry|Frames")
    FString ChildFrameId = TEXT("base_link");

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Odometry|Tracking")
    FComponentReference CustomTrackedComponent;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Odometry|Noise", meta = (ClampMin = "0.0"))
    float PositionNoiseStdDevMeters = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Odometry|Noise", meta = (ClampMin = "0.0"))
    float OrientationNoiseStdDevDegrees = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Odometry|Noise", meta = (ClampMin = "0.0"))
    float LinearVelocityNoiseStdDevMetersPerSecond = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Odometry|Noise", meta = (ClampMin = "0.0"))
    float AngularVelocityNoiseStdDevRadPerSecond = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Odometry|Drift", meta = (ClampMin = "0.0"))
    float PositionRandomWalkStdDevMetersPerSqrtSecond = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Odometry|Drift", meta = (ClampMin = "0.0"))
    float OrientationRandomWalkStdDevDegreesPerSqrtSecond = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Odometry|History", meta = (ClampMin = "0.01"))
    float TeleportDistanceMeters = 5.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Odometry|History", meta = (ClampMin = "1.0", ClampMax = "180.0"))
    float TeleportAngleDegrees = 120.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Odometry|History", meta = (ClampMin = "0.0001"))
    float MinimumValidDeltaSeconds = 0.0001f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Odometry|History", meta = (ClampMin = "0.01"))
    float MaximumValidDeltaSeconds = 0.5f;

    UPROPERTY(BlueprintAssignable, Category = "Odometry")
    FUBotOdometryDataEvent OnOdometryDataUpdated;

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|Odometry")
    FUBotOdometrySensorData GetLatestOdometryData() const { return LatestOdometryData; }

    UFUNCTION(BlueprintCallable, Category = "uBot Sensor|Odometry")
    bool SampleOdometry() { return SampleSensor(); }

    UFUNCTION(BlueprintCallable, Category = "uBot Sensor|Odometry")
    void ResetOdometry();

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|Odometry")
    USceneComponent* GetTrackedComponent() const;

    /** Returns "Odometry". */
    virtual FName GetSensorTypeName() const override;

protected:
    virtual bool CaptureSensorData(FUBotFrameHeader& Header, FString& OutErrorMessage) override;
    virtual void OnSamplingStarted() override;

private:
    void InitializeAtTransform(const FTransform& Transform, double TimestampSeconds, bool bSetOrigin);
    void FillCovariance(FUBotOdometrySensorData& Data) const;

    UPROPERTY(Transient)
    FUBotOdometrySensorData LatestOdometryData;

    FRandomStream RandomStream;
    FTransform OriginWorld = FTransform::Identity;
    FTransform PreviousWorld = FTransform::Identity;
    FVector PositionDriftMeters = FVector::ZeroVector;
    FVector OrientationDriftDegrees = FVector::ZeroVector;
    double PreviousTimestampSeconds = 0.0;
    bool bHasOrigin = false;
    bool bHasPreviousSample = false;
};
