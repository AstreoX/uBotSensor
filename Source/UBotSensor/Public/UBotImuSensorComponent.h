#pragma once

#include "CoreMinimal.h"
#include "Math/RandomStream.h"
#include "UBotSensorComponent.h"
#include "UBotSensorTypes.h"
#include "UBotTypes.h"
#include "UBotImuSensorComponent.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FUBotImuDataEvent, const FUBotImuSensorData&, Data);

UCLASS(Blueprintable, ClassGroup = (uBot), meta = (BlueprintSpawnableComponent, DisplayName = "uBot IMU Sensor"))
class UBOTSENSOR_API UUBotImuSensorComponent : public UUBotSensorComponent
{
    GENERATED_BODY()

public:
    UUBotImuSensorComponent();

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "IMU|Noise", meta = (ClampMin = "0.0"))
    float AccelerometerNoiseStdDev = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "IMU|Noise", meta = (ClampMin = "0.0"))
    float GyroscopeNoiseStdDev = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "IMU|Noise")
    FVector AccelerometerBias = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "IMU|Noise")
    FVector GyroscopeBias = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "IMU|Noise", meta = (ClampMin = "0.0"))
    float AccelerometerBiasRandomWalkStdDev = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "IMU|Noise", meta = (ClampMin = "0.0"))
    float GyroscopeBiasRandomWalkStdDev = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "IMU|Filter")
    bool bLowPassFilterEnabled = false;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "IMU|Filter", meta = (EditCondition = "bLowPassFilterEnabled", ClampMin = "0.1"))
    float LowPassCutoffHz = 20.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "IMU|Safety")
    bool bClampOutput = false;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "IMU|Safety", meta = (EditCondition = "bClampOutput", ClampMin = "0.1"))
    float MaxLinearAcceleration = 100.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "IMU|Safety", meta = (EditCondition = "bClampOutput", ClampMin = "0.1"))
    float MaxAngularVelocity = 50.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "IMU|History", meta = (ClampMin = "0.01"))
    float TeleportDistanceMeters = 5.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "IMU|History", meta = (ClampMin = "1.0", ClampMax = "180.0"))
    float TeleportAngleDegrees = 120.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "IMU|History", meta = (ClampMin = "0.01"))
    float MaxSampleDeltaSeconds = 0.5f;

    UPROPERTY(BlueprintAssignable, Category = "IMU")
    FUBotImuDataEvent OnImuDataUpdated;

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|IMU")
    FUBotImuSensorData GetLatestIMUData() const { return LatestImuData; }

    UFUNCTION(BlueprintCallable, Category = "uBot Sensor|IMU")
    bool SampleIMU() { return SampleSensor(); }

    UFUNCTION(BlueprintCallable, Category = "uBot Sensor|IMU")
    void ResetIMUBias();

    UFUNCTION(BlueprintCallable, Category = "uBot Sensor|IMU")
    void ResetIMUHistory();

    /** Returns "IMU". */
    virtual FName GetSensorTypeName() const override;

protected:
    virtual bool CaptureSensorData(FUBotFrameHeader& Header, FString& OutErrorMessage) override;
    virtual void OnSamplingStarted() override;

private:
    FVector GaussianVector(float StandardDeviation);
    FVector ApplyLowPass(const FVector& Input, const FVector& PreviousOutput, double DeltaSeconds) const;

    UPROPERTY(Transient)
    FUBotImuSensorData LatestImuData;

    FRandomStream RandomStream;
    FVector CurrentAccelerometerBias = FVector::ZeroVector;
    FVector CurrentGyroscopeBias = FVector::ZeroVector;
    FVector PreviousPositionMeters = FVector::ZeroVector;
    FQuat PreviousOrientation = FQuat::Identity;
    FVector PreviousVelocityMetersPerSecond = FVector::ZeroVector;
    FVector PreviousFilteredAcceleration = FVector::ZeroVector;
    FVector PreviousFilteredAngularVelocity = FVector::ZeroVector;
    double PreviousTimestampSeconds = 0.0;
    int32 HistorySampleCount = 0;
};
