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

// Added in 0.1.2. Everything from here on follows all earlier members on purpose: released UBotROS binaries
// access this class at compiled offsets, so new members go at the very end and no virtual function is added.
public:
    /**
     * When a body state source (a physics backend binding, a vehicle's own integrator) moves the body this
     * sensor is attached to, read velocities and accelerations from that state instead of differentiating
     * poses. The parts between that body and the sensor count as rigid; turn this off for a sensor on a part
     * that moves on its own, such as a gimbal.
     */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "IMU|Motion")
    bool bUseBodyState = true;

    /** True when the latest sample took its motion from a body state source. */
    UFUNCTION(BlueprintPure, Category = "uBot Sensor|IMU")
    bool IsUsingBodyState() const { return bLatestSampleUsedBodyState; }

    /** Sample time minus the time of the body state the latest sample used (how far the physics step lay behind the sample), s; 0 when it used none. */
    UFUNCTION(BlueprintPure, Category = "uBot Sensor|IMU")
    double GetBodyStateAgeSeconds() const { return LatestBodyStateAgeSeconds; }

private:
    /**
     * Noise, bias random walk, low-pass filter and clamp of one valid sample, applied to the true values
     * (difference and state mode alike). DeltaSeconds is the interval between the headers of the two samples.
     */
    void PostProcessSample(FVector& InOutAccelerationLocal, FVector& InOutAngularVelocityLocal, double DeltaSeconds);

    /**
     * Records a body state in the state-mode history. When the state time moved forward since the previous
     * state, the body accelerations are derived from the velocity change over that time; otherwise the
     * last derived values are held.
     */
    void UpdateBodyStateHistory(double StateTimeSeconds, const FVector& BodyLinearVelocity, const FVector& BodyAngularVelocity);

    bool bLatestSampleUsedBodyState = false;
    double LatestBodyStateAgeSeconds = 0.0;

    // Mode the pose history above (Previous*, HistorySampleCount) was recorded in.
    bool bHistoryInStateMode = false;

    // State-mode history: the previous state's time and body velocities, and the body accelerations derived
    // from them for sources whose state carries none.
    bool bHasStateHistory = false;
    double PreviousStateTimeSeconds = 0.0;
    FVector PreviousBodyLinearVelocity = FVector::ZeroVector;
    FVector PreviousBodyAngularVelocity = FVector::ZeroVector;
    bool bHasDerivedBodyAccelerations = false;
    FVector DerivedBodyLinearAcceleration = FVector::ZeroVector;
    FVector DerivedBodyAngularAcceleration = FVector::ZeroVector;
};
