#include "UBotImuSensorComponent.h"

#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "UBotSensorMath.h"
#include "UBotUnits.h"
#include "UObject/ConstructorHelpers.h"

UUBotImuSensorComponent::UUBotImuSensorComponent()
{
    static ConstructorHelpers::FObjectFinder<UStaticMesh> SensorMesh(TEXT("/UBotSensor/Models/SM_UBotSensor_IMU.SM_UBotSensor_IMU"));
    if (SensorMesh.Succeeded())
    {
        SetStaticMesh(SensorMesh.Object);
    }
    SensorId = TEXT("imu_sensor");
    FrameId = TEXT("imu_sensor");
    UpdateRateHz = 100.0f;
}

FName UUBotImuSensorComponent::GetSensorTypeName() const
{
    static const FName TypeName(TEXT("IMU"));
    return TypeName;
}

bool UUBotImuSensorComponent::CaptureSensorData(FUBotFrameHeader& Header, FString& OutErrorMessage)
{
    const FVector CurrentPositionMeters = UBot::Units::CentimetersToMeters(GetComponentLocation());
    const FQuat CurrentOrientation = GetComponentQuat().GetNormalized();
    LatestImuData = FUBotImuSensorData();
    LatestImuData.Header = Header;
    LatestImuData.OrientationQuaternion = CurrentOrientation;
    LatestImuData.AngularVelocityVariance = FVector(GyroscopeNoiseStdDev * GyroscopeNoiseStdDev);
    LatestImuData.LinearAccelerationVariance = FVector(AccelerometerNoiseStdDev * AccelerometerNoiseStdDev);

    if (HistorySampleCount == 0)
    {
        PreviousPositionMeters = CurrentPositionMeters;
        PreviousOrientation = CurrentOrientation;
        PreviousTimestampSeconds = Header.TimestampSeconds;
        HistorySampleCount = 1;
        MarkHeader(Header, EUBotSampleStatus::WarmingUp, false);
        LatestImuData.Header = Header;
        OnImuDataUpdated.Broadcast(LatestImuData);
        return false;
    }

    const double DeltaSeconds = Header.TimestampSeconds - PreviousTimestampSeconds;
    if (DeltaSeconds <= UE_SMALL_NUMBER || DeltaSeconds > MaxSampleDeltaSeconds)
    {
        ResetIMUHistory();
        PreviousPositionMeters = CurrentPositionMeters;
        PreviousOrientation = CurrentOrientation;
        PreviousTimestampSeconds = Header.TimestampSeconds;
        HistorySampleCount = 1;
        MarkHeader(Header, EUBotSampleStatus::InvalidDeltaTime, false);
        LatestImuData.Header = Header;
        OnImuDataUpdated.Broadcast(LatestImuData);
        return false;
    }

    const double TranslationDistance = FVector::Distance(CurrentPositionMeters, PreviousPositionMeters);
    const double RotationAngleDegrees = FMath::RadiansToDegrees(CurrentOrientation.AngularDistance(PreviousOrientation));
    if (TranslationDistance > TeleportDistanceMeters || RotationAngleDegrees > TeleportAngleDegrees)
    {
        ResetIMUHistory();
        PreviousPositionMeters = CurrentPositionMeters;
        PreviousOrientation = CurrentOrientation;
        PreviousTimestampSeconds = Header.TimestampSeconds;
        HistorySampleCount = 1;
        MarkHeader(Header, EUBotSampleStatus::TeleportDetected, false);
        LatestImuData.Header = Header;
        OnImuDataUpdated.Broadcast(LatestImuData);
        return false;
    }

    const FVector CurrentVelocity = (CurrentPositionMeters - PreviousPositionMeters) / DeltaSeconds;
    const FVector AngularVelocityWorld = UBot::SensorMath::QuaternionDeltaToAngularVelocity(PreviousOrientation, CurrentOrientation, DeltaSeconds);
    if (HistorySampleCount == 1)
    {
        PreviousVelocityMetersPerSecond = CurrentVelocity;
        PreviousPositionMeters = CurrentPositionMeters;
        PreviousOrientation = CurrentOrientation;
        PreviousTimestampSeconds = Header.TimestampSeconds;
        HistorySampleCount = 2;
        MarkHeader(Header, EUBotSampleStatus::WarmingUp, false);
        LatestImuData.Header = Header;
        OnImuDataUpdated.Broadcast(LatestImuData);
        return false;
    }

    const FVector KinematicAccelerationWorld = (CurrentVelocity - PreviousVelocityMetersPerSecond) / DeltaSeconds;
    const FVector GravityMetersPerSecondSquared = GetWorld() ? UBot::Units::CentimetersToMeters(GetWorld()->GetGravityZ() * FVector::UpVector) : FVector(0.0, 0.0, -9.80665);
    FVector AccelerationLocal = CurrentOrientation.UnrotateVector(KinematicAccelerationWorld - GravityMetersPerSecondSquared);
    FVector AngularVelocityLocal = CurrentOrientation.UnrotateVector(AngularVelocityWorld);

    if (bNoiseEnabled)
    {
        const float RandomWalkScale = FMath::Sqrt(static_cast<float>(DeltaSeconds));
        CurrentAccelerometerBias += GaussianVector(AccelerometerBiasRandomWalkStdDev * RandomWalkScale);
        CurrentGyroscopeBias += GaussianVector(GyroscopeBiasRandomWalkStdDev * RandomWalkScale);
        AccelerationLocal += CurrentAccelerometerBias + GaussianVector(AccelerometerNoiseStdDev);
        AngularVelocityLocal += CurrentGyroscopeBias + GaussianVector(GyroscopeNoiseStdDev);
    }

    if (bLowPassFilterEnabled)
    {
        AccelerationLocal = ApplyLowPass(AccelerationLocal, PreviousFilteredAcceleration, DeltaSeconds);
        AngularVelocityLocal = ApplyLowPass(AngularVelocityLocal, PreviousFilteredAngularVelocity, DeltaSeconds);
    }

    if (bClampOutput)
    {
        AccelerationLocal = AccelerationLocal.GetClampedToMaxSize(MaxLinearAcceleration);
        AngularVelocityLocal = AngularVelocityLocal.GetClampedToMaxSize(MaxAngularVelocity);
    }

    PreviousFilteredAcceleration = AccelerationLocal;
    PreviousFilteredAngularVelocity = AngularVelocityLocal;
    PreviousVelocityMetersPerSecond = CurrentVelocity;
    PreviousPositionMeters = CurrentPositionMeters;
    PreviousOrientation = CurrentOrientation;
    PreviousTimestampSeconds = Header.TimestampSeconds;
    HistorySampleCount++;

    LatestImuData.AngularVelocityRadPerSecond = AngularVelocityLocal;
    LatestImuData.LinearAccelerationMetersPerSecondSquared = AccelerationLocal;
    MarkHeader(Header, EUBotSampleStatus::Valid, true);
    LatestImuData.Header = Header;
    OnImuDataUpdated.Broadcast(LatestImuData);
    return true;
}

void UUBotImuSensorComponent::OnSamplingStarted()
{
    RandomStream.Initialize(RandomSeed);
    ResetIMUBias();
    ResetIMUHistory();
}

void UUBotImuSensorComponent::ResetIMUBias()
{
    CurrentAccelerometerBias = AccelerometerBias;
    CurrentGyroscopeBias = GyroscopeBias;
}

void UUBotImuSensorComponent::ResetIMUHistory()
{
    PreviousPositionMeters = FVector::ZeroVector;
    PreviousOrientation = FQuat::Identity;
    PreviousVelocityMetersPerSecond = FVector::ZeroVector;
    PreviousFilteredAcceleration = FVector::ZeroVector;
    PreviousFilteredAngularVelocity = FVector::ZeroVector;
    PreviousTimestampSeconds = 0.0;
    HistorySampleCount = 0;
}

FVector UUBotImuSensorComponent::GaussianVector(float StandardDeviation)
{
    if (StandardDeviation <= 0.0f)
    {
        return FVector::ZeroVector;
    }
    return FVector(
        RandomStream.GetFraction() + RandomStream.GetFraction() + RandomStream.GetFraction() - 1.5f,
        RandomStream.GetFraction() + RandomStream.GetFraction() + RandomStream.GetFraction() - 1.5f,
        RandomStream.GetFraction() + RandomStream.GetFraction() + RandomStream.GetFraction() - 1.5f) * (StandardDeviation * 1.41421356f);
}

FVector UUBotImuSensorComponent::ApplyLowPass(const FVector& Input, const FVector& PreviousOutput, double DeltaSeconds) const
{
    const double Rc = 1.0 / (2.0 * UE_PI * FMath::Max(0.1f, LowPassCutoffHz));
    const double Alpha = DeltaSeconds / (Rc + DeltaSeconds);
    return FMath::Lerp(PreviousOutput, Input, Alpha);
}
