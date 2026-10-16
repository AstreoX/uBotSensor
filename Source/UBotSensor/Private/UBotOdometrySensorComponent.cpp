#include "UBotOdometrySensorComponent.h"

#include "Components/SceneComponent.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/Actor.h"
#include "UBotSensorBodyMotion.h"
#include "UBotSensorMath.h"
#include "UBotUnits.h"
#include "UObject/ConstructorHelpers.h"

UUBotOdometrySensorComponent::UUBotOdometrySensorComponent()
{
    static ConstructorHelpers::FObjectFinder<UStaticMesh> SensorMesh(TEXT("/UBotSensor/Models/SM_UBotSensor_Odometry.SM_UBotSensor_Odometry"));
    if (SensorMesh.Succeeded())
    {
        SetStaticMesh(SensorMesh.Object);
    }
    SensorId = TEXT("odometry_sensor");
    FrameId = TEXT("odom");
    UpdateRateHz = 30.0f;
}

FName UUBotOdometrySensorComponent::GetSensorTypeName() const
{
    static const FName TypeName(TEXT("Odometry"));
    return TypeName;
}

bool UUBotOdometrySensorComponent::CaptureSensorData(FUBotFrameHeader& Header, FString& OutErrorMessage)
{
    namespace SensorBodyMotion = UBot::SensorBodyMotion;

    USceneComponent* TrackedComponent = GetTrackedComponent();
    bLatestSampleUsedBodyState = false;
    LatestBodyStateAgeSeconds = 0.0;
    LatestOdometryData = FUBotOdometrySensorData();
    LatestOdometryData.Header = Header;
    LatestOdometryData.OdometryFrameId = OdometryFrameId.IsEmpty() ? TEXT("odom") : OdometryFrameId;
    LatestOdometryData.ChildFrameId = ChildFrameId.IsEmpty() ? TEXT("base_link") : ChildFrameId;
    FillCovariance(LatestOdometryData);

    if (!TrackedComponent)
    {
        OutErrorMessage = TEXT("Odometry Sensor has no valid tracked component.");
        MarkHeader(Header, EUBotSampleStatus::Error, false);
        LatestOdometryData.Header = Header;
        OnOdometryDataUpdated.Broadcast(LatestOdometryData);
        return false;
    }

    // Where the velocities of this sample come from. State mode: the body state source that moves the
    // tracked component's body. Difference mode (no source, or bUseBodyState off): finite differences of
    // component poses, the only mode before 0.1.2. The pose is the component's in both modes.
    SensorBodyMotion::FPointMotion Motion;
    SensorBodyMotion::EReadResult ReadResult = SensorBodyMotion::EReadResult::NoSource;
    if (bUseBodyState)
    {
        ReadResult = SensorBodyMotion::Read(*TrackedComponent, Motion);
    }
    const bool bStateMode = ReadResult == SensorBodyMotion::EReadResult::Ok;
    if (bStateMode)
    {
        bLatestSampleUsedBodyState = true;
        LatestBodyStateAgeSeconds = Header.TimestampSeconds - Motion.StateTimeSeconds;
    }

    const FTransform CurrentWorld = TrackedComponent->GetComponentTransform();
    if (!bHasOrigin)
    {
        InitializeAtTransform(CurrentWorld, Header.TimestampSeconds, true);
        bPreviousSampleInStateMode = bStateMode;
        MarkHeader(Header, EUBotSampleStatus::WarmingUp, false);
        LatestOdometryData.Header = Header;
        OnOdometryDataUpdated.Broadcast(LatestOdometryData);
        return false;
    }

    const FTransform RelativePose = CurrentWorld.GetRelativeTransform(OriginWorld);
    LatestOdometryData.PositionMeters = UBot::Units::CentimetersToMeters(RelativePose.GetLocation());
    LatestOdometryData.OrientationQuaternion = RelativePose.GetRotation().GetNormalized();

    // The previous sample is also dropped when a provided body has no state yet and when the mode changed
    // since it (a source came or went, or bUseBodyState changed): the origin stays.
    if (!bHasPreviousSample || ReadResult == SensorBodyMotion::EReadResult::NoState || bPreviousSampleInStateMode != bStateMode)
    {
        InitializeAtTransform(CurrentWorld, Header.TimestampSeconds, false);
        bPreviousSampleInStateMode = bStateMode;
        MarkHeader(Header, EUBotSampleStatus::WarmingUp, false);
        LatestOdometryData.Header = Header;
        OnOdometryDataUpdated.Broadcast(LatestOdometryData);
        return false;
    }

    // Ends an invalid sample after which the odometry starts over at the current pose (new origin).
    const auto RestartAtCurrentPose = [&](EUBotSampleStatus Status)
    {
        InitializeAtTransform(CurrentWorld, Header.TimestampSeconds, true);
        bPreviousSampleInStateMode = bStateMode;
        LatestOdometryData = FUBotOdometrySensorData();
        LatestOdometryData.OdometryFrameId = OdometryFrameId.IsEmpty() ? TEXT("odom") : OdometryFrameId;
        LatestOdometryData.ChildFrameId = ChildFrameId.IsEmpty() ? TEXT("base_link") : ChildFrameId;
        FillCovariance(LatestOdometryData);
        MarkHeader(Header, Status, false);
        LatestOdometryData.Header = Header;
        OnOdometryDataUpdated.Broadcast(LatestOdometryData);
        return false;
    };

    const double DeltaSeconds = Header.TimestampSeconds - PreviousTimestampSeconds;
    if (DeltaSeconds < MinimumValidDeltaSeconds || DeltaSeconds > MaximumValidDeltaSeconds)
    {
        return RestartAtCurrentPose(EUBotSampleStatus::InvalidDeltaTime);
    }

    const float TravelMeters = UBot::Units::CentimetersToMeters(CurrentWorld.GetLocation() - PreviousWorld.GetLocation()).Size();
    const float RotationDegrees = FMath::RadiansToDegrees(PreviousWorld.GetRotation().AngularDistance(CurrentWorld.GetRotation()));
    if (TravelMeters > TeleportDistanceMeters || RotationDegrees > TeleportAngleDegrees)
    {
        return RestartAtCurrentPose(EUBotSampleStatus::TeleportDetected);
    }

    if (bStateMode)
    {
        // The state lies in the future or too far in the past: it does not describe this sample.
        if (!SensorBodyMotion::IsStateAgeAcceptable(LatestBodyStateAgeSeconds, MaximumValidDeltaSeconds))
        {
            return RestartAtCurrentPose(EUBotSampleStatus::InvalidDeltaTime);
        }

        LatestOdometryData.LinearVelocityLocalMetersPerSecond = CurrentWorld.GetRotation().UnrotateVector(Motion.LinearVelocity);
        LatestOdometryData.AngularVelocityLocalRadPerSecond = CurrentWorld.GetRotation().UnrotateVector(Motion.AngularVelocity);
    }
    else
    {
        const FVector LinearVelocityWorld = UBot::Units::CentimetersToMeters(CurrentWorld.GetLocation() - PreviousWorld.GetLocation()) / DeltaSeconds;
        const FVector AngularVelocityWorld = UBot::SensorMath::QuaternionDeltaToAngularVelocity(PreviousWorld.GetRotation(), CurrentWorld.GetRotation(), DeltaSeconds);
        LatestOdometryData.LinearVelocityLocalMetersPerSecond = CurrentWorld.GetRotation().UnrotateVector(LinearVelocityWorld);
        LatestOdometryData.AngularVelocityLocalRadPerSecond = CurrentWorld.GetRotation().UnrotateVector(AngularVelocityWorld);
    }

    if (bNoiseEnabled)
    {
        const float RandomWalkScale = FMath::Sqrt(static_cast<float>(DeltaSeconds));
        PositionDriftMeters += UBot::SensorMath::GaussianVector(RandomStream, PositionRandomWalkStdDevMetersPerSqrtSecond * RandomWalkScale);
        OrientationDriftDegrees += UBot::SensorMath::GaussianVector(RandomStream, OrientationRandomWalkStdDevDegreesPerSqrtSecond * RandomWalkScale);
        LatestOdometryData.PositionMeters += PositionDriftMeters + UBot::SensorMath::GaussianVector(RandomStream, PositionNoiseStdDevMeters);
        const FVector OrientationErrorDegrees = OrientationDriftDegrees + UBot::SensorMath::GaussianVector(RandomStream, OrientationNoiseStdDevDegrees);
        LatestOdometryData.OrientationQuaternion = (FQuat::MakeFromEuler(OrientationErrorDegrees) * LatestOdometryData.OrientationQuaternion).GetNormalized();
        LatestOdometryData.LinearVelocityLocalMetersPerSecond += UBot::SensorMath::GaussianVector(RandomStream, LinearVelocityNoiseStdDevMetersPerSecond);
        LatestOdometryData.AngularVelocityLocalRadPerSecond += UBot::SensorMath::GaussianVector(RandomStream, AngularVelocityNoiseStdDevRadPerSecond);
    }

    LatestOdometryData.bHasVelocity = true;
    PreviousWorld = CurrentWorld;
    PreviousTimestampSeconds = Header.TimestampSeconds;
    bPreviousSampleInStateMode = bStateMode;
    MarkHeader(Header, EUBotSampleStatus::Valid, true);
    LatestOdometryData.Header = Header;
    OnOdometryDataUpdated.Broadcast(LatestOdometryData);
    return true;
}

void UUBotOdometrySensorComponent::OnSamplingStarted()
{
    ResetOdometry();
}

void UUBotOdometrySensorComponent::ResetOdometry()
{
    RandomStream.Initialize(RandomSeed);
    PositionDriftMeters = FVector::ZeroVector;
    OrientationDriftDegrees = FVector::ZeroVector;
    bHasOrigin = false;
    bHasPreviousSample = false;
    PreviousTimestampSeconds = 0.0;
    // The state-mode history goes with it.
    bPreviousSampleInStateMode = false;
    if (USceneComponent* TrackedComponent = GetTrackedComponent())
    {
        OriginWorld = TrackedComponent->GetComponentTransform();
        bHasOrigin = true;
    }
}

USceneComponent* UUBotOdometrySensorComponent::GetTrackedComponent() const
{
    AActor* Owner = GetOwner();
    if (!Owner)
    {
        return nullptr;
    }
    if (USceneComponent* CustomComponent = Cast<USceneComponent>(CustomTrackedComponent.GetComponent(Owner)))
    {
        return CustomComponent;
    }
    return Owner->GetRootComponent();
}

void UUBotOdometrySensorComponent::InitializeAtTransform(const FTransform& Transform, double TimestampSeconds, bool bSetOrigin)
{
    if (bSetOrigin)
    {
        OriginWorld = Transform;
        PositionDriftMeters = FVector::ZeroVector;
        OrientationDriftDegrees = FVector::ZeroVector;
        bHasOrigin = true;
    }
    PreviousWorld = Transform;
    PreviousTimestampSeconds = TimestampSeconds;
    bHasPreviousSample = true;
}

void UUBotOdometrySensorComponent::FillCovariance(FUBotOdometrySensorData& Data) const
{
    const double PositionVariance = FMath::Square(static_cast<double>(PositionNoiseStdDevMeters));
    const double OrientationVariance = FMath::Square(FMath::DegreesToRadians(static_cast<double>(OrientationNoiseStdDevDegrees)));
    const double LinearVariance = FMath::Square(static_cast<double>(LinearVelocityNoiseStdDevMetersPerSecond));
    const double AngularVariance = FMath::Square(static_cast<double>(AngularVelocityNoiseStdDevRadPerSecond));
    Data.PoseCovariance = UBot::SensorMath::MakeDiagonalCovariance6(FVector(PositionVariance), FVector(OrientationVariance));
    Data.TwistCovariance = UBot::SensorMath::MakeDiagonalCovariance6(FVector(LinearVariance), FVector(AngularVariance));
}
