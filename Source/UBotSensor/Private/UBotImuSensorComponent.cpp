#include "UBotImuSensorComponent.h"

#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "UBotBodyState.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"
#include "UBotSensorBodyMotion.h"
#include "UBotSensorMath.h"
#include "UBotUnits.h"
#include "UObject/ConstructorHelpers.h"

namespace UBot::SensorImu::Private
{
    // World gravity in m/s^2 as the IMU subtracts it; standard gravity when there is no world.
    FVector GravityMetersPerSecondSquared(const UWorld* World)
    {
        return World ? UBot::Units::CentimetersToMeters(World->GetGravityZ() * FVector::UpVector) : FVector(0.0, 0.0, -9.80665);
    }
}

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
    TRACE_CPUPROFILER_EVENT_SCOPE(UBot_IMU_Measurement);
    namespace SensorBodyMotion = UBot::SensorBodyMotion;

    // Where the motion of this sample comes from. State mode: the body state source that moves this
    // sensor's body. Difference mode (no source, or bUseBodyState off): finite differences of component
    // poses, the only mode before 0.1.2.
    bLatestSampleUsedBodyState = false;
    LatestBodyStateAgeSeconds = 0.0;
    SensorBodyMotion::FPointMotion Motion;
    SensorBodyMotion::EReadResult ReadResult = SensorBodyMotion::EReadResult::NoSource;
    if (bCapturingTimeline && TimelineBody.IsValid())
    {
        SensorBodyMotion::FromState(*this, *TimelineBody.Get(), PendingTimelineState, Motion);
        ReadResult = SensorBodyMotion::EReadResult::Ok;
    }
    else if (bUseBodyState)
    {
        ReadResult = SensorBodyMotion::Read(*this, Motion);
    }
    const bool bStateMode = ReadResult == SensorBodyMotion::EReadResult::Ok;
    bool bStateAgeValid = false;
    if (bStateMode)
    {
        bLatestSampleUsedBodyState = true;
        LatestBodyStateAgeSeconds = Header.TimestampSeconds - Motion.StateTimeSeconds;
        bStateAgeValid = SensorBodyMotion::IsStateAgeAcceptable(LatestBodyStateAgeSeconds, MaxSampleDeltaSeconds);
    }

    const FVector CurrentPositionMeters = bCapturingTimeline ? Motion.PositionMeters : UBot::Units::CentimetersToMeters(GetComponentLocation());
    const FQuat CurrentOrientation = bCapturingTimeline ? Motion.Orientation : GetComponentQuat().GetNormalized();
    LatestImuData = FUBotImuSensorData();
    LatestImuData.Header = Header;
    LatestImuData.OrientationQuaternion = bStateMode ? Motion.Orientation : CurrentOrientation;
    LatestImuData.AngularVelocityVariance = FVector(GyroscopeNoiseStdDev * GyroscopeNoiseStdDev);
    LatestImuData.LinearAccelerationVariance = FVector(AccelerometerNoiseStdDev * AccelerometerNoiseStdDev);

    // Ends an invalid sample: sets the status, reports the data and returns false.
    const auto FinishInvalid = [&](EUBotSampleStatus Status)
    {
        MarkHeader(Header, Status, false);
        LatestImuData.Header = Header;
        OnImuDataUpdated.Broadcast(LatestImuData);
        return false;
    };

    // Makes this sample the only entry of the history (the caller has reset the history, or it is empty).
    // A state that does not describe the sample time is not kept as the base of later derivations.
    const auto RestartHistory = [&]()
    {
        PreviousPositionMeters = CurrentPositionMeters;
        PreviousOrientation = CurrentOrientation;
        PreviousTimestampSeconds = Header.TimestampSeconds;
        HistorySampleCount = 1;
        bHistoryInStateMode = bStateMode;
        if (bStateMode && bStateAgeValid)
        {
            UpdateBodyStateHistory(Motion.StateTimeSeconds, Motion.BodyLinearVelocity, Motion.AngularVelocity);
        }
    };

    if (ReadResult == SensorBodyMotion::EReadResult::NoState)
    {
        // A body is provided, but its source has no state for it yet: nothing to read, and the history
        // belongs to no mode.
        ResetIMUHistory();
        return FinishInvalid(EUBotSampleStatus::WarmingUp);
    }

    if (HistorySampleCount > 0 && bHistoryInStateMode != bStateMode)
    {
        // A source came or went, or bUseBodyState changed: the history is of the other mode.
        ResetIMUHistory();
    }

    if (HistorySampleCount == 0)
    {
        RestartHistory();
        return FinishInvalid(EUBotSampleStatus::WarmingUp);
    }

    const double DeltaSeconds = Header.TimestampSeconds - PreviousTimestampSeconds;
    if (DeltaSeconds <= UE_SMALL_NUMBER || DeltaSeconds > MaxSampleDeltaSeconds)
    {
        ResetIMUHistory();
        RestartHistory();
        return FinishInvalid(EUBotSampleStatus::InvalidDeltaTime);
    }

    const double TranslationDistance = FVector::Distance(CurrentPositionMeters, PreviousPositionMeters);
    const double RotationAngleDegrees = FMath::RadiansToDegrees(CurrentOrientation.AngularDistance(PreviousOrientation));
    if (TranslationDistance > TeleportDistanceMeters || RotationAngleDegrees > TeleportAngleDegrees)
    {
        ResetIMUHistory();
        RestartHistory();
        return FinishInvalid(EUBotSampleStatus::TeleportDetected);
    }

    FVector AccelerationLocal = FVector::ZeroVector;
    FVector AngularVelocityLocal = FVector::ZeroVector;
    FVector CurrentVelocity = FVector::ZeroVector;
    if (bStateMode)
    {
        if (!bStateAgeValid)
        {
            // The state lies in the future or too far in the past: it does not describe this sample.
            ResetIMUHistory();
            RestartHistory();
            return FinishInvalid(EUBotSampleStatus::InvalidDeltaTime);
        }

        // Accelerations of the body origin: as the state gives them, else derived from the state history.
        UpdateBodyStateHistory(Motion.StateTimeSeconds, Motion.BodyLinearVelocity, Motion.AngularVelocity);
        const bool bHasLinear = Motion.bHasLinearAcceleration || bHasDerivedBodyAccelerations;
        const bool bHasAngular = Motion.bHasAngularAcceleration || bHasDerivedBodyAccelerations;
        if (!bHasLinear || !bHasAngular)
        {
            // Nothing to derive them from yet (needs a second state time): wait, keeping the history.
            PreviousPositionMeters = CurrentPositionMeters;
            PreviousOrientation = CurrentOrientation;
            PreviousTimestampSeconds = Header.TimestampSeconds;
            return FinishInvalid(EUBotSampleStatus::WarmingUp);
        }
        const FVector BodyLinearAcceleration = Motion.bHasLinearAcceleration ? Motion.BodyLinearAcceleration : DerivedBodyLinearAcceleration;
        const FVector BodyAngularAcceleration = Motion.bHasAngularAcceleration ? Motion.BodyAngularAcceleration : DerivedBodyAngularAcceleration;

        // Specific force and gyro of the sensor point at the state time, in the sensor frame at that time.
        const FVector KinematicAccelerationWorld = UBot::SensorMath::RigidPointAcceleration(BodyLinearAcceleration, Motion.AngularVelocity, BodyAngularAcceleration, Motion.OffsetWorldMeters);
        AccelerationLocal = Motion.Orientation.UnrotateVector(KinematicAccelerationWorld - UBot::SensorImu::Private::GravityMetersPerSecondSquared(GetWorld()));
        AngularVelocityLocal = Motion.Orientation.UnrotateVector(Motion.AngularVelocity);
    }
    else
    {
        CurrentVelocity = (CurrentPositionMeters - PreviousPositionMeters) / DeltaSeconds;
        const FVector AngularVelocityWorld = UBot::SensorMath::QuaternionDeltaToAngularVelocity(PreviousOrientation, CurrentOrientation, DeltaSeconds);
        if (HistorySampleCount == 1)
        {
            PreviousVelocityMetersPerSecond = CurrentVelocity;
            PreviousPositionMeters = CurrentPositionMeters;
            PreviousOrientation = CurrentOrientation;
            PreviousTimestampSeconds = Header.TimestampSeconds;
            HistorySampleCount = 2;
            return FinishInvalid(EUBotSampleStatus::WarmingUp);
        }

        const FVector KinematicAccelerationWorld = (CurrentVelocity - PreviousVelocityMetersPerSecond) / DeltaSeconds;
        AccelerationLocal = CurrentOrientation.UnrotateVector(KinematicAccelerationWorld - UBot::SensorImu::Private::GravityMetersPerSecondSquared(GetWorld()));
        AngularVelocityLocal = CurrentOrientation.UnrotateVector(AngularVelocityWorld);
    }

    PostProcessSample(AccelerationLocal, AngularVelocityLocal, DeltaSeconds);

    if (!bStateMode)
    {
        PreviousVelocityMetersPerSecond = CurrentVelocity;
    }
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

void UUBotImuSensorComponent::PostProcessSample(FVector& InOutAccelerationLocal, FVector& InOutAngularVelocityLocal, double DeltaSeconds)
{
    if (bNoiseEnabled)
    {
        const float RandomWalkScale = FMath::Sqrt(static_cast<float>(DeltaSeconds));
        CurrentAccelerometerBias += GaussianVector(AccelerometerBiasRandomWalkStdDev * RandomWalkScale);
        CurrentGyroscopeBias += GaussianVector(GyroscopeBiasRandomWalkStdDev * RandomWalkScale);
        InOutAccelerationLocal += CurrentAccelerometerBias + GaussianVector(AccelerometerNoiseStdDev);
        InOutAngularVelocityLocal += CurrentGyroscopeBias + GaussianVector(GyroscopeNoiseStdDev);
    }

    if (bLowPassFilterEnabled)
    {
        InOutAccelerationLocal = ApplyLowPass(InOutAccelerationLocal, PreviousFilteredAcceleration, DeltaSeconds);
        InOutAngularVelocityLocal = ApplyLowPass(InOutAngularVelocityLocal, PreviousFilteredAngularVelocity, DeltaSeconds);
    }

    if (bClampOutput)
    {
        InOutAccelerationLocal = InOutAccelerationLocal.GetClampedToMaxSize(MaxLinearAcceleration);
        InOutAngularVelocityLocal = InOutAngularVelocityLocal.GetClampedToMaxSize(MaxAngularVelocity);
    }

    PreviousFilteredAcceleration = InOutAccelerationLocal;
    PreviousFilteredAngularVelocity = InOutAngularVelocityLocal;
}

void UUBotImuSensorComponent::UpdateBodyStateHistory(double StateTimeSeconds, const FVector& BodyLinearVelocity, const FVector& BodyAngularVelocity)
{
    if (bHasStateHistory)
    {
        const double StateDeltaSeconds = StateTimeSeconds - PreviousStateTimeSeconds;
        if (StateDeltaSeconds <= UE_SMALL_NUMBER)
        {
            // The physics step did not advance since the previous state: keep the derived values.
            return;
        }
        DerivedBodyLinearAcceleration = (BodyLinearVelocity - PreviousBodyLinearVelocity) / StateDeltaSeconds;
        DerivedBodyAngularAcceleration = (BodyAngularVelocity - PreviousBodyAngularVelocity) / StateDeltaSeconds;
        bHasDerivedBodyAccelerations = true;
    }
    bHasStateHistory = true;
    PreviousStateTimeSeconds = StateTimeSeconds;
    PreviousBodyLinearVelocity = BodyLinearVelocity;
    PreviousBodyAngularVelocity = BodyAngularVelocity;
}

void UUBotImuSensorComponent::OnSamplingStarted()
{
    RandomStream.Initialize(RandomSeed);
    ResetIMUBias();
    ResetIMUHistory();
    TimelineHistoryDrops = TimelineUnavailable = 0;
    TimelineRebindDrops = 0;
    TimelineLastNow = -1.0;
    if (SamplingMode == EUBotImuSamplingMode::PhysicsTimeline) { RefreshTimelineSource(); }
}

bool UUBotImuSensorComponent::CanStartSensor() const
{
    if (SamplingMode != EUBotImuSamplingMode::PhysicsTimeline) { return Super::CanStartSensor(); }
    const USceneComponent* Body = nullptr;
    UObject* SourceObject = UBot::BodyState::FindSourceInParents(this, Body);
    IUBotBodyStateSource* Source = Cast<IUBotBodyStateSource>(SourceObject);
    if (!bUseBodyState || !Source || !Source->AcquireBodyStateHistory(Body))
    {
        UE_LOG(LogTemp, Warning, TEXT("%s: PhysicsTimeline IMU needs a body state history provider and Use Body State."), *GetPathName());
        return false;
    }
    return true;
}

bool UUBotImuSensorComponent::WantsImmediateStartupSample() const
{
    return SamplingMode == EUBotImuSamplingMode::FrameTick;
}

bool UUBotImuSensorComponent::SampleRequested()
{
    return SamplingMode == EUBotImuSamplingMode::PhysicsTimeline ? ProcessTimeline(1) > 0 : Super::SampleRequested();
}

void UUBotImuSensorComponent::SampleDue(double NowSeconds)
{
    if (SamplingMode == EUBotImuSamplingMode::PhysicsTimeline) { ProcessTimeline(MAX_int32); }
    else { Super::SampleDue(NowSeconds); }
}

void UUBotImuSensorComponent::ResetTimelineMeasurement(bool bClearPending)
{
    ResetIMUHistory();
    bHasTimelineSegment = false;
    if (bClearPending) { OnIMUTimelineReset.Broadcast(); }
}

void UUBotImuSensorComponent::ReleaseTimelineSource()
{
    if (UActorComponent* Component = Cast<UActorComponent>(TimelineSource.Get())) { RemoveTickPrerequisiteComponent(Component); }
    TimelineHistory.Reset();
    TimelineSource.Reset();
    TimelineBody.Reset();
    bHasTimelineGeneration = bHasTimelineSegment = false;
}

void UUBotImuSensorComponent::OnSamplingStopped()
{
    if (TimelineHistory) { ResetTimelineMeasurement(); }
    ReleaseTimelineSource();
    Super::OnSamplingStopped();
}

bool UUBotImuSensorComponent::RefreshTimelineSource()
{
    const USceneComponent* Body = nullptr;
    UObject* SourceObject = UBot::BodyState::FindSourceInParents(this, Body);
    if (!bUseBodyState || !SourceObject || !Body) { return false; }
    const FQuat BodyRotation = Body->GetComponentQuat().GetNormalized();
    const FTransform Mount((BodyRotation.Inverse() * GetComponentQuat()).GetNormalized(),
        BodyRotation.UnrotateVector(GetComponentLocation() - Body->GetComponentLocation()));
    if (TimelineSource.Get() != SourceObject || TimelineBody.Get() != Body || !TimelineHistory)
    {
        const bool bWasBound = TimelineHistory.IsValid();
        ReleaseTimelineSource();
        TimelineHistory = Cast<IUBotBodyStateSource>(SourceObject)->AcquireBodyStateHistory(Body);
        if (!TimelineHistory) { return false; }
        TimelineSource = SourceObject;
        TimelineBody = Body;
        TimelineMount = Mount;
        ++TimelineBindingRevision;
        if (bWasBound) { TimelineRebindDrops += GetSampleScheduler().SkipBefore(GetSensorTimestampSeconds()); }
        if (UActorComponent* Component = Cast<UActorComponent>(SourceObject)) { AddTickPrerequisiteComponent(Component); }
        ResetTimelineMeasurement();
    }
    else if (!Mount.Equals(TimelineMount, 1.e-6))
    {
        TimelineMount = Mount;
        ++TimelineBindingRevision;
        // The old mount at past due times is unknown. Do not apply new extrinsics retroactively.
        TimelineRebindDrops += GetSampleScheduler().SkipBefore(GetSensorTimestampSeconds());
        ResetTimelineMeasurement();
    }
    return true;
}

int32 UUBotImuSensorComponent::ProcessTimeline(int32 MaxSamples)
{
    if (bProcessingTimeline) { return 0; }
    TGuardValue<bool> ProcessingGuard(bProcessingTimeline, true);
    if (!IsSensorRunning() || !RefreshTimelineSource())
    {
        if (IsSensorRunning())
        {
            OnSensorError.Broadcast(TEXT("PhysicsTimeline IMU lost its history provider; sampling stopped."));
            StopSensor();
        }
        return 0;
    }
    const double Now = GetSensorTimestampSeconds();
    if (TimelineLastNow >= 0.0 && Now < TimelineLastNow)
    {
        GetSampleScheduler().Reset(UpdateRateHz, Now + 1.0 / FMath::Clamp(UpdateRateHz, 0.1f, 240.0f));
        ResetTimelineMeasurement();
    }
    TimelineLastNow = Now;
    FUBotBodyStateHistoryRange Range;
    int32 Captured = 0, Consumed = 0;
    const uint64 BindingRevision = TimelineBindingRevision;
    const auto RefreshAfterCallbacks = [&]()
    {
        if (!IsSensorRunning()) { return false; }
        if (!RefreshTimelineSource())
        {
            OnSensorError.Broadcast(TEXT("PhysicsTimeline IMU lost its history provider; sampling stopped."));
            StopSensor();
            return false;
        }
        return TimelineBindingRevision == BindingRevision;
    };
    double Due = 0.0;
    // A finite retained window bounds the amount of catch-up work. No current scene is reconstructed.
    while (Consumed < MaxSamples && IsSensorRunning() && TimelineHistory && TimelineHistory->GetRange(Range))
    {
        if (bHasTimelineGeneration && TimelineGeneration != Range.Generation)
        {
            ResetTimelineMeasurement();
            if (!RefreshAfterCallbacks()) { break; }
            // A reset listener may itself change provider history. Re-read before consuming a time.
            FUBotBodyStateHistoryRange AfterReset;
            if (!TimelineHistory->GetRange(AfterReset) || AfterReset.Generation != Range.Generation) { break; }
            Range = AfterReset;
        }
        TimelineGeneration = Range.Generation;
        bHasTimelineGeneration = true;
        TimelineHistoryDrops += GetSampleScheduler().SkipBefore(Range.OldestSeconds);
        Due = GetSampleScheduler().GetNextDueSeconds();
        if (!GetSampleScheduler().IsInitialized() || Due > FMath::Min(Now, Range.NewestSeconds)) { break; }
        uint64 Segment = 0;
        const EUBotBodyStateQueryResult Result = TimelineHistory->Query(Due, PendingTimelineState, Segment);
        if (Result == EUBotBodyStateQueryResult::NotReady) { break; }
        if (!GetSampleScheduler().ConsumeNextDue(FMath::Min(Now, Range.NewestSeconds), Due)) { break; }
        ++Consumed;
        if (Result != EUBotBodyStateQueryResult::Exact && Result != EUBotBodyStateQueryResult::Interpolated)
        {
            ++TimelineUnavailable;
            ResetTimelineMeasurement(false);
            continue;
        }
        if (bHasTimelineSegment && TimelineSegment != Segment) { ResetTimelineMeasurement(false); }
        if (!IsSensorRunning() || !TimelineHistory) { break; }
        TimelineSegment = Segment;
        bHasTimelineSegment = true;
        bCapturingTimeline = true;
        const bool bValid = CaptureSampleAt(Due);
        bCapturingTimeline = false;
        Captured += bValid ? 1 : 0;
        // A sample listener can move/reparent the sensor, change its provider or stop it.
        // Refresh before producing another historical sample with potentially different extrinsics.
        if (!RefreshAfterCallbacks()) { break; }
    }
    return Captured;
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

    // The state-mode history goes with it.
    bHistoryInStateMode = false;
    bHasStateHistory = false;
    PreviousStateTimeSeconds = 0.0;
    PreviousBodyLinearVelocity = FVector::ZeroVector;
    PreviousBodyAngularVelocity = FVector::ZeroVector;
    bHasDerivedBodyAccelerations = false;
    DerivedBodyLinearAcceleration = FVector::ZeroVector;
    DerivedBodyAngularAcceleration = FVector::ZeroVector;
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
