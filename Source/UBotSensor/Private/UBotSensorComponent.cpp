#include "UBotSensorComponent.h"

#include "Engine/EngineBaseTypes.h"
#include "Engine/World.h"
#include "UBotClock.h"

namespace UBot::SensorComponent::Private
{
    constexpr float MinimumUpdateRateHz = 0.1f;
    constexpr float MaximumUpdateRateHz = 240.0f;

    float ClampUpdateRateHz(float RateHz)
    {
        return FMath::Clamp(RateHz, MinimumUpdateRateHz, MaximumUpdateRateHz);
    }
}

UUBotSensorComponent::UUBotSensorComponent()
{
    // Sampling runs from the component tick, enabled only while the sensor runs (StartSensor/StopSensor).
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.bStartWithTickEnabled = false;
    PrimaryComponentTick.TickGroup = TG_PostPhysics;
    PrimaryComponentTick.bTickEvenWhenPaused = false;
    SetCollisionEnabled(ECollisionEnabled::NoCollision);
    SetGenerateOverlapEvents(false);
    SetCanEverAffectNavigation(false);
    SetCastShadow(false);
    SetReceivesDecals(false);
    SetHiddenInGame(false);
}

void UUBotSensorComponent::BeginPlay()
{
    Super::BeginPlay();
    SetHiddenInGame(bHideSensorModelInGame);
    if (bAutoStart && ShouldAutoStartSensor())
    {
        StartSensor();
    }
}

void UUBotSensorComponent::SetSensorModelVisibleInGame(bool bModelVisible)
{
    bHideSensorModelInGame = !bModelVisible;
    if (GetWorld() && GetWorld()->IsGameWorld())
    {
        SetHiddenInGame(!bModelVisible);
    }
}

void UUBotSensorComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    StopSensor();
    Super::EndPlay(EndPlayReason);
}

void UUBotSensorComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (!bSensorRunning)
    {
        // The tick was enabled from outside (e.g. Activate); nothing to sample until StartSensor.
        SetComponentTickEnabled(false);
        return;
    }

    if (Scheduler.ConsumeDue(UBot::Clock::GetSimTimeSeconds(this)))
    {
        SampleSensor();
    }
}

bool UUBotSensorComponent::StartSensor()
{
    if (bSensorRunning)
    {
        return true;
    }
    if (!bSensorEnabled || !GetWorld() || !CanStartSensor())
    {
        return false;
    }

    bSensorRunning = true;
    OnSamplingStarted();
    OnSensorStarted.Broadcast();
    SampleSensor();

    // A listener of the first sample may already have stopped the sensor again.
    if (bSensorRunning)
    {
        const double RateHz = UBot::SensorComponent::Private::ClampUpdateRateHz(UpdateRateHz);
        Scheduler.Reset(RateHz, UBot::Clock::GetSimTimeSeconds(this) + 1.0 / RateHz);
        SetComponentTickEnabled(true);
    }
    return true;
}

void UUBotSensorComponent::StopSensor()
{
    if (!bSensorRunning)
    {
        return;
    }
    SetComponentTickEnabled(false);
    bSensorRunning = false;
    OnSamplingStopped();
    OnSensorStopped.Broadcast();
}

bool UUBotSensorComponent::SampleSensor()
{
    FUBotFrameHeader Header = CreateFrameHeader();
    if (!bSensorEnabled)
    {
        MarkHeader(Header, EUBotSampleStatus::Disabled, false);
        LatestFrameHeader = Header;
        OnSensorSampled.Broadcast(Header);
        return false;
    }

    FString ErrorMessage;
    const bool bCaptured = CaptureSensorData(Header, ErrorMessage);
    LatestFrameHeader = Header;
    OnSensorSampled.Broadcast(Header);
    if (!ErrorMessage.IsEmpty())
    {
        OnSensorError.Broadcast(ErrorMessage);
    }
    return bCaptured && Header.bValid;
}

void UUBotSensorComponent::SetUpdateRateHz(float NewUpdateRateHz)
{
    UpdateRateHz = UBot::SensorComponent::Private::ClampUpdateRateHz(NewUpdateRateHz);
    if (bSensorRunning)
    {
        // Keeps the pending due time (phase); later samples follow the new interval.
        Scheduler.SetRateHz(UpdateRateHz);
    }
}

int64 UUBotSensorComponent::GetDroppedSampleCount() const
{
    return Scheduler.GetDroppedSampleCount();
}

FName UUBotSensorComponent::GetSensorTypeName() const
{
    return NAME_None;
}

FString UUBotSensorComponent::GetSensorId_Implementation() const
{
    return ResolveSensorId();
}

FString UUBotSensorComponent::GetSensorFrameId_Implementation() const
{
    return ResolveFrameId(ResolveSensorId());
}

FName UUBotSensorComponent::GetSensorType_Implementation() const
{
    return GetSensorTypeName();
}

bool UUBotSensorComponent::IsSensorRunning_Implementation() const
{
    return bSensorRunning;
}

FUBotFrameHeader UUBotSensorComponent::GetLatestSensorHeader_Implementation() const
{
    return LatestFrameHeader;
}

USceneComponent* UUBotSensorComponent::GetSensorFrameComponent_Implementation() const
{
    return const_cast<UUBotSensorComponent*>(this);
}

void UUBotSensorComponent::OnSamplingStarted()
{
}

void UUBotSensorComponent::OnSamplingStopped()
{
}

bool UUBotSensorComponent::ShouldAutoStartSensor() const
{
    return true;
}

bool UUBotSensorComponent::CanStartSensor() const
{
    return true;
}

FUBotFrameHeader UUBotSensorComponent::CreateFrameHeader()
{
    FUBotFrameHeader Header;
    Header.SensorId = ResolveSensorId();
    Header.FrameId = ResolveFrameId(Header.SensorId);
    Header.Sequence = NextSequence++;
    Header.TimestampSeconds = GetSensorTimestampSeconds();
    return Header;
}

void UUBotSensorComponent::MarkHeader(FUBotFrameHeader& Header, EUBotSampleStatus Status, bool bValid) const
{
    Header.Status = Status;
    Header.bValid = bValid;
}

void UUBotSensorComponent::OverrideHeaderTimestamp(FUBotFrameHeader& Header, double TimestampSeconds) const
{
    Header.TimestampSeconds = TimestampSeconds;
}

double UUBotSensorComponent::GetSensorTimestampSeconds() const
{
    return UBot::Clock::GetSimTimeSeconds(this);
}

FString UUBotSensorComponent::ResolveSensorId() const
{
    return SensorId.IsEmpty() ? GetName() : SensorId;
}

FString UUBotSensorComponent::ResolveFrameId(const FString& ResolvedSensorId) const
{
    return FrameId.IsEmpty() ? ResolvedSensorId : FrameId;
}
