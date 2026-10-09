#include "UBotCameraCaptureCoordinatorComponent.h"

#include "Engine/World.h"
#include "UBotCameraCaptureSubsystem.h"
#include "UBotClock.h"

UUBotCameraCaptureCoordinatorComponent::UUBotCameraCaptureCoordinatorComponent()
{
    // Automatic captures run from this tick instead of a looping FTimerManager timer. The tick is only
    // enabled while the capture group is running. TG_PostPhysics matches the sensor components; the
    // legacy timer fired right after TG_PostPhysics, in the same frame.
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.bStartWithTickEnabled = false;
    PrimaryComponentTick.TickGroup = TG_PostPhysics;
    PrimaryComponentTick.bTickEvenWhenPaused = false;
}

void UUBotCameraCaptureCoordinatorComponent::BeginPlay()
{
    Super::BeginPlay();
    FString ErrorMessage;
    if (GetWorld())
    {
        if (UUBotCameraCaptureSubsystem* Subsystem = GetWorld()->GetSubsystem<UUBotCameraCaptureSubsystem>())
        {
            bRegistered = Subsystem->RegisterCoordinator(this, ErrorMessage);
        }
    }
    if (!bRegistered)
    {
        OnCaptureGroupError.Broadcast(ErrorMessage.IsEmpty() ? TEXT("Camera capture coordinator registration failed.") : ErrorMessage);
        return;
    }
    if (bAutoStart)
    {
        StartCaptureGroup();
    }
}

void UUBotCameraCaptureCoordinatorComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    StopCaptureGroup();
    if (bRegistered && GetWorld())
    {
        if (UUBotCameraCaptureSubsystem* Subsystem = GetWorld()->GetSubsystem<UUBotCameraCaptureSubsystem>())
        {
            Subsystem->UnregisterCoordinator(this);
        }
    }
    bRegistered = false;
    Super::EndPlay(EndPlayReason);
}

void UUBotCameraCaptureCoordinatorComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (!bRunning)
    {
        // The tick was enabled from outside (e.g. Activate); nothing to capture until StartCaptureGroup.
        SetComponentTickEnabled(false);
        return;
    }

    // At most one group capture per tick; due times skipped by long frames are dropped by the scheduler
    // instead of being captured in a burst that would immediately make the previous frames stale.
    if (CaptureScheduler.ConsumeDue(UBot::Clock::GetSimTimeSeconds(this)))
    {
        CaptureGroupOnce();
    }
}

bool UUBotCameraCaptureCoordinatorComponent::StartCaptureGroup()
{
    if (bRunning)
    {
        return true;
    }
    if (!bEnabled || !bRegistered || !GetWorld())
    {
        return false;
    }
    bRunning = true;
    // Same as the legacy looping timer (first delay == interval): no immediate capture, the first
    // automatic capture is due one interval from now.
    const double RateHz = FMath::Clamp(UpdateRateHz, 0.1f, 240.0f);
    CaptureScheduler.Reset(RateHz, UBot::Clock::GetSimTimeSeconds(this) + 1.0 / RateHz);
    SetComponentTickEnabled(true);
    return true;
}

void UUBotCameraCaptureCoordinatorComponent::StopCaptureGroup()
{
    SetComponentTickEnabled(false);
    bRunning = false;
}

bool UUBotCameraCaptureCoordinatorComponent::CaptureGroupOnce()
{
    if (!bEnabled || !bRegistered || !GetWorld())
    {
        return false;
    }
    UUBotCameraCaptureSubsystem* Subsystem = GetWorld()->GetSubsystem<UUBotCameraCaptureSubsystem>();
    if (!Subsystem)
    {
        return false;
    }
    FString ErrorMessage;
    const bool bSuccess = Subsystem->CaptureGroup(CaptureGroupId, NextCaptureSequence++, UBot::Clock::GetSimTimeSeconds(this), ErrorMessage);
    if (!bSuccess && !ErrorMessage.IsEmpty())
    {
        OnCaptureGroupError.Broadcast(ErrorMessage);
    }
    return bSuccess;
}

int32 UUBotCameraCaptureCoordinatorComponent::GetRegisteredCameraCount() const
{
    if (!GetWorld())
    {
        return 0;
    }
    const UUBotCameraCaptureSubsystem* Subsystem = GetWorld()->GetSubsystem<UUBotCameraCaptureSubsystem>();
    return Subsystem ? Subsystem->GetRegisteredCameraCount(CaptureGroupId) : 0;
}
