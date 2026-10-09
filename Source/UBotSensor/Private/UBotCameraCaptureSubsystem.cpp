#include "UBotCameraCaptureSubsystem.h"

#include "UBotCameraCaptureCoordinatorComponent.h"
#include "UBotCameraSensorComponent.h"

bool UUBotCameraCaptureSubsystem::RegisterCoordinator(UUBotCameraCaptureCoordinatorComponent* Coordinator, FString& ErrorMessage)
{
    if (!Coordinator || Coordinator->CaptureGroupId.IsEmpty())
    {
        ErrorMessage = TEXT("Camera capture coordinator requires a non-empty Capture Group Id.");
        return false;
    }
    if (const TWeakObjectPtr<UUBotCameraCaptureCoordinatorComponent>* Existing = Coordinators.Find(Coordinator->CaptureGroupId))
    {
        if (Existing->IsValid() && Existing->Get() != Coordinator)
        {
            ErrorMessage = FString::Printf(TEXT("Camera capture group '%s' already has a coordinator."), *Coordinator->CaptureGroupId);
            return false;
        }
    }
    Coordinators.Add(Coordinator->CaptureGroupId, Coordinator);
    return true;
}

void UUBotCameraCaptureSubsystem::UnregisterCoordinator(UUBotCameraCaptureCoordinatorComponent* Coordinator)
{
    if (!Coordinator)
    {
        return;
    }
    if (const TWeakObjectPtr<UUBotCameraCaptureCoordinatorComponent>* Existing = Coordinators.Find(Coordinator->CaptureGroupId))
    {
        if (Existing->Get() == Coordinator)
        {
            Coordinators.Remove(Coordinator->CaptureGroupId);
        }
    }
}

bool UUBotCameraCaptureSubsystem::HasCoordinator(const FString& CaptureGroupId) const
{
    const TWeakObjectPtr<UUBotCameraCaptureCoordinatorComponent>* Coordinator = Coordinators.Find(CaptureGroupId);
    return Coordinator && Coordinator->IsValid();
}

void UUBotCameraCaptureSubsystem::RegisterCamera(UUBotCameraSensorComponent* Camera)
{
    if (!Camera || Camera->CaptureGroupId.IsEmpty())
    {
        return;
    }
    TArray<TWeakObjectPtr<UUBotCameraSensorComponent>>& Cameras = CamerasByGroup.FindOrAdd(Camera->CaptureGroupId);
    Cameras.RemoveAll([](const TWeakObjectPtr<UUBotCameraSensorComponent>& Item) { return !Item.IsValid(); });
    if (!Cameras.Contains(Camera))
    {
        Cameras.Add(Camera);
    }
}

void UUBotCameraCaptureSubsystem::UnregisterCamera(UUBotCameraSensorComponent* Camera)
{
    if (!Camera)
    {
        return;
    }
    if (TArray<TWeakObjectPtr<UUBotCameraSensorComponent>>* Cameras = CamerasByGroup.Find(Camera->CaptureGroupId))
    {
        Cameras->RemoveAll([Camera](const TWeakObjectPtr<UUBotCameraSensorComponent>& Item) { return !Item.IsValid() || Item.Get() == Camera; });
        if (Cameras->IsEmpty())
        {
            CamerasByGroup.Remove(Camera->CaptureGroupId);
        }
    }
}

int32 UUBotCameraCaptureSubsystem::GetRegisteredCameraCount(const FString& CaptureGroupId) const
{
    const TArray<TWeakObjectPtr<UUBotCameraSensorComponent>>* Cameras = CamerasByGroup.Find(CaptureGroupId);
    if (!Cameras)
    {
        return 0;
    }
    int32 Count = 0;
    for (const TWeakObjectPtr<UUBotCameraSensorComponent>& Camera : *Cameras)
    {
        Count += Camera.IsValid() ? 1 : 0;
    }
    return Count;
}

bool UUBotCameraCaptureSubsystem::CaptureGroup(const FString& CaptureGroupId, int64 Sequence, double TimestampSeconds, FString& ErrorMessage)
{
    TArray<TWeakObjectPtr<UUBotCameraSensorComponent>>* Registered = CamerasByGroup.Find(CaptureGroupId);
    if (!Registered)
    {
        ErrorMessage = FString::Printf(TEXT("Camera capture group '%s' has no registered cameras."), *CaptureGroupId);
        return false;
    }
    Registered->RemoveAll([](const TWeakObjectPtr<UUBotCameraSensorComponent>& Item) { return !Item.IsValid(); });
    TArray<UUBotCameraSensorComponent*> Cameras;
    for (const TWeakObjectPtr<UUBotCameraSensorComponent>& Camera : *Registered)
    {
        if (Camera.IsValid())
        {
            Cameras.Add(Camera.Get());
        }
    }
    Cameras.Sort([](const UUBotCameraSensorComponent& A, const UUBotCameraSensorComponent& B) { return A.GetPathName() < B.GetPathName(); });
    bool bAllSucceeded = !Cameras.IsEmpty();
    for (UUBotCameraSensorComponent* Camera : Cameras)
    {
        bAllSucceeded &= Camera->CaptureSynchronized(Sequence, TimestampSeconds);
    }
    if (!bAllSucceeded && ErrorMessage.IsEmpty())
    {
        ErrorMessage = FString::Printf(TEXT("One or more cameras failed in capture group '%s'."), *CaptureGroupId);
    }
    return bAllSucceeded;
}
