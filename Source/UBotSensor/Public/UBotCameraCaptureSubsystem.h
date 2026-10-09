#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "UBotCameraCaptureSubsystem.generated.h"

class UUBotCameraCaptureCoordinatorComponent;
class UUBotCameraSensorComponent;

/** Per-world registry of camera capture groups: one coordinator and any number of cameras per group id. */
UCLASS()
class UBOTSENSOR_API UUBotCameraCaptureSubsystem : public UWorldSubsystem
{
    GENERATED_BODY()

public:
    bool RegisterCoordinator(UUBotCameraCaptureCoordinatorComponent* Coordinator, FString& ErrorMessage);
    void UnregisterCoordinator(UUBotCameraCaptureCoordinatorComponent* Coordinator);
    bool HasCoordinator(const FString& CaptureGroupId) const;
    void RegisterCamera(UUBotCameraSensorComponent* Camera);
    void UnregisterCamera(UUBotCameraSensorComponent* Camera);
    int32 GetRegisteredCameraCount(const FString& CaptureGroupId) const;

    /** Captures every live camera of the group in path-name order with the shared sequence and timestamp. */
    bool CaptureGroup(const FString& CaptureGroupId, int64 Sequence, double TimestampSeconds, FString& ErrorMessage);

private:
    TMap<FString, TWeakObjectPtr<UUBotCameraCaptureCoordinatorComponent>> Coordinators;
    TMap<FString, TArray<TWeakObjectPtr<UUBotCameraSensorComponent>>> CamerasByGroup;
};
