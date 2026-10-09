#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "UBotSampleScheduler.h"
#include "UBotCameraCaptureCoordinatorComponent.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FUBotCameraCaptureGroupErrorEvent, const FString&, ErrorMessage);

/**
 * Captures every camera registered under CaptureGroupId in one call with a shared sequence number
 * and timestamp. While running, captures are triggered from this component's tick (TG_PostPhysics)
 * through FUBotSampleScheduler at UpdateRateHz, at most once per world tick; the first automatic
 * capture is due one interval after StartCaptureGroup.
 */
UCLASS(Blueprintable, ClassGroup = (uBot), meta = (BlueprintSpawnableComponent, DisplayName = "uBot Camera Capture Coordinator"))
class UBOTSENSOR_API UUBotCameraCaptureCoordinatorComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UUBotCameraCaptureCoordinatorComponent();

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Camera Capture Group")
    FString CaptureGroupId;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Camera Capture Group", meta = (ClampMin = "0.1", ClampMax = "240.0"))
    float UpdateRateHz = 30.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Camera Capture Group")
    bool bAutoStart = true;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Camera Capture Group")
    bool bEnabled = true;

    UPROPERTY(BlueprintAssignable, Category = "Camera Capture Group")
    FUBotCameraCaptureGroupErrorEvent OnCaptureGroupError;

    UFUNCTION(BlueprintCallable, Category = "uBot Sensor|Camera Capture Group")
    bool StartCaptureGroup();

    UFUNCTION(BlueprintCallable, Category = "uBot Sensor|Camera Capture Group")
    void StopCaptureGroup();

    UFUNCTION(BlueprintCallable, Category = "uBot Sensor|Camera Capture Group")
    bool CaptureGroupOnce();

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|Camera Capture Group")
    int32 GetRegisteredCameraCount() const;

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|Camera Capture Group")
    bool IsCaptureGroupRunning() const { return bRunning; }

    /** Due group captures skipped because a frame spanned more than one capture interval (reset by StartCaptureGroup). */
    UFUNCTION(BlueprintPure, Category = "uBot Sensor|Camera Capture Group")
    int64 GetDroppedCaptureCount() const { return CaptureScheduler.GetDroppedSampleCount(); }

    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
    /** Replaces the legacy looping capture timer; polled from TickComponent with uBot sim time. */
    FUBotSampleScheduler CaptureScheduler;
    int64 NextCaptureSequence = 1;
    bool bRegistered = false;
    bool bRunning = false;
};
