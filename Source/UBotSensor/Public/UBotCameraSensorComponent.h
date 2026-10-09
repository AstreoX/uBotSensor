#pragma once

#include "CoreMinimal.h"
#include "Engine/EngineTypes.h"
#include "Engine/TextureRenderTarget2D.h"
#include "UBotSensorComponent.h"
#include "UBotSensorTypes.h"
#include "UBotTypes.h"
#include "UBotCameraSensorComponent.generated.h"

class UUBotImageFrame;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FUBotImageFrameEvent, UUBotImageFrame*, Frame);

/**
 * Base class of the uBot cameras. Every sample renders the scene into the preview render target
 * (CaptureScene) and publishes a UUBotImageFrame that references it; pixels are read back to the CPU
 * only when a consumer calls UUBotImageFrame::RequestPixelReadback, and only while the frame is still
 * the latest capture (capture generation check).
 *
 * Triggering: a camera without CaptureGroupId samples from the base component tick
 * (TG_PostPhysics, FUBotSampleScheduler) at UpdateRateHz. A camera with a CaptureGroupId never starts
 * on its own; the group's UUBotCameraCaptureCoordinatorComponent captures it through
 * CaptureSynchronized.
 */
UCLASS(Abstract, Blueprintable, ClassGroup = (uBot))
class UBOTSENSOR_API UUBotCameraSensorComponent : public UUBotSensorComponent
{
    GENERATED_BODY()

public:
    UUBotCameraSensorComponent();

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Camera|Image", meta = (ClampMin = "16", ClampMax = "4096"))
    int32 ResolutionWidth = 640;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Camera|Image", meta = (ClampMin = "16", ClampMax = "4096"))
    int32 ResolutionHeight = 480;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Camera|Projection", meta = (ClampMin = "1.0", ClampMax = "170.0"))
    float HorizontalFovDegrees = 90.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Camera|Projection", meta = (ClampMin = "0.1"))
    float PreviewDistanceMeters = 10.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Camera|Synchronization")
    FString CaptureGroupId;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Camera|Render Target")
    TObjectPtr<UTextureRenderTarget2D> ExternalRenderTarget;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Camera|Smoke")
    bool bEnableSmokeInterference = true;

    /** Legacy tag of smoke actors discovered by reflection (kept for AgentCombatCore smoke actors). */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Camera|Smoke")
    FName SmokeVolumeActorTag = TEXT("AgentSensor.SmokeVolume");

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Camera|Smoke", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float SmokeDensityThreshold = 0.05f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Camera|Smoke", meta = (ClampMin = "0.01"))
    float SmokeCellRadiusScale = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Camera|Smoke")
    FLinearColor SmokeInterferenceTint = FLinearColor(0.48f, 0.50f, 0.52f, 1.0f);

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Camera|Smoke", meta = (ClampMin = "0.0"))
    float SmokeInterferenceOpacityScale = 0.75f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Camera|Smoke", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float MaximumSmokeInterferenceOpacity = 0.85f;

    UPROPERTY(BlueprintAssignable, Category = "Camera")
    FUBotImageFrameEvent OnImageFrameReady;

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|Camera")
    UUBotImageFrame* GetLatestImageFrame() const { return LatestImageFrame; }

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|Camera")
    UTextureRenderTarget2D* GetPreviewRenderTarget() const;

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|Camera")
    USceneCaptureComponent2D* GetSceneCaptureComponent() const { return SceneCaptureComponent; }

    /**
     * Synchronous CPU readback of Frame (flushes rendering). Fails when Frame belongs to another
     * camera or is stale, i.e. the render target has been captured again since Frame was produced.
     */
    bool ReadbackFrame(UUBotImageFrame* Frame, FString& ErrorMessage);

    /** Captures one frame for a capture group: the frame carries the group sequence and timestamp. */
    bool CaptureSynchronized(int64 GroupSequence, double GroupTimestampSeconds);

    /** Incremented by every CaptureScene of this camera. */
    uint64 GetCaptureGeneration() const { return CaptureGeneration; }

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual bool CaptureSensorData(FUBotFrameHeader& Header, FString& OutErrorMessage) override;
    virtual bool ShouldAutoStartSensor() const override;
    virtual bool CanStartSensor() const override;
    virtual EUBotImageEncoding GetImageEncoding() const PURE_VIRTUAL(UUBotCameraSensorComponent::GetImageEncoding, return EUBotImageEncoding::BGRA8;);
    virtual ETextureRenderTargetFormat GetRequiredRenderTargetFormat() const PURE_VIRTUAL(UUBotCameraSensorComponent::GetRequiredRenderTargetFormat, return RTF_RGBA8;);
    virtual ESceneCaptureSource GetCaptureSource() const PURE_VIRTUAL(UUBotCameraSensorComponent::GetCaptureSource, return SCS_FinalColorLDR;);
    virtual void ConfigureSceneCapture(USceneCaptureComponent2D& CaptureComponent);
    virtual bool ProcessReadback(UUBotImageFrame& Frame, FString& ErrorMessage);

private:
    // Lets the automation tests read the pending group values a capture coordinator hands to
    // CaptureSynchronized from a test camera, without a scene capture or GPU readback.
    friend struct FUBotCameraSensorTestAccess;

    bool EnsureCaptureResources(FString& ErrorMessage);
    bool EnsureSceneCaptureComponent(FString& ErrorMessage);
    bool ValidateRenderTarget(const UTextureRenderTarget2D& RenderTarget, FString& ErrorMessage) const;
    void RegisterWithCaptureGroup();
    void UnregisterFromCaptureGroup();
    void CheckCoordinatorAvailability();

    UPROPERTY(Transient)
    TObjectPtr<USceneCaptureComponent2D> SceneCaptureComponent;

    UPROPERTY(Transient)
    TObjectPtr<UTextureRenderTarget2D> AutomaticRenderTarget;

    UPROPERTY(Transient)
    TObjectPtr<UUBotImageFrame> LatestImageFrame;

    uint64 CaptureGeneration = 0;
    int64 PendingGroupSequence = 0;
    double PendingGroupTimestampSeconds = 0.0;
    bool bPendingSynchronizedCapture = false;
    bool bReportedMissingCoordinator = false;
};
