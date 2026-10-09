#pragma once

#include "CoreMinimal.h"
#include "UBotCameraSensorComponent.h"
#include "UBotDepthCameraSensorComponent.generated.h"

UCLASS(Blueprintable, ClassGroup = (uBot), meta = (BlueprintSpawnableComponent, DisplayName = "uBot Depth Camera Sensor"))
class UBOTSENSOR_API UUBotDepthCameraSensorComponent : public UUBotCameraSensorComponent
{
    GENERATED_BODY()

public:
    UUBotDepthCameraSensorComponent();

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Depth Camera|Range", meta = (ClampMin = "0.0"))
    float MinimumDepthMeters = 0.1f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Depth Camera|Range", meta = (ClampMin = "0.01"))
    float MaximumDepthMeters = 100.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Depth Camera|Preview", meta = (ClampMin = "0.0"))
    float PreviewMinimumDepthMeters = 0.1f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Depth Camera|Preview", meta = (ClampMin = "0.01"))
    float PreviewMaximumDepthMeters = 100.0f;

    UFUNCTION(BlueprintCallable, Category = "uBot Sensor|Depth Camera")
    bool SampleDepthCamera() { return SampleSensor(); }

    /** Returns "DepthCamera". */
    virtual FName GetSensorTypeName() const override;

protected:
    virtual EUBotImageEncoding GetImageEncoding() const override { return EUBotImageEncoding::Depth32FC1; }
    virtual ETextureRenderTargetFormat GetRequiredRenderTargetFormat() const override { return RTF_R32f; }
    virtual ESceneCaptureSource GetCaptureSource() const override { return SCS_SceneDepth; }
    virtual bool ProcessReadback(UUBotImageFrame& Frame, FString& ErrorMessage) override;
};
