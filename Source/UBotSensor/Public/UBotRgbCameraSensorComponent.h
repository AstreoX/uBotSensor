#pragma once

#include "CoreMinimal.h"
#include "UBotCameraSensorComponent.h"
#include "UBotRgbCameraSensorComponent.generated.h"

UCLASS(Blueprintable, ClassGroup = (uBot), meta = (BlueprintSpawnableComponent, DisplayName = "uBot RGB Camera Sensor"))
class UBOTSENSOR_API UUBotRgbCameraSensorComponent : public UUBotCameraSensorComponent
{
    GENERATED_BODY()

public:
    UUBotRgbCameraSensorComponent();

    UFUNCTION(BlueprintCallable, Category = "uBot Sensor|RGB Camera")
    bool SampleRGBCamera() { return SampleSensor(); }

    /** Returns "RGBCamera". */
    virtual FName GetSensorTypeName() const override;

protected:
    virtual EUBotImageEncoding GetImageEncoding() const override { return EUBotImageEncoding::BGRA8; }
    virtual ETextureRenderTargetFormat GetRequiredRenderTargetFormat() const override { return RTF_RGBA8; }
    virtual ESceneCaptureSource GetCaptureSource() const override { return SCS_FinalColorLDR; }
    virtual void ConfigureSceneCapture(USceneCaptureComponent2D& CaptureComponent) override;
};
