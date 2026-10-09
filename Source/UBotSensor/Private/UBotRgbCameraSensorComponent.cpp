#include "UBotRgbCameraSensorComponent.h"

#include "Components/SceneCaptureComponent2D.h"
#include "Engine/StaticMesh.h"
#include "UObject/ConstructorHelpers.h"

UUBotRgbCameraSensorComponent::UUBotRgbCameraSensorComponent()
{
    static ConstructorHelpers::FObjectFinder<UStaticMesh> SensorMesh(TEXT("/UBotSensor/Models/SM_UBotSensor_RGBCamera.SM_UBotSensor_RGBCamera"));
    if (SensorMesh.Succeeded())
    {
        SetStaticMesh(SensorMesh.Object);
    }
    SensorId = TEXT("rgb_camera");
    FrameId = TEXT("rgb_camera");
}

FName UUBotRgbCameraSensorComponent::GetSensorTypeName() const
{
    static const FName TypeName(TEXT("RGBCamera"));
    return TypeName;
}

void UUBotRgbCameraSensorComponent::ConfigureSceneCapture(USceneCaptureComponent2D& CaptureComponent)
{
    CaptureComponent.PostProcessBlendWeight = 1.0f;
    CaptureComponent.PostProcessSettings.bOverride_AutoExposureMethod = false;
}
