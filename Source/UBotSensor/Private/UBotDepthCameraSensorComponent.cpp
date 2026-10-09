#include "UBotDepthCameraSensorComponent.h"

#include "Engine/StaticMesh.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "TextureResource.h"
#include "UBotImageFrame.h"
#include "UBotPerceptionMedium.h"
#include "UBotSensorMath.h"
#include "UBotSensorMedium.h"
#include "UObject/ConstructorHelpers.h"

UUBotDepthCameraSensorComponent::UUBotDepthCameraSensorComponent()
{
    static ConstructorHelpers::FObjectFinder<UStaticMesh> SensorMesh(TEXT("/UBotSensor/Models/SM_UBotSensor_DepthCamera.SM_UBotSensor_DepthCamera"));
    if (SensorMesh.Succeeded())
    {
        SetStaticMesh(SensorMesh.Object);
    }
    SensorId = TEXT("depth_camera");
    FrameId = TEXT("depth_camera");
}

FName UUBotDepthCameraSensorComponent::GetSensorTypeName() const
{
    static const FName TypeName(TEXT("DepthCamera"));
    return TypeName;
}

bool UUBotDepthCameraSensorComponent::ProcessReadback(UUBotImageFrame& Frame, FString& ErrorMessage)
{
    if (MaximumDepthMeters <= MinimumDepthMeters)
    {
        ErrorMessage = TEXT("Depth camera measurement range is invalid.");
        return false;
    }
    FTextureRenderTargetResource* Resource = Frame.RawCaptureRenderTarget ? Frame.RawCaptureRenderTarget->GameThread_GetRenderTargetResource() : nullptr;
    if (!Resource)
    {
        ErrorMessage = TEXT("Depth camera render target resource is unavailable.");
        return false;
    }
    TArray<FLinearColor> LinearPixels;
    if (!Resource->ReadLinearColorPixels(LinearPixels) || LinearPixels.Num() != Frame.Width * Frame.Height)
    {
        ErrorMessage = TEXT("Depth camera pixel readback failed.");
        return false;
    }
    Frame.DepthValuesMeters.SetNumUninitialized(LinearPixels.Num());
    for (int32 Index = 0; Index < LinearPixels.Num(); ++Index)
    {
        Frame.DepthValuesMeters[Index] = UBot::SensorMath::FilterSceneDepthCentimeters(LinearPixels[Index].R, MinimumDepthMeters, MaximumDepthMeters);
    }
    if (bEnableSmokeInterference && GetWorld())
    {
        UBot::SensorMedium::FQuerySettings MediumSettings;
        MediumSettings.LegacySmokeVolumeTag = SmokeVolumeActorTag;
        MediumSettings.DensityThreshold = SmokeDensityThreshold;
        MediumSettings.CellRadiusScale = SmokeCellRadiusScale;

        TArray<FUBotMediumSnapshot> Media;
        UBot::SensorMedium::CollectMedia(*GetWorld(), MediumSettings, Media);
        UBot::SensorMedium::ApplyToDepthValues(
            Frame.SensorTransformWorld,
            Frame.Width,
            Frame.Height,
            Frame.Fx,
            Frame.Fy,
            Frame.Cx,
            Frame.Cy,
            MinimumDepthMeters,
            MaximumDepthMeters,
            Media,
            MediumSettings,
            Frame.DepthValuesMeters);
    }
    return true;
}
