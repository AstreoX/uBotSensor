#include "UBotCameraSensorComponentVisualizer.h"

#include "SceneManagement.h"
#include "UBotCameraSensorComponent.h"
#include "UBotDepthCameraSensorComponent.h"
#include "UBotSensorMath.h"

void FUBotCameraSensorComponentVisualizer::DrawVisualization(const UActorComponent* Component, const FSceneView* View, FPrimitiveDrawInterface* PDI)
{
    const UUBotCameraSensorComponent* Camera = Cast<const UUBotCameraSensorComponent>(Component);
    if (!Camera || !PDI)
    {
        return;
    }
    const FTransform Transform = Camera->GetComponentTransform();
    const FVector Origin = Transform.GetLocation();
    const float Distance = FMath::Max(0.1f, Camera->PreviewDistanceMeters) * 100.0f;
    const float HalfHorizontal = FMath::DegreesToRadians(FMath::Clamp(Camera->HorizontalFovDegrees, 1.0f, 170.0f) * 0.5f);
    const float HalfVertical = UBot::SensorMath::CameraVerticalFovRadians(Camera->ResolutionWidth, Camera->ResolutionHeight, Camera->HorizontalFovDegrees) * 0.5f;
    const float HalfWidth = Distance * FMath::Tan(HalfHorizontal);
    const float HalfHeight = Distance * FMath::Tan(HalfVertical);
    const FLinearColor FrustumColor = Cast<UUBotDepthCameraSensorComponent>(Camera) ? FLinearColor(0.25f, 0.85f, 1.0f) : FLinearColor(1.0f, 0.65f, 0.15f);

    const FVector LocalCorners[] = {
        FVector(Distance, -HalfWidth, HalfHeight),
        FVector(Distance, HalfWidth, HalfHeight),
        FVector(Distance, HalfWidth, -HalfHeight),
        FVector(Distance, -HalfWidth, -HalfHeight)
    };
    FVector Corners[4];
    for (int32 Index = 0; Index < 4; ++Index)
    {
        Corners[Index] = Transform.TransformPosition(LocalCorners[Index]);
        PDI->DrawLine(Origin, Corners[Index], FrustumColor, SDPG_World, 1.0f);
        PDI->DrawLine(Corners[Index], Corners[(Index + 1) % 4], FrustumColor, SDPG_World, 1.5f);
    }
    PDI->DrawLine(Origin, Origin + Transform.GetUnitAxis(EAxis::X) * FMath::Min(Distance, 150.0f), FLinearColor::Red, SDPG_World, 2.0f);
    PDI->DrawLine(Origin, Origin + Transform.GetUnitAxis(EAxis::Y) * FMath::Min(Distance, 100.0f), FLinearColor::Green, SDPG_World, 1.5f);
    PDI->DrawLine(Origin, Origin + Transform.GetUnitAxis(EAxis::Z) * FMath::Min(Distance, 100.0f), FLinearColor::Blue, SDPG_World, 1.5f);
}
