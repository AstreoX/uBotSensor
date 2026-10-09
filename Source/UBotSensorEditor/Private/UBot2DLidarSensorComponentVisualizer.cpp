#include "UBot2DLidarSensorComponentVisualizer.h"

#include "DynamicMeshBuilder.h"
#include "Engine/Canvas.h"
#include "Materials/MaterialInterface.h"
#include "SceneManagement.h"
#include "SceneView.h"
#include "UBot2DLidarSensorComponent.h"
#include "UBotLidarVisualizerUtils.h"

namespace
{
    FVector LocalDirection(float AngleDegrees)
    {
        const float AngleRadians = FMath::DegreesToRadians(AngleDegrees);
        return FVector(FMath::Cos(AngleRadians), FMath::Sin(AngleRadians), 0.0f);
    }

    void DrawArc(
        FPrimitiveDrawInterface* PDI,
        const FTransform& Transform,
        float Radius,
        float StartAngleDegrees,
        float EndAngleDegrees,
        int32 Segments,
        const FLinearColor& Color,
        float Thickness)
    {
        FVector Previous = Transform.TransformPosition(LocalDirection(StartAngleDegrees) * Radius);
        for (int32 Segment = 1; Segment <= Segments; ++Segment)
        {
            const float Alpha = static_cast<float>(Segment) / Segments;
            const FVector Current = Transform.TransformPosition(LocalDirection(FMath::Lerp(StartAngleDegrees, EndAngleDegrees, Alpha)) * Radius);
            PDI->DrawLine(Previous, Current, Color, SDPG_World, Thickness);
            Previous = Current;
        }
    }
}

void FUBot2DLidarSensorComponentVisualizer::DrawVisualization(const UActorComponent* Component, const FSceneView* View, FPrimitiveDrawInterface* PDI)
{
    const UUBot2DLidarSensorComponent* Lidar = Cast<const UUBot2DLidarSensorComponent>(Component);
    if (!Lidar || !View || !PDI || Lidar->StartAngleDegrees >= Lidar->EndAngleDegrees)
    {
        return;
    }

    const FTransform Transform = Lidar->GetComponentTransform();
    const FVector Origin = Transform.GetLocation();
    const float MinimumRadius = FMath::Max(0.0f, Lidar->MinimumRangeMeters * 100.0f);
    const float MaximumRadius = FMath::Max(MinimumRadius, Lidar->MaximumRangeMeters * 100.0f);
    if (MaximumRadius <= MinimumRadius + KINDA_SMALL_NUMBER)
    {
        return;
    }

    const float AngularSpan = Lidar->EndAngleDegrees - Lidar->StartAngleDegrees;
    const int32 ArcSegments = FMath::Clamp(FMath::CeilToInt(AngularSpan / 3.0f), 16, 128);
    const FColor FillColor = UBot::LidarVisualizer::MakeFillColor(Lidar->RangePreviewColor, Lidar->RangePreviewOpacity);
    const FLinearColor GridColor = UBot::LidarVisualizer::MakeGridColor(Lidar->RangePreviewColor, 0.75f);
    const FLinearColor MajorGridColor = UBot::LidarVisualizer::MakeGridColor(Lidar->RangePreviewColor, 1.15f);
    const FLinearColor BoundaryColor = UBot::LidarVisualizer::MakeGridColor(Lidar->RangePreviewColor, 1.45f);

    if (FillColor.A > 0)
    {
        if (UMaterialInterface* Material = UBot::LidarVisualizer::GetPreviewMaterial())
        {
            FDynamicMeshBuilder MeshBuilder(PDI->View->GetFeatureLevel());
            for (int32 Segment = 0; Segment <= ArcSegments; ++Segment)
            {
                const float Alpha = static_cast<float>(Segment) / ArcSegments;
                const FVector Direction = LocalDirection(FMath::Lerp(Lidar->StartAngleDegrees, Lidar->EndAngleDegrees, Alpha));
                MeshBuilder.AddVertex(UBot::LidarVisualizer::MakeVertex(Direction * MinimumRadius, FVector::UpVector, FillColor));
                MeshBuilder.AddVertex(UBot::LidarVisualizer::MakeVertex(Direction * MaximumRadius, FVector::UpVector, FillColor));
            }
            for (int32 Segment = 0; Segment < ArcSegments; ++Segment)
            {
                const int32 Base = Segment * 2;
                UBot::LidarVisualizer::AddDoubleSidedQuad(MeshBuilder, Base, Base + 1, Base + 3, Base + 2);
            }
            MeshBuilder.Draw(PDI, Transform.ToMatrixWithScale(), Material->GetRenderProxy(), SDPG_World, 0.0f);
        }
    }

    DrawArc(PDI, Transform, MinimumRadius, Lidar->StartAngleDegrees, Lidar->EndAngleDegrees, ArcSegments, GridColor, 1.0f);
    DrawArc(PDI, Transform, MaximumRadius, Lidar->StartAngleDegrees, Lidar->EndAngleDegrees, ArcSegments, BoundaryColor, 2.0f);

    for (const float DistanceMeters : UBot::LidarVisualizer::GetMajorDistances(Lidar->MinimumRangeMeters, Lidar->MaximumRangeMeters, Lidar->DistanceGridIntervalMeters))
    {
        DrawArc(PDI, Transform, DistanceMeters * 100.0f, Lidar->StartAngleDegrees, Lidar->EndAngleDegrees, ArcSegments, MajorGridColor, 1.0f);
    }

    const float AngularInterval = FMath::Max(Lidar->AngularGridIntervalDegrees, 1.0f);
    const int32 MaxSpokes = 48;
    int32 SpokeCount = 0;
    float Angle = FMath::CeilToFloat(Lidar->StartAngleDegrees / AngularInterval) * AngularInterval;
    for (; Angle < Lidar->EndAngleDegrees - KINDA_SMALL_NUMBER && SpokeCount < MaxSpokes; Angle += AngularInterval, ++SpokeCount)
    {
        const FVector Direction = Transform.TransformVectorNoScale(LocalDirection(Angle)).GetSafeNormal();
        PDI->DrawLine(Origin + Direction * MinimumRadius, Origin + Direction * MaximumRadius, GridColor, SDPG_World, 0.75f);
    }

    for (const float BoundaryAngle : {Lidar->StartAngleDegrees, Lidar->EndAngleDegrees})
    {
        const FVector Direction = Transform.TransformVectorNoScale(LocalDirection(BoundaryAngle)).GetSafeNormal();
        PDI->DrawLine(Origin + Direction * MinimumRadius, Origin + Direction * MaximumRadius, BoundaryColor, SDPG_World, 2.0f);
    }

    PDI->DrawLine(Origin, Origin + Transform.GetUnitAxis(EAxis::X) * FMath::Min(MaximumRadius, 150.0f), FLinearColor::Red, SDPG_World, 2.0f);
}

void FUBot2DLidarSensorComponentVisualizer::DrawVisualizationHUD(
    const UActorComponent* Component,
    const FViewport* Viewport,
    const FSceneView* View,
    FCanvas* Canvas)
{
    const UUBot2DLidarSensorComponent* Lidar = Cast<const UUBot2DLidarSensorComponent>(Component);
    if (!Lidar || !Lidar->bShowRangeLabels || !Viewport || !View || !Canvas || Lidar->StartAngleDegrees >= Lidar->EndAngleDegrees)
    {
        return;
    }

    const float LabelAngle = Lidar->StartAngleDegrees <= 0.0f && Lidar->EndAngleDegrees >= 0.0f
        ? 0.0f
        : 0.5f * (Lidar->StartAngleDegrees + Lidar->EndAngleDegrees);
    const FTransform Transform = Lidar->GetComponentTransform();
    const FLinearColor LabelColor = UBot::LidarVisualizer::MakeGridColor(Lidar->RangePreviewColor, 1.3f);
    for (const float DistanceMeters : UBot::LidarVisualizer::GetMajorDistances(Lidar->MinimumRangeMeters, Lidar->MaximumRangeMeters, Lidar->DistanceGridIntervalMeters))
    {
        UBot::LidarVisualizer::DrawDistanceLabel(
            Transform.TransformPosition(LocalDirection(LabelAngle) * DistanceMeters * 100.0f),
            DistanceMeters,
            View,
            Canvas,
            LabelColor);
    }
    UBot::LidarVisualizer::DrawDistanceLabel(
        Transform.TransformPosition(LocalDirection(LabelAngle) * Lidar->MaximumRangeMeters * 100.0f),
        Lidar->MaximumRangeMeters,
        View,
        Canvas,
        LabelColor);
}
