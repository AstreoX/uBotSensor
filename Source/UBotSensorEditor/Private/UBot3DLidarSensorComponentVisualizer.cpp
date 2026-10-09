#include "UBot3DLidarSensorComponentVisualizer.h"

#include "DynamicMeshBuilder.h"
#include "Engine/Canvas.h"
#include "Materials/MaterialInterface.h"
#include "SceneManagement.h"
#include "SceneView.h"
#include "UBot3DLidarSensorComponent.h"
#include "UBotLidarVisualizerUtils.h"
#include "UBotSensorMath.h"

namespace
{
    FVector LocalPoint(float Radius, float HorizontalDegrees, float VerticalDegrees)
    {
        return UBot::SensorMath::LidarDirectionLocal(HorizontalDegrees, VerticalDegrees) * Radius;
    }

    void DrawLatitude(
        FPrimitiveDrawInterface* PDI,
        const FTransform& Transform,
        float Radius,
        float VerticalDegrees,
        float StartHorizontalDegrees,
        float EndHorizontalDegrees,
        int32 Segments,
        const FLinearColor& Color,
        float Thickness)
    {
        FVector Previous = Transform.TransformPosition(LocalPoint(Radius, StartHorizontalDegrees, VerticalDegrees));
        for (int32 Segment = 1; Segment <= Segments; ++Segment)
        {
            const float Alpha = static_cast<float>(Segment) / Segments;
            const FVector Current = Transform.TransformPosition(LocalPoint(
                Radius,
                FMath::Lerp(StartHorizontalDegrees, EndHorizontalDegrees, Alpha),
                VerticalDegrees));
            PDI->DrawLine(Previous, Current, Color, SDPG_World, Thickness);
            Previous = Current;
        }
    }

    void DrawLongitude(
        FPrimitiveDrawInterface* PDI,
        const FTransform& Transform,
        float Radius,
        float HorizontalDegrees,
        float MinimumVerticalDegrees,
        float MaximumVerticalDegrees,
        int32 Segments,
        const FLinearColor& Color,
        float Thickness)
    {
        FVector Previous = Transform.TransformPosition(LocalPoint(Radius, HorizontalDegrees, MinimumVerticalDegrees));
        for (int32 Segment = 1; Segment <= Segments; ++Segment)
        {
            const float Alpha = static_cast<float>(Segment) / Segments;
            const FVector Current = Transform.TransformPosition(LocalPoint(
                Radius,
                HorizontalDegrees,
                FMath::Lerp(MinimumVerticalDegrees, MaximumVerticalDegrees, Alpha)));
            PDI->DrawLine(Previous, Current, Color, SDPG_World, Thickness);
            Previous = Current;
        }
    }
}

void FUBot3DLidarSensorComponentVisualizer::DrawVisualization(const UActorComponent* Component, const FSceneView* View, FPrimitiveDrawInterface* PDI)
{
    const UUBot3DLidarSensorComponent* Lidar = Cast<const UUBot3DLidarSensorComponent>(Component);
    if (!Lidar || !View || !PDI || Lidar->MaximumRangeMeters <= Lidar->MinimumRangeMeters)
    {
        return;
    }

    const FTransform Transform = Lidar->GetComponentTransform();
    const FVector Origin = Transform.GetLocation();
    const float MinimumRadius = FMath::Max(0.0f, Lidar->MinimumRangeMeters * 100.0f);
    const float MaximumRadius = Lidar->MaximumRangeMeters * 100.0f;
    const float HorizontalFov = FMath::Clamp(Lidar->HorizontalFovDegrees, 0.01f, 360.0f);
    const bool bFullHorizontalScan = HorizontalFov >= 359.999f;
    const float StartHorizontal = -0.5f * HorizontalFov;
    const float EndHorizontal = 0.5f * HorizontalFov;
    const float MinimumVertical = -FMath::Clamp(Lidar->VerticalFovDownDegrees, 0.0f, 90.0f);
    const float MaximumVertical = FMath::Clamp(Lidar->VerticalFovUpDegrees, 0.0f, 90.0f);
    const float VerticalSpan = MaximumVertical - MinimumVertical;
    if (VerticalSpan <= KINDA_SMALL_NUMBER)
    {
        return;
    }

    const int32 HorizontalSegments = FMath::Clamp(FMath::CeilToInt(HorizontalFov / 5.0f), 12, 96);
    const int32 VerticalSegments = FMath::Clamp(FMath::CeilToInt(VerticalSpan / 3.0f), 4, 32);
    const FColor FillColor = UBot::LidarVisualizer::MakeFillColor(Lidar->RangePreviewColor, Lidar->RangePreviewOpacity);
    const FLinearColor GridColor = UBot::LidarVisualizer::MakeGridColor(Lidar->RangePreviewColor, 0.7f);
    const FLinearColor MajorGridColor = UBot::LidarVisualizer::MakeGridColor(Lidar->RangePreviewColor, 1.05f);
    const FLinearColor BoundaryColor = UBot::LidarVisualizer::MakeGridColor(Lidar->RangePreviewColor, 1.45f);

    if (FillColor.A > 0)
    {
        if (UMaterialInterface* Material = UBot::LidarVisualizer::GetPreviewMaterial())
        {
            FDynamicMeshBuilder MeshBuilder(PDI->View->GetFeatureLevel());
            TArray<int32> OuterVertices;
            TArray<int32> InnerVertices;
            OuterVertices.SetNum((VerticalSegments + 1) * (HorizontalSegments + 1));
            InnerVertices.SetNum((VerticalSegments + 1) * (HorizontalSegments + 1));

            auto GridIndex = [HorizontalSegments](int32 VerticalIndex, int32 HorizontalIndex)
            {
                return VerticalIndex * (HorizontalSegments + 1) + HorizontalIndex;
            };

            for (int32 VerticalIndex = 0; VerticalIndex <= VerticalSegments; ++VerticalIndex)
            {
                const float VerticalAlpha = static_cast<float>(VerticalIndex) / VerticalSegments;
                const float VerticalDegrees = FMath::Lerp(MinimumVertical, MaximumVertical, VerticalAlpha);
                for (int32 HorizontalIndex = 0; HorizontalIndex <= HorizontalSegments; ++HorizontalIndex)
                {
                    const float HorizontalAlpha = static_cast<float>(HorizontalIndex) / HorizontalSegments;
                    const float HorizontalDegrees = FMath::Lerp(StartHorizontal, EndHorizontal, HorizontalAlpha);
                    const FVector Direction = UBot::SensorMath::LidarDirectionLocal(HorizontalDegrees, VerticalDegrees);
                    const int32 Index = GridIndex(VerticalIndex, HorizontalIndex);
                    OuterVertices[Index] = MeshBuilder.AddVertex(UBot::LidarVisualizer::MakeVertex(Direction * MaximumRadius, Direction, FillColor));
                    InnerVertices[Index] = MeshBuilder.AddVertex(UBot::LidarVisualizer::MakeVertex(Direction * MinimumRadius, -Direction, FillColor));
                }
            }

            for (int32 VerticalIndex = 0; VerticalIndex < VerticalSegments; ++VerticalIndex)
            {
                for (int32 HorizontalIndex = 0; HorizontalIndex < HorizontalSegments; ++HorizontalIndex)
                {
                    const int32 A = GridIndex(VerticalIndex, HorizontalIndex);
                    const int32 B = GridIndex(VerticalIndex, HorizontalIndex + 1);
                    const int32 C = GridIndex(VerticalIndex + 1, HorizontalIndex + 1);
                    const int32 D = GridIndex(VerticalIndex + 1, HorizontalIndex);
                    UBot::LidarVisualizer::AddDoubleSidedQuad(MeshBuilder, OuterVertices[A], OuterVertices[B], OuterVertices[C], OuterVertices[D]);
                    UBot::LidarVisualizer::AddDoubleSidedQuad(MeshBuilder, InnerVertices[D], InnerVertices[C], InnerVertices[B], InnerVertices[A]);
                }
            }

            for (int32 HorizontalIndex = 0; HorizontalIndex < HorizontalSegments; ++HorizontalIndex)
            {
                const int32 BottomA = GridIndex(0, HorizontalIndex);
                const int32 BottomB = GridIndex(0, HorizontalIndex + 1);
                UBot::LidarVisualizer::AddDoubleSidedQuad(
                    MeshBuilder,
                    InnerVertices[BottomA],
                    InnerVertices[BottomB],
                    OuterVertices[BottomB],
                    OuterVertices[BottomA]);

                const int32 TopA = GridIndex(VerticalSegments, HorizontalIndex);
                const int32 TopB = GridIndex(VerticalSegments, HorizontalIndex + 1);
                UBot::LidarVisualizer::AddDoubleSidedQuad(
                    MeshBuilder,
                    InnerVertices[TopB],
                    InnerVertices[TopA],
                    OuterVertices[TopA],
                    OuterVertices[TopB]);
            }

            if (!bFullHorizontalScan)
            {
                for (int32 VerticalIndex = 0; VerticalIndex < VerticalSegments; ++VerticalIndex)
                {
                    const int32 StartA = GridIndex(VerticalIndex, 0);
                    const int32 StartB = GridIndex(VerticalIndex + 1, 0);
                    UBot::LidarVisualizer::AddDoubleSidedQuad(
                        MeshBuilder,
                        InnerVertices[StartA],
                        OuterVertices[StartA],
                        OuterVertices[StartB],
                        InnerVertices[StartB]);

                    const int32 EndA = GridIndex(VerticalIndex, HorizontalSegments);
                    const int32 EndB = GridIndex(VerticalIndex + 1, HorizontalSegments);
                    UBot::LidarVisualizer::AddDoubleSidedQuad(
                        MeshBuilder,
                        InnerVertices[EndB],
                        OuterVertices[EndB],
                        OuterVertices[EndA],
                        InnerVertices[EndA]);
                }
            }

            MeshBuilder.Draw(PDI, Transform.ToMatrixWithScale(), Material->GetRenderProxy(), SDPG_World, 0.0f);
        }
    }

    const TArray<float> MajorDistances = UBot::LidarVisualizer::GetMajorDistances(
        Lidar->MinimumRangeMeters,
        Lidar->MaximumRangeMeters,
        Lidar->DistanceGridIntervalMeters);
    for (const float DistanceMeters : MajorDistances)
    {
        const float Radius = DistanceMeters * 100.0f;
        DrawLatitude(PDI, Transform, Radius, MinimumVertical, StartHorizontal, EndHorizontal, HorizontalSegments, MajorGridColor, 0.8f);
        DrawLatitude(PDI, Transform, Radius, 0.0f, StartHorizontal, EndHorizontal, HorizontalSegments, MajorGridColor, 1.0f);
        DrawLatitude(PDI, Transform, Radius, MaximumVertical, StartHorizontal, EndHorizontal, HorizontalSegments, MajorGridColor, 0.8f);
    }

    const float AngularInterval = FMath::Max(Lidar->AngularGridIntervalDegrees, 1.0f);
    int32 LongitudeCount = 0;
    float Horizontal = FMath::CeilToFloat(StartHorizontal / AngularInterval) * AngularInterval;
    for (; Horizontal < EndHorizontal - KINDA_SMALL_NUMBER && LongitudeCount < 48; Horizontal += AngularInterval, ++LongitudeCount)
    {
        DrawLongitude(PDI, Transform, MaximumRadius, Horizontal, MinimumVertical, MaximumVertical, VerticalSegments, GridColor, 0.75f);
    }

    int32 LatitudeCount = 0;
    float Vertical = FMath::CeilToFloat(MinimumVertical / AngularInterval) * AngularInterval;
    for (; Vertical < MaximumVertical - KINDA_SMALL_NUMBER && LatitudeCount < 24; Vertical += AngularInterval, ++LatitudeCount)
    {
        DrawLatitude(PDI, Transform, MaximumRadius, Vertical, StartHorizontal, EndHorizontal, HorizontalSegments, GridColor, 0.75f);
    }

    for (const float VerticalBoundary : {MinimumVertical, MaximumVertical})
    {
        DrawLatitude(PDI, Transform, MinimumRadius, VerticalBoundary, StartHorizontal, EndHorizontal, HorizontalSegments, GridColor, 0.8f);
        DrawLatitude(PDI, Transform, MaximumRadius, VerticalBoundary, StartHorizontal, EndHorizontal, HorizontalSegments, BoundaryColor, 1.8f);
    }
    DrawLatitude(PDI, Transform, MaximumRadius, 0.0f, StartHorizontal, EndHorizontal, HorizontalSegments, BoundaryColor, 1.5f);

    if (!bFullHorizontalScan)
    {
        DrawLongitude(PDI, Transform, MaximumRadius, StartHorizontal, MinimumVertical, MaximumVertical, VerticalSegments, BoundaryColor, 1.8f);
        DrawLongitude(PDI, Transform, MaximumRadius, EndHorizontal, MinimumVertical, MaximumVertical, VerticalSegments, BoundaryColor, 1.8f);
        for (const float HorizontalBoundary : {StartHorizontal, EndHorizontal})
        {
            for (const float VerticalBoundary : {MinimumVertical, MaximumVertical})
            {
                const FVector Direction = Transform.TransformVectorNoScale(UBot::SensorMath::LidarDirectionLocal(HorizontalBoundary, VerticalBoundary)).GetSafeNormal();
                PDI->DrawLine(Origin + Direction * MinimumRadius, Origin + Direction * MaximumRadius, BoundaryColor, SDPG_World, 1.4f);
            }
        }
    }

    PDI->DrawLine(Origin, Origin + Transform.GetUnitAxis(EAxis::X) * FMath::Min(MaximumRadius, 150.0f), FLinearColor::Red, SDPG_World, 2.0f);
    PDI->DrawLine(Origin, Origin + Transform.GetUnitAxis(EAxis::Y) * FMath::Min(MaximumRadius, 100.0f), FLinearColor::Green, SDPG_World, 1.5f);
    PDI->DrawLine(Origin, Origin + Transform.GetUnitAxis(EAxis::Z) * FMath::Min(MaximumRadius, 100.0f), FLinearColor::Blue, SDPG_World, 1.5f);
}

void FUBot3DLidarSensorComponentVisualizer::DrawVisualizationHUD(
    const UActorComponent* Component,
    const FViewport* Viewport,
    const FSceneView* View,
    FCanvas* Canvas)
{
    const UUBot3DLidarSensorComponent* Lidar = Cast<const UUBot3DLidarSensorComponent>(Component);
    if (!Lidar || !Lidar->bShowRangeLabels || !Viewport || !View || !Canvas || Lidar->MaximumRangeMeters <= Lidar->MinimumRangeMeters)
    {
        return;
    }

    const float HorizontalFov = FMath::Clamp(Lidar->HorizontalFovDegrees, 0.01f, 360.0f);
    const float StartHorizontal = -0.5f * HorizontalFov;
    const float EndHorizontal = 0.5f * HorizontalFov;
    const float LabelHorizontal = FMath::Clamp(0.0f, StartHorizontal, EndHorizontal);
    const float MinimumVertical = -FMath::Clamp(Lidar->VerticalFovDownDegrees, 0.0f, 90.0f);
    const float MaximumVertical = FMath::Clamp(Lidar->VerticalFovUpDegrees, 0.0f, 90.0f);
    const float LabelVertical = FMath::Clamp(0.0f, MinimumVertical, MaximumVertical);
    const FTransform Transform = Lidar->GetComponentTransform();
    const FLinearColor LabelColor = UBot::LidarVisualizer::MakeGridColor(Lidar->RangePreviewColor, 1.3f);

    for (const float DistanceMeters : UBot::LidarVisualizer::GetMajorDistances(
        Lidar->MinimumRangeMeters,
        Lidar->MaximumRangeMeters,
        Lidar->DistanceGridIntervalMeters))
    {
        UBot::LidarVisualizer::DrawDistanceLabel(
            Transform.TransformPosition(LocalPoint(DistanceMeters * 100.0f, LabelHorizontal, LabelVertical)),
            DistanceMeters,
            View,
            Canvas,
            LabelColor);
    }
    UBot::LidarVisualizer::DrawDistanceLabel(
        Transform.TransformPosition(LocalPoint(Lidar->MaximumRangeMeters * 100.0f, LabelHorizontal, LabelVertical)),
        Lidar->MaximumRangeMeters,
        View,
        Canvas,
        LabelColor);
}
