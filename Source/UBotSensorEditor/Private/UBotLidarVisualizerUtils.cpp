#include "UBotLidarVisualizerUtils.h"

#include "CanvasTypes.h"
#include "DynamicMeshBuilder.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Materials/MaterialInterface.h"
#include "SceneView.h"
#include "UObject/UObjectGlobals.h"

namespace UBot::LidarVisualizer
{
    UMaterialInterface* GetPreviewMaterial()
    {
        static UMaterialInterface* Material = LoadObject<UMaterialInterface>(
            nullptr,
            TEXT("/Engine/EditorMaterials/WidgetVertexColorMaterial.WidgetVertexColorMaterial"));
        return Material;
    }

    FColor MakeFillColor(const FLinearColor& Color, float Opacity)
    {
        FColor Result = Color.ToFColor(true);
        Result.A = static_cast<uint8>(FMath::RoundToInt(FMath::Clamp(Opacity, 0.0f, 1.0f) * 255.0f));
        return Result;
    }

    FLinearColor MakeGridColor(const FLinearColor& Color, float Scale)
    {
        FLinearColor Result = Color * Scale;
        Result.A = 1.0f;
        return Result;
    }

    float GetAdaptiveDistanceInterval(float RequestedIntervalMeters, float MinimumRangeMeters, float MaximumRangeMeters)
    {
        float Interval = FMath::Max(RequestedIntervalMeters, 0.1f);
        const float Span = FMath::Max(0.0f, MaximumRangeMeters - MinimumRangeMeters);
        while (Span / Interval > 8.0f)
        {
            Interval *= 2.0f;
        }
        return Interval;
    }

    TArray<float> GetMajorDistances(float MinimumRangeMeters, float MaximumRangeMeters, float RequestedIntervalMeters)
    {
        TArray<float> Distances;
        if (MaximumRangeMeters <= MinimumRangeMeters)
        {
            return Distances;
        }

        const float Interval = GetAdaptiveDistanceInterval(RequestedIntervalMeters, MinimumRangeMeters, MaximumRangeMeters);
        float Distance = FMath::CeilToFloat(MinimumRangeMeters / Interval) * Interval;
        if (Distance <= MinimumRangeMeters + KINDA_SMALL_NUMBER)
        {
            Distance += Interval;
        }
        for (; Distance < MaximumRangeMeters - KINDA_SMALL_NUMBER && Distances.Num() < 8; Distance += Interval)
        {
            Distances.Add(Distance);
        }
        return Distances;
    }

    FDynamicMeshVertex MakeVertex(const FVector& Position, const FVector& Normal, const FColor& Color)
    {
        FVector SafeNormal = Normal.GetSafeNormal();
        if (SafeNormal.IsNearlyZero())
        {
            SafeNormal = FVector::UpVector;
        }
        FVector TangentX = FVector::CrossProduct(FVector::UpVector, SafeNormal).GetSafeNormal();
        if (TangentX.IsNearlyZero())
        {
            TangentX = FVector::ForwardVector;
        }
        const FVector TangentY = FVector::CrossProduct(SafeNormal, TangentX).GetSafeNormal();

        FDynamicMeshVertex Vertex;
        Vertex.Position = FVector3f(Position);
        Vertex.Color = Color;
        Vertex.TextureCoordinate[0] = FVector2f::ZeroVector;
        Vertex.SetTangents(FVector3f(TangentX), FVector3f(TangentY), FVector3f(SafeNormal));
        return Vertex;
    }

    void AddDoubleSidedTriangle(FDynamicMeshBuilder& MeshBuilder, int32 A, int32 B, int32 C)
    {
        MeshBuilder.AddTriangle(A, B, C);
        MeshBuilder.AddTriangle(C, B, A);
    }

    void AddDoubleSidedQuad(FDynamicMeshBuilder& MeshBuilder, int32 A, int32 B, int32 C, int32 D)
    {
        AddDoubleSidedTriangle(MeshBuilder, A, B, C);
        AddDoubleSidedTriangle(MeshBuilder, A, C, D);
    }

    void DrawDistanceLabel(const FVector& WorldPosition, float DistanceMeters, const FSceneView* View, FCanvas* Canvas, const FLinearColor& Color)
    {
        if (!View || !Canvas || !GEngine)
        {
            return;
        }
        FVector2D Pixel;
        if (!View->WorldToPixel(WorldPosition, Pixel))
        {
            return;
        }
        const FIntRect Rect = Canvas->GetViewRect();
        if (!Rect.Contains(FIntPoint(FMath::RoundToInt(Pixel.X), FMath::RoundToInt(Pixel.Y))))
        {
            return;
        }
        const FString Label = FString::Printf(TEXT("%g m"), DistanceMeters);
        Canvas->DrawShadowedString(Pixel.X + 4.0f, Pixel.Y - 8.0f, *Label, GEngine->GetSmallFont(), Color);
    }
}
