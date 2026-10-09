#pragma once

#include "CoreMinimal.h"

class FCanvas;
class FDynamicMeshBuilder;
class FPrimitiveDrawInterface;
class FSceneView;
class UMaterialInterface;
struct FDynamicMeshVertex;

namespace UBot::LidarVisualizer
{
    UMaterialInterface* GetPreviewMaterial();
    FColor MakeFillColor(const FLinearColor& Color, float Opacity);
    FLinearColor MakeGridColor(const FLinearColor& Color, float Scale = 1.0f);
    float GetAdaptiveDistanceInterval(float RequestedIntervalMeters, float MinimumRangeMeters, float MaximumRangeMeters);
    TArray<float> GetMajorDistances(float MinimumRangeMeters, float MaximumRangeMeters, float RequestedIntervalMeters);
    FDynamicMeshVertex MakeVertex(const FVector& Position, const FVector& Normal, const FColor& Color);
    void AddDoubleSidedTriangle(FDynamicMeshBuilder& MeshBuilder, int32 A, int32 B, int32 C);
    void AddDoubleSidedQuad(FDynamicMeshBuilder& MeshBuilder, int32 A, int32 B, int32 C, int32 D);
    void DrawDistanceLabel(const FVector& WorldPosition, float DistanceMeters, const FSceneView* View, FCanvas* Canvas, const FLinearColor& Color);
}
