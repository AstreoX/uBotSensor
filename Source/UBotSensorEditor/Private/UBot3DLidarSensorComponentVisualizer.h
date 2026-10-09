#pragma once

#include "CoreMinimal.h"
#include "ComponentVisualizer.h"

class FUBot3DLidarSensorComponentVisualizer final : public FComponentVisualizer
{
public:
    virtual void DrawVisualization(const UActorComponent* Component, const FSceneView* View, FPrimitiveDrawInterface* PDI) override;
    virtual void DrawVisualizationHUD(const UActorComponent* Component, const FViewport* Viewport, const FSceneView* View, FCanvas* Canvas) override;
};
