#pragma once

#include "CoreMinimal.h"
#include "ComponentVisualizer.h"

class FUBotCameraSensorComponentVisualizer final : public FComponentVisualizer
{
public:
    virtual void DrawVisualization(const UActorComponent* Component, const FSceneView* View, FPrimitiveDrawInterface* PDI) override;
};
