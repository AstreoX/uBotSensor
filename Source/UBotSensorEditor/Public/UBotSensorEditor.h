#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleInterface.h"

class FComponentVisualizer;

class FUBotSensorEditorModule final : public IModuleInterface
{
public:
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;

private:
    void RegisterComponentVisualizer(FName ComponentClassName, TSharedPtr<FComponentVisualizer> Visualizer);

    TArray<FName> RegisteredComponentVisualizers;
};
