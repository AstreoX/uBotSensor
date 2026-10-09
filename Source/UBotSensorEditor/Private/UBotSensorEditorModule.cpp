#include "UBotSensorEditor.h"

#include "ComponentVisualizer.h"
#include "Editor/UnrealEdEngine.h"
#include "Modules/ModuleManager.h"
#include "UBot2DLidarSensorComponent.h"
#include "UBot2DLidarSensorComponentVisualizer.h"
#include "UBot3DLidarSensorComponent.h"
#include "UBot3DLidarSensorComponentVisualizer.h"
#include "UBotCameraSensorComponent.h"
#include "UBotCameraSensorComponentVisualizer.h"
#include "UBotSensor.h"
#include "UnrealEdGlobals.h"

IMPLEMENT_MODULE(FUBotSensorEditorModule, UBotSensorEditor)

void FUBotSensorEditorModule::StartupModule()
{
    if (!GUnrealEd)
    {
        // Commandlets (package validation, Python scripts, ...) load this module without an UnrealEd engine;
        // there is no viewport to draw into, so skipping the visualizers is expected there.
        if (IsRunningCommandlet())
        {
            UE_LOG(LogUBotSensor, Log, TEXT("uBot sensor component visualizers are not registered in a commandlet."));
        }
        else
        {
            UE_LOG(LogUBotSensor, Error, TEXT("uBot sensor component visualizers could not register because GUnrealEd is unavailable."));
        }
        return;
    }

    RegisterComponentVisualizer(UUBot2DLidarSensorComponent::StaticClass()->GetFName(), MakeShared<FUBot2DLidarSensorComponentVisualizer>());
    RegisterComponentVisualizer(UUBot3DLidarSensorComponent::StaticClass()->GetFName(), MakeShared<FUBot3DLidarSensorComponentVisualizer>());
    // Registered on the camera base class; the editor looks visualizers up through the class hierarchy,
    // so it covers the RGB and depth cameras.
    RegisterComponentVisualizer(UUBotCameraSensorComponent::StaticClass()->GetFName(), MakeShared<FUBotCameraSensorComponentVisualizer>());
    UE_LOG(LogUBotSensor, Display, TEXT("Registered UBotSensor component visualizers."));
}

void FUBotSensorEditorModule::ShutdownModule()
{
    if (!GUnrealEd)
    {
        return;
    }
    for (FName ClassName : RegisteredComponentVisualizers)
    {
        GUnrealEd->UnregisterComponentVisualizer(ClassName);
    }
    RegisteredComponentVisualizers.Reset();
}

void FUBotSensorEditorModule::RegisterComponentVisualizer(FName ComponentClassName, TSharedPtr<FComponentVisualizer> Visualizer)
{
    GUnrealEd->RegisterComponentVisualizer(ComponentClassName, Visualizer);
    RegisteredComponentVisualizers.Add(ComponentClassName);
    if (Visualizer.IsValid())
    {
        Visualizer->OnRegister();
    }
}
