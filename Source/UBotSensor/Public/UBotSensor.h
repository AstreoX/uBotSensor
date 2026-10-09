#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleInterface.h"

UBOTSENSOR_API DECLARE_LOG_CATEGORY_EXTERN(LogUBotSensor, Log, All);

class FUBotSensorModule final : public IModuleInterface
{
public:
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;
};
