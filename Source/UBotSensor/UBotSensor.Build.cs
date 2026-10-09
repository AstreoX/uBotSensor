using UnrealBuildTool;

public class UBotSensor : ModuleRules
{
    public UBotSensor(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(new[]
        {
            "Core",
            "CoreUObject",
            "Engine",
            "UBotCore"
        });
    }
}
