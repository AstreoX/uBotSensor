#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/ConfigCacheIni.h"
#include "Misc/PackageName.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "UBotImuSensorComponent.h"
#include "UBotSensorTypes.h"
#include "UBotTypes.h"
#include "UObject/Class.h"
#include "UObject/CoreRedirects.h"
#include "UObject/UObjectGlobals.h"

namespace UBotSensorRedirectTests
{
    const TCHAR* const LegacyScriptPackage = TEXT("/Script/AgentSensorCore");

    // Every reflected type declared in the legacy AgentSensorCore Public headers (AgentSensorCore 0.5.0).
    const TCHAR* const LegacyClasses[] =
    {
        TEXT("AgentSensorComponent"),
        TEXT("AgentPoseSensorComponent"),
        TEXT("AgentImuSensorComponent"),
        TEXT("AgentOdometrySensorComponent"),
        TEXT("Agent2DLidarSensorComponent"),
        TEXT("Agent3DLidarSensorComponent"),
        TEXT("AgentCameraSensorComponent"),
        TEXT("AgentRgbCameraSensorComponent"),
        TEXT("AgentDepthCameraSensorComponent"),
        TEXT("AgentCameraCaptureCoordinatorComponent"),
        TEXT("AgentCameraCaptureSubsystem"),
        TEXT("Agent2DLidarFrame"),
        TEXT("AgentPointCloudFrame"),
        TEXT("AgentImageFrame"),
    };

    const TCHAR* const LegacyStructs[] =
    {
        TEXT("AgentSensorFrameHeader"),
        TEXT("AgentPoseSensorData"),
        TEXT("AgentImuSensorData"),
        TEXT("AgentOdometrySensorData"),
    };

    const TCHAR* const LegacyEnums[] =
    {
        TEXT("EAgentSensorSampleStatus"),
        TEXT("EAgentPoseReferenceFrame"),
        TEXT("EAgentImageEncoding"),
        TEXT("EAgentDepthPreviewMode"),
    };

    // File-scope DECLARE_DYNAMIC_MULTICAST_DELEGATE* types; UHT names their signature <Name>__DelegateSignature.
    const TCHAR* const LegacyDelegates[] =
    {
        TEXT("AgentSensorSimpleEvent"),
        TEXT("AgentSensorSampledEvent"),
        TEXT("AgentSensorErrorEvent"),
        TEXT("AgentPoseDataEvent"),
        TEXT("AgentImuDataEvent"),
        TEXT("AgentOdometryDataEvent"),
        TEXT("Agent2DLidarFrameEvent"),
        TEXT("Agent3DLidarFrameEvent"),
        TEXT("AgentImageFrameEvent"),
        TEXT("AgentCameraCaptureGroupErrorEvent"),
    };

    const TCHAR* const MeshKinds[] =
    {
        TEXT("DepthCamera"),
        TEXT("IMU"),
        TEXT("Lidar2D"),
        TEXT("Lidar3D"),
        TEXT("Odometry"),
        TEXT("Pose"),
        TEXT("RGBCamera"),
    };

    // Same key -> type mapping as FCoreRedirects::Initialize uses for ini sections.
    ECoreRedirectFlags TypeForConfigKey(FName Key)
    {
        struct FKeyType
        {
            const TCHAR* Key;
            ECoreRedirectFlags Type;
        };
        static const FKeyType KeyTypes[] =
        {
            { TEXT("ObjectRedirects"), ECoreRedirectFlags::Type_Object },
            { TEXT("ClassRedirects"), ECoreRedirectFlags::Type_Class },
            { TEXT("StructRedirects"), ECoreRedirectFlags::Type_Struct },
            { TEXT("EnumRedirects"), ECoreRedirectFlags::Type_Enum },
            { TEXT("FunctionRedirects"), ECoreRedirectFlags::Type_Function },
            { TEXT("PropertyRedirects"), ECoreRedirectFlags::Type_Property },
            { TEXT("PackageRedirects"), ECoreRedirectFlags::Type_Package },
        };
        for (const FKeyType& KeyType : KeyTypes)
        {
            if (Key == FName(KeyType.Key))
            {
                return KeyType.Type;
            }
        }
        return ECoreRedirectFlags::None;
    }

    FString Redirect(ECoreRedirectFlags Type, const FString& OldName)
    {
        return FCoreRedirects::GetRedirectedName(Type, FCoreRedirectObjectName(OldName)).ToString();
    }

    // Compares as strings so a failure prints both names.
    bool TestRedirect(FAutomationTestBase& Test, const FString& What, ECoreRedirectFlags Type, const FString& OldName, const FString& ExpectedNewName)
    {
        const FString ActualNewName = Redirect(Type, OldName);
        return Test.TestEqual(*What, *ActualNewName, *ExpectedNewName);
    }

    struct FConfigRedirect
    {
        FName Key;
        FString OldName;
        FString NewName;
    };

    // Reads [CoreRedirects] from the plugin's Config/DefaultUBotSensor.ini, the single list of these redirects.
    // The file is parsed on its own instead of asking GConfig: FPluginManager loads it into the "UBotSensor"
    // config branch and InitUObject reads the redirects from there at startup, but a plugin branch does not
    // keep the section readable through GConfig afterwards (UE 5.5.4: only SectionsToSave is left). That the
    // list really was picked up is proven by the lookups below, which only the ini can have registered.
    bool ReadPluginConfigRedirects(TArray<FConfigRedirect>& OutRedirects, FString& OutFilename)
    {
        OutRedirects.Reset();

        // <plugin>/Content/ is mounted as /UBotSensor/, the config file sits next to it.
        FString ContentDir;
        if (!FPackageName::TryConvertLongPackageNameToFilename(FString(TEXT("/UBotSensor/")), ContentDir))
        {
            return false;
        }
        const FString PluginDir = FPaths::GetPath(FPaths::GetPath(ContentDir));
        OutFilename = FPaths::ConvertRelativePathToFull(PluginDir / TEXT("Config") / TEXT("DefaultUBotSensor.ini"));
        if (!FPaths::FileExists(OutFilename))
        {
            return false;
        }

        FConfigFile File;
        File.Read(OutFilename);
        const FConfigSection* Section = File.FindSection(TEXT("CoreRedirects"));
        if (!Section)
        {
            return false;
        }
        for (FConfigSection::TConstIterator It(*Section); It; ++It)
        {
            FConfigRedirect& Entry = OutRedirects.AddDefaulted_GetRef();
            // A raw file read keeps the array-add operator that the config hierarchy strips: "+ClassRedirects".
            FString KeyString = It.Key().ToString();
            KeyString.RemoveFromStart(TEXT("+"), ESearchCase::CaseSensitive);
            Entry.Key = FName(*KeyString);
            const FString& Value = It.Value().GetValue();
            FParse::Value(*Value, TEXT("OldName="), Entry.OldName);
            FParse::Value(*Value, TEXT("NewName="), Entry.NewName);
        }
        return true;
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorRedirectsResolveTest, "UBotSensor.Redirects.Resolve", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorRedirectsResolveTest::RunTest(const FString& Parameters)
{
    using namespace UBotSensorRedirectTests;

    // 1. The list lives in Config/DefaultUBotSensor.ini, and every line of it is active in FCoreRedirects.
    TArray<FConfigRedirect> ConfigRedirects;
    FString ConfigFilename;
    if (!TestTrue(TEXT("Config/DefaultUBotSensor.ini has a [CoreRedirects] section"), ReadPluginConfigRedirects(ConfigRedirects, ConfigFilename)))
    {
        return false;
    }
    AddInfo(FString::Printf(TEXT("%d CoreRedirects read from %s"), ConfigRedirects.Num(), *ConfigFilename));
    TestTrue(TEXT("The plugin config lists redirects"), ConfigRedirects.Num() > 0);
    for (const FConfigRedirect& Entry : ConfigRedirects)
    {
        const ECoreRedirectFlags Type = TypeForConfigKey(Entry.Key);
        if (!TestTrue(*FString::Printf(TEXT("Known redirect key %s"), *Entry.Key.ToString()), Type != ECoreRedirectFlags::None))
        {
            continue;
        }
        TestRedirect(*this, FString::Printf(TEXT("%s %s is active"), *Entry.Key.ToString(), *Entry.OldName),
            Type, Entry.OldName, FCoreRedirectObjectName(Entry.NewName).ToString());
    }

    // 2. One class, struct, enum and package entry, queried the way the linker queries them, land on the real types.
    TestRedirect(*this, TEXT("Class redirect"), ECoreRedirectFlags::Type_Class,
        TEXT("/Script/AgentSensorCore.AgentImuSensorComponent"), UUBotImuSensorComponent::StaticClass()->GetPathName());
    TestRedirect(*this, TEXT("Struct redirect into UBotCore"), ECoreRedirectFlags::Type_Struct,
        TEXT("/Script/AgentSensorCore.AgentSensorFrameHeader"), FUBotFrameHeader::StaticStruct()->GetPathName());
    TestRedirect(*this, TEXT("Struct redirect"), ECoreRedirectFlags::Type_Struct,
        TEXT("/Script/AgentSensorCore.AgentImuSensorData"), FUBotImuSensorData::StaticStruct()->GetPathName());
    TestRedirect(*this, TEXT("Enum redirect into UBotCore"), ECoreRedirectFlags::Type_Enum,
        TEXT("/Script/AgentSensorCore.EAgentSensorSampleStatus"), StaticEnum<EUBotSampleStatus>()->GetPathName());
    TestRedirect(*this, TEXT("Enum redirect"), ECoreRedirectFlags::Type_Enum,
        TEXT("/Script/AgentSensorCore.EAgentPoseReferenceFrame"), StaticEnum<EUBotPoseReferenceFrame>()->GetPathName());
    TestRedirect(*this, TEXT("Mesh package redirect"), ECoreRedirectFlags::Type_Package,
        TEXT("/AgentSensorCore/Models/SM_AgentSensor_IMU"), TEXT("/UBotSensor/Models/SM_UBotSensor_IMU"));
    TestRedirect(*this, TEXT("Script package redirect"), ECoreRedirectFlags::Type_Package,
        LegacyScriptPackage, TEXT("/Script/UBotSensor"));
    // A Default__ import keeps its old outer after the class rename alone; the package redirect moves it.
    TestRedirect(*this, TEXT("Default__ objects of the legacy classes leave the legacy script package"), ECoreRedirectFlags::Type_Object,
        TEXT("/Script/AgentSensorCore.Default__AgentImuSensorComponent"), TEXT("/Script/UBotSensor.Default__AgentImuSensorComponent"));
    // Saved enum values are "EAgentSensorSampleStatus::X"; UEnum re-qualifies them after the type redirect.
    TestEqual(TEXT("Legacy enum value names resolve"),
        StaticEnum<EUBotSampleStatus>()->GetValueByName(FName(TEXT("EAgentSensorSampleStatus::InvalidDeltaTime"))),
        static_cast<int64>(EUBotSampleStatus::InvalidDeltaTime));

    // 3. Exhaustive: every legacy reflected type, delegate signature and mesh resolves to something that exists.
    for (const TCHAR* Name : LegacyClasses)
    {
        const FString NewPath = Redirect(ECoreRedirectFlags::Type_Class, FString::Printf(TEXT("%s.%s"), LegacyScriptPackage, Name));
        TestNotNull(*FString::Printf(TEXT("Class %s -> %s exists"), Name, *NewPath), FindObject<UClass>(nullptr, *NewPath));
    }
    for (const TCHAR* Name : LegacyStructs)
    {
        const FString NewPath = Redirect(ECoreRedirectFlags::Type_Struct, FString::Printf(TEXT("%s.%s"), LegacyScriptPackage, Name));
        TestNotNull(*FString::Printf(TEXT("Struct %s -> %s exists"), Name, *NewPath), FindObject<UScriptStruct>(nullptr, *NewPath));
    }
    for (const TCHAR* Name : LegacyEnums)
    {
        const FString NewPath = Redirect(ECoreRedirectFlags::Type_Enum, FString::Printf(TEXT("%s.%s"), LegacyScriptPackage, Name));
        TestNotNull(*FString::Printf(TEXT("Enum %s -> %s exists"), Name, *NewPath), FindObject<UEnum>(nullptr, *NewPath));
    }
    for (const TCHAR* Name : LegacyDelegates)
    {
        // The linker resolves an import of class DelegateFunction with Type_Object (FCoreRedirects::GetFlagsForTypeName).
        const FString NewPath = Redirect(ECoreRedirectFlags::Type_Object, FString::Printf(TEXT("%s.%s__DelegateSignature"), LegacyScriptPackage, Name));
        TestNotNull(*FString::Printf(TEXT("Delegate signature %s -> %s exists"), Name, *NewPath), FindObject<UFunction>(nullptr, *NewPath));
    }
    for (const TCHAR* Kind : MeshKinds)
    {
        const FString OldPackage = FString::Printf(TEXT("/AgentSensorCore/Models/SM_AgentSensor_%s"), Kind);
        const FString NewPackage = FString::Printf(TEXT("/UBotSensor/Models/SM_UBotSensor_%s"), Kind);
        TestRedirect(*this, FString::Printf(TEXT("Mesh package %s"), Kind), ECoreRedirectFlags::Type_Package, OldPackage, NewPackage);
        TestRedirect(*this, FString::Printf(TEXT("Mesh object %s"), Kind), ECoreRedirectFlags::Type_Object,
            FString::Printf(TEXT("%s.SM_AgentSensor_%s"), *OldPackage, Kind),
            FString::Printf(TEXT("%s.SM_UBotSensor_%s"), *NewPackage, Kind));
        TestTrue(*FString::Printf(TEXT("Mesh package %s exists"), *NewPackage), FPackageName::DoesPackageExist(NewPackage));
    }
    return true;
}

#endif
