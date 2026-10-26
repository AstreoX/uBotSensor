#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Components/BoxComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/CollisionProfile.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "UBot2DLidarFrame.h"
#include "UBot2DLidarSensorComponent.h"
#include "UBot3DLidarSensorComponent.h"
#include "UBotPointCloudFrame.h"
#include "UBotSensorMath.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformMemory.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/ScopeExit.h"
#include "UObject/StrongObjectPtr.h"
#include "UObject/GarbageCollection.h"
#include "PhysicsEngine/ClusterUnionComponent.h"

namespace UBotLidarTests
{
    // Transient game world with a physics scene, so line traces work right after components register.
    // Destroyed when the scope ends.
    class FScopedLidarTestWorld
    {
    public:
        UE_NONCOPYABLE(FScopedLidarTestWorld);

        FScopedLidarTestWorld()
            : World(UWorld::CreateWorld(EWorldType::Game, false))
        {
        }

        ~FScopedLidarTestWorld()
        {
            if (World)
            {
                World->DestroyWorld(false);
            }
        }

        UWorld* Get() const
        {
            return World;
        }

        // Blocking box whose -X face is centered on FaceCenterCentimeters, so that face's normal is -X.
        AActor* SpawnWall(const FVector& FaceCenterCentimeters, const FVector& HalfExtentCentimeters) const
        {
            AActor* Wall = World ? World->SpawnActor<AActor>() : nullptr;
            if (!Wall)
            {
                return nullptr;
            }
            UBoxComponent* Box = NewObject<UBoxComponent>(Wall, TEXT("Box"));
            Box->SetBoxExtent(HalfExtentCentimeters, false);
            Box->SetCollisionProfileName(UCollisionProfile::BlockAll_ProfileName);
            Wall->SetRootComponent(Box);
            Box->SetWorldLocation(FaceCenterCentimeters + FVector(HalfExtentCentimeters.X, 0.0, 0.0));
            Box->RegisterComponent();
            return Wall;
        }

        // LiDAR as the root of its own carrier actor at the world origin, facing +X.
        template <typename TLidarComponent>
        TLidarComponent* SpawnLidar() const
        {
            AActor* Carrier = World ? World->SpawnActor<AActor>() : nullptr;
            if (!Carrier)
            {
                return nullptr;
            }
            TLidarComponent* Lidar = NewObject<TLidarComponent>(Carrier, TEXT("Lidar"));
            Carrier->SetRootComponent(Lidar);
            Lidar->RegisterComponent();
            return Lidar;
        }

    private:
        UWorld* World = nullptr;
    };

    // Expected intensity of a surface return hit at the given incidence angle.
    float ExpectedIntensity(float IncidenceDegrees)
    {
        return 255.0f * FMath::Cos(FMath::DegreesToRadians(IncidenceDegrees));
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorLidarFrameBoundsTest, "UBotSensor.Lidar.FrameBounds", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorLidarFrameBoundsTest::RunTest(const FString& Parameters)
{
    UUBot2DLidarFrame* Frame = NewObject<UUBot2DLidarFrame>();
    float Range = -1.0f;
    bool bHit = true;
    TestFalse(TEXT("Empty frame rejects index"), Frame->GetRangeByIndex(0, Range, bHit));
    TestEqual(TEXT("Rejected index resets range"), Range, 0.0f);
    TestFalse(TEXT("Rejected index resets hit flag"), bHit);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorLidarAnglesTest, "UBotSensor.Lidar3D.Angles", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorLidarAnglesTest::RunTest(const FString& Parameters)
{
    TestEqual(TEXT("Lowest ring points down"), UBot::SensorMath::LidarVerticalAngleDegrees(0, 3, 10.0f, 20.0f), -10.0f);
    TestEqual(TEXT("Highest ring points up"), UBot::SensorMath::LidarVerticalAngleDegrees(2, 3, 10.0f, 20.0f), 20.0f);
    TestEqual(TEXT("Partial FOV starts at boundary"), UBot::SensorMath::LidarHorizontalAngleDegrees(0, 3, 90.0f), -45.0f);
    TestEqual(TEXT("Partial FOV ends at boundary"), UBot::SensorMath::LidarHorizontalAngleDegrees(2, 3, 90.0f), 45.0f);
    const float FullStart = UBot::SensorMath::LidarHorizontalAngleDegrees(0, 4, 360.0f);
    const float FullEnd = UBot::SensorMath::LidarHorizontalAngleDegrees(3, 4, 360.0f);
    TestTrue(TEXT("Full revolution does not duplicate first direction"), !FMath::IsNearlyEqual(FullStart, FullEnd));
    const FVector Forward = UBot::SensorMath::LidarDirectionLocal(0.0f, 0.0f);
    TestTrue(TEXT("Zero angles face local X"), Forward.Equals(FVector::ForwardVector, 0.001));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorPointCloudBoundsTest, "UBotSensor.Lidar3D.FrameBounds", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorPointCloudBoundsTest::RunTest(const FString& Parameters)
{
    UUBotPointCloudFrame* Frame = NewObject<UUBotPointCloudFrame>();
    FVector Point(1.0, 2.0, 3.0);
    int32 Ring = -1;
    float TimeOffset = -1.0f;
    float Intensity = -1.0f;
    bool bHasIntensity = true;
    TestFalse(TEXT("Empty point cloud rejects index"), Frame->GetPointByIndex(0, Point, Ring, TimeOffset, Intensity, bHasIntensity));
    TestTrue(TEXT("Rejected point is reset"), Point.IsZero());
    TestEqual(TEXT("Rejected ring is reset"), Ring, 0);
    TestEqual(TEXT("Rejected time is reset"), TimeOffset, 0.0f);
    TestFalse(TEXT("Rejected point has no intensity"), bHasIntensity);
    TestFalse(TEXT("Empty point cloud reports no intensity data"), Frame->HasIntensityData());
    TestEqual(TEXT("Empty point cloud has zero hit points"), Frame->HitPointCount, 0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorLidar3DIntensityRangeTest, "UBotSensor.Lidar3D.IntensityRange", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorLidar3DIntensityRangeTest::RunTest(const FString& Parameters)
{
    using namespace UBotLidarTests;

    FScopedLidarTestWorld TestWorld;
    if (!TestNotNull(TEXT("Test world"), TestWorld.Get()))
    {
        return false;
    }

    // Wall face at X = 4.5 m, 6 m wide and 6 m tall. With 8 samples over 360 degrees only the 0 degree
    // column reaches it (the +-45 degree columns pass beside it), so the rings at -10, 0 and +10 degrees
    // produce exactly three surface returns.
    TestNotNull(TEXT("Wall"), TestWorld.SpawnWall(FVector(450.0, 0.0, 0.0), FVector(50.0, 300.0, 300.0)));
    UUBot3DLidarSensorComponent* Lidar = TestWorld.SpawnLidar<UUBot3DLidarSensorComponent>();
    if (!TestNotNull(TEXT("3D LiDAR"), Lidar))
    {
        return false;
    }
    Lidar->NumberOfChannels = 3;
    Lidar->VerticalFovDownDegrees = 10.0f;
    Lidar->VerticalFovUpDegrees = 10.0f;
    Lidar->HorizontalFovDegrees = 360.0f;
    Lidar->HorizontalSamples = 8;
    Lidar->MaximumRangeMeters = 20.0f;

    TestTrue(TEXT("Scan succeeds"), Lidar->Sample3DLidar());
    const UUBotPointCloudFrame* Frame = Lidar->GetLatest3DLidarFrame();
    if (!TestNotNull(TEXT("Point cloud frame"), Frame))
    {
        return false;
    }

    const TArray<float>& Intensities = Frame->GetIntensities();
    TestEqual(TEXT("Wall produces one return per ring"), Frame->HitPointCount, 3);
    TestEqual(TEXT("One intensity per point"), Intensities.Num(), Frame->GetPointCount());
    TestTrue(TEXT("Frame reports intensity data"), Frame->HasIntensityData());

    float MinimumIntensity = TNumericLimits<float>::Max();
    float MaximumIntensity = TNumericLimits<float>::Lowest();
    for (int32 Index = 0; Index < Intensities.Num(); ++Index)
    {
        const float Intensity = Intensities[Index];
        TestTrue(FString::Printf(TEXT("Intensity %d (%f) lies in [0, 255]"), Index, Intensity), Intensity >= 0.0f && Intensity <= 255.0f);
        MinimumIntensity = FMath::Min(MinimumIntensity, Intensity);
        MaximumIntensity = FMath::Max(MaximumIntensity, Intensity);

        FVector Point;
        int32 Ring = -1;
        float TimeOffset = -1.0f;
        float PointIntensity = -1.0f;
        bool bHasIntensity = false;
        TestTrue(FString::Printf(TEXT("Point %d is readable"), Index), Frame->GetPointByIndex(Index, Point, Ring, TimeOffset, PointIntensity, bHasIntensity));
        TestTrue(FString::Printf(TEXT("Point %d has an intensity"), Index), bHasIntensity);
        TestEqual(FString::Printf(TEXT("Point %d intensity matches the array"), Index), PointIntensity, Intensity);
    }
    if (!Intensities.IsEmpty())
    {
        TestTrue(TEXT("Head-on return is the brightest (255)"), FMath::IsNearlyEqual(MaximumIntensity, ExpectedIntensity(0.0f), 0.5f));
        TestTrue(TEXT("Tilted rings follow the incidence cosine"), FMath::IsNearlyEqual(MinimumIntensity, ExpectedIntensity(10.0f), 0.5f));
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorLidar2DIntensityRangeTest, "UBotSensor.Lidar2D.IntensityRange", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorLidar2DIntensityRangeTest::RunTest(const FString& Parameters)
{
    using namespace UBotLidarTests;

    FScopedLidarTestWorld TestWorld;
    if (!TestNotNull(TEXT("Test world"), TestWorld.Get()))
    {
        return false;
    }

    // Wall face at X = 4.5 m, 20 m wide: the -45, 0 and +45 degree samples hit it, the +-90 degree samples
    // run parallel to it and miss.
    TestNotNull(TEXT("Wall"), TestWorld.SpawnWall(FVector(450.0, 0.0, 0.0), FVector(50.0, 1000.0, 300.0)));
    UUBot2DLidarSensorComponent* Lidar = TestWorld.SpawnLidar<UUBot2DLidarSensorComponent>();
    if (!TestNotNull(TEXT("2D LiDAR"), Lidar))
    {
        return false;
    }
    Lidar->StartAngleDegrees = -90.0f;
    Lidar->EndAngleDegrees = 90.0f;
    Lidar->HorizontalSamples = 5;

    TestTrue(TEXT("Scan succeeds"), Lidar->Sample2DLidar());
    const UUBot2DLidarFrame* Frame = Lidar->GetLatest2DLidarFrame();
    if (!TestNotNull(TEXT("2D LiDAR frame"), Frame))
    {
        return false;
    }

    const TArray<float>& Intensities = Frame->GetIntensities();
    if (!TestEqual(TEXT("One intensity per sample"), Intensities.Num(), Frame->GetSampleCount())
        || !TestEqual(TEXT("Sample count"), Frame->GetSampleCount(), 5))
    {
        return false;
    }

    for (int32 Index = 0; Index < Intensities.Num(); ++Index)
    {
        const float Intensity = Intensities[Index];
        TestTrue(FString::Printf(TEXT("Intensity %d (%f) lies in [0, 255]"), Index, Intensity), Intensity >= 0.0f && Intensity <= 255.0f);
        float RangeMeters = 0.0f;
        bool bHit = false;
        Frame->GetRangeByIndex(Index, RangeMeters, bHit);
        if (!bHit)
        {
            TestEqual(FString::Printf(TEXT("No-hit sample %d has zero intensity"), Index), Intensity, 0.0f);
        }
    }

    float RangeMeters = 0.0f;
    bool bHit = false;
    TestTrue(TEXT("Center sample hits the wall"), Frame->GetRangeByIndex(2, RangeMeters, bHit) && bHit);
    TestTrue(TEXT("Center sample range is the wall distance"), FMath::IsNearlyEqual(RangeMeters, 4.5f, 0.01f));
    TestTrue(TEXT("Head-on sample is the brightest (255)"), FMath::IsNearlyEqual(Intensities[2], ExpectedIntensity(0.0f), 0.5f));
    TestTrue(TEXT("-45 degree sample follows the incidence cosine"), FMath::IsNearlyEqual(Intensities[1], ExpectedIntensity(45.0f), 0.5f));
    TestTrue(TEXT("+45 degree sample follows the incidence cosine"), FMath::IsNearlyEqual(Intensities[3], ExpectedIntensity(45.0f), 0.5f));
    TestTrue(TEXT("-90 degree sample misses"), Frame->GetRangeByIndex(0, RangeMeters, bHit) && !bHit);
    TestTrue(TEXT("+90 degree sample misses"), Frame->GetRangeByIndex(4, RangeMeters, bHit) && !bHit);
    TestEqual(TEXT("-90 degree miss has zero intensity"), Intensities[0], 0.0f);
    TestEqual(TEXT("+90 degree miss has zero intensity"), Intensities[4], 0.0f);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotLidarParallelEquivalenceTest, "UBotSensor.Lidar3D.ParallelEquivalence", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FUBotLidarParallelEquivalenceTest::RunTest(const FString& Parameters)
{
    using namespace UBotLidarTests;
    FScopedLidarTestWorld Scope;
    AActor* Wall = Scope.SpawnWall(FVector(450, 0, 0), FVector(50, 1000, 1000));
    UUBot3DLidarSensorComponent* Lidar = Scope.SpawnLidar<UUBot3DLidarSensorComponent>();
    Lidar->NumberOfChannels = 32; Lidar->HorizontalSamples = 720;
    Lidar->HorizontalFovDegrees = 180; Lidar->MaximumRangeMeters = 30;
    for (bool bNoise : {false, true}) for (bool bComplex : {false, true})
    {
        Lidar->bNoiseEnabled = bNoise; Lidar->RangeNoiseStdDevMeters = 0.01f;
        Lidar->bTraceComplex = bComplex;
        Lidar->QueryMode = EUBotLidarQueryMode::Serial;
        TestTrue(TEXT("Serial sample"), Lidar->StartSensor());
        TStrongObjectPtr<UUBotPointCloudFrame> Serial(Lidar->GetLatest3DLidarFrame());
        const TArray<FVector> SavedPoints = Serial->GetPointsMeters();
        Lidar->StopSensor();
        Lidar->QueryMode = EUBotLidarQueryMode::Parallel;
        TestTrue(TEXT("Parallel sample"), Lidar->StartSensor());
        const UUBotPointCloudFrame* Parallel = Lidar->GetLatest3DLidarFrame();
        TestTrue(TEXT("Worker path used"), Lidar->GetLastCaptureTimings().bUsedParallel);
        TestTrue(TEXT("Point order and seeded noise identical"), Serial->GetPointsMeters() == Parallel->GetPointsMeters());
        TestTrue(TEXT("Rings identical"), Serial->GetRingIndices() == Parallel->GetRingIndices());
        TestTrue(TEXT("Intensities identical"), Serial->GetIntensities() == Parallel->GetIntensities());
        TestTrue(TEXT("Snapshot time offsets identical"), Serial->GetTimeOffsetsSeconds() == Parallel->GetTimeOffsetsSeconds());
        TestEqual(TEXT("Timestamp unchanged"), Serial->Header.TimestampSeconds, Parallel->Header.TimestampSeconds);
        Wall->AddActorWorldOffset(FVector(10, 0, 0));
        Lidar->Sample3DLidar();
        TestTrue(TEXT("Published old frame is immutable"), Serial->GetPointsMeters() == SavedPoints);
        Lidar->StopSensor();
    }
    Lidar->IgnoredActors.Add(Wall);
    Lidar->Sample3DLidar();
    TestEqual(TEXT("Ignored actors respected in workers"), Lidar->GetLatest3DLidarFrame()->GetPointCount(), 0);
    Lidar->IgnoredActors.Reset();
    Lidar->HorizontalSamples = 9; Lidar->NumberOfChannels = 3; Lidar->HorizontalFovDegrees = 90;
    Lidar->Sample3DLidar();
    TestEqual(TEXT("Configuration invalidates cached directions"), Lidar->GetLatest3DLidarFrame()->TotalRayCount, 27);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotLidarComplexMeshEquivalenceTest, "UBotSensor.Lidar3D.ComplexMeshEquivalence", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FUBotLidarComplexMeshEquivalenceTest::RunTest(const FString& Parameters)
{
    using namespace UBotLidarTests;
    FScopedLidarTestWorld Scope;
    AActor* Actor = Scope.Get()->SpawnActor<AActor>();
    auto* Mesh = NewObject<UStaticMeshComponent>(Actor);
    auto* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
    if (!TestNotNull(TEXT("Engine mesh with triangle collision"), Cube)) { return false; }
    Mesh->SetStaticMesh(Cube);
    Mesh->SetCollisionProfileName(UCollisionProfile::BlockAll_ProfileName);
    Actor->SetRootComponent(Mesh);
    Mesh->SetWorldLocation(FVector(500,0,0));
    Mesh->SetWorldScale3D(FVector(1,8,8));
    Mesh->RegisterComponent();
    auto* Lidar = Scope.SpawnLidar<UUBot3DLidarSensorComponent>();
    Lidar->bTraceComplex = true;
    Lidar->NumberOfChannels = 16;
    Lidar->HorizontalSamples = 128;
    Lidar->HorizontalFovDegrees = 120;
    Lidar->MaximumRangeMeters = 30;
    for (int32 Move = 0; Move < 3; ++Move)
    {
        Actor->AddActorWorldOffset(FVector(15,10,0));
        Lidar->QueryMode = EUBotLidarQueryMode::Serial;
        TestTrue(TEXT("Serial triangle scan"), Lidar->Sample3DLidar());
        TStrongObjectPtr<UUBotPointCloudFrame> Serial(Lidar->GetLatest3DLidarFrame());
        TestTrue(TEXT("Triangle mesh produces actual returns"), Serial->HitPointCount > 0);
        Lidar->QueryMode = EUBotLidarQueryMode::Parallel;
        TestTrue(TEXT("Parallel triangle scan"), Lidar->Sample3DLidar());
        const auto* Parallel = Lidar->GetLatest3DLidarFrame();
        TestTrue(TEXT("Triangle query executes in workers"), Lidar->GetLastCaptureTimings().bUsedParallel);
        TestTrue(TEXT("Moving mesh preserves exact point order and values"), Serial->GetPointsMeters() == Parallel->GetPointsMeters());
        TestTrue(TEXT("Triangle surface normals preserve intensity"), Serial->GetIntensities() == Parallel->GetIntensities());
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotLidarSpecialQueryFallbackTest, "UBotSensor.Lidar3D.SpecialQueryFallback", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FUBotLidarSpecialQueryFallbackTest::RunTest(const FString& Parameters)
{
    using namespace UBotLidarTests;
    FScopedLidarTestWorld Scope;
    Scope.SpawnWall(FVector(450, 0, 0), FVector(50, 1000, 1000));
    auto* Lidar = Scope.SpawnLidar<UUBot3DLidarSensorComponent>();
#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
    Scope.Get()->bDebugDrawAllTraceTags = true;
    Lidar->QueryMode = EUBotLidarQueryMode::Parallel;
    TestTrue(TEXT("Engine query debug drawing still samples"), Lidar->Sample3DLidar());
    TestFalse(TEXT("Engine query drawing retains its GT behavior"), Lidar->GetLastCaptureTimings().bUsedParallel);
    Scope.Get()->bDebugDrawAllTraceTags = false;
#endif
    Lidar->NumberOfChannels = 1;
    Lidar->QueryMode = EUBotLidarQueryMode::Parallel;
    TestTrue(TEXT("Single-channel request still samples"), Lidar->Sample3DLidar());
    TestFalse(TEXT("One executable block is reported as serial"), Lidar->GetLastCaptureTimings().bUsedParallel);
    Lidar->NumberOfChannels = 16;
    AActor* UnionActor = Scope.Get()->SpawnActor<AActor>();
    auto* Union = NewObject<UClusterUnionComponent>(UnionActor);
    UnionActor->SetRootComponent(Union);
    Union->RegisterComponent();
    Lidar->QueryMode = EUBotLidarQueryMode::Serial;
    Lidar->Sample3DLidar();
    const TArray<FVector> Expected = Lidar->GetLatest3DLidarFrame()->GetPointsMeters();
    Lidar->QueryMode = EUBotLidarQueryMode::Parallel;
    TestTrue(TEXT("Special-query world still samples"), Lidar->Sample3DLidar());
    TestFalse(TEXT("Unvalidated component extension path retains serial queries"), Lidar->GetLastCaptureTimings().bUsedParallel);
    TestTrue(TEXT("Fallback preserves results"), Expected == Lidar->GetLatest3DLidarFrame()->GetPointsMeters());
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotLidarPerformanceTest, "UBotSensor.Performance.LidarPipeline", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FUBotLidarPerformanceTest::RunTest(const FString& Parameters)
{
    const FString Output = FPlatformMisc::GetEnvironmentVariable(TEXT("UBOT_SENSOR_BENCH_OUTPUT"));
    if (Output.IsEmpty()) { AddInfo(TEXT("Opt-in wall-clock benchmark: set UBOT_SENSOR_BENCH_OUTPUT.")); return true; }
    using namespace UBotLidarTests;
    FScopedLidarTestWorld Scope;
    Scope.Get()->AddToRoot();
    ON_SCOPE_EXIT { Scope.Get()->RemoveFromRoot(); };
    // Enclosing room, columns and elevated shelves: the same geometry is retained across every A/B.
    for (const FVector& Center : {FVector(1200,0,0), FVector(-1200,0,0), FVector(0,900,0), FVector(0,-900,0), FVector(0,0,-150), FVector(0,0,350)})
    {
        const FVector Extent = FMath::Abs(Center.X) > 1000 ? FVector(10,900,250) :
            FMath::Abs(Center.Y) > 800 ? FVector(1200,10,250) : FVector(1200,900,10);
        AActor* Box = Scope.SpawnWall(Center, Extent); Box->SetActorLocation(Center);
    }
    for (int32 X = -3; X <= 3; ++X) for (int32 Y = -2; Y <= 2; ++Y)
    {
        if (!X && !Y) { continue; }
        Scope.SpawnWall(FVector(X*270,Y*270,0), FVector(18,18,220));
        Scope.SpawnWall(FVector(X*270,Y*270,100), FVector(100,35,10));
    }
    const int32 LidarCount = FMath::Clamp(FCString::Atoi(*FPlatformMisc::GetEnvironmentVariable(TEXT("UBOT_SENSOR_BENCH_LIDARS"))), 1, 4);
    const FString WarmEnv = FPlatformMisc::GetEnvironmentVariable(TEXT("UBOT_SENSOR_BENCH_WARMUP"));
    const FString DurationEnv = FPlatformMisc::GetEnvironmentVariable(TEXT("UBOT_SENSOR_BENCH_SECONDS"));
    const double Warmup = WarmEnv.IsEmpty() ? 10.0 : FCString::Atod(*WarmEnv);
    const double Duration = DurationEnv.IsEmpty() ? 60.0 : FCString::Atod(*DurationEnv);
    const FString SamplesEnv = FPlatformMisc::GetEnvironmentVariable(TEXT("UBOT_SENSOR_BENCH_COLUMNS"));
    const int32 Columns = SamplesEnv.IsEmpty() ? 720 : FMath::Clamp(FCString::Atoi(*SamplesEnv), 4, 2048);
    TArray<UUBot3DLidarSensorComponent*> Lidars;
    for (int32 Index = 0; Index < LidarCount; ++Index)
    {
        auto* Lidar = Scope.SpawnLidar<UUBot3DLidarSensorComponent>();
        Lidar->NumberOfChannels = 32; Lidar->HorizontalSamples = Columns;
        Lidar->VerticalFovUpDegrees = Lidar->VerticalFovDownDegrees = 30;
        Lidar->MaximumRangeMeters = 30; Lidar->bNoiseEnabled = false;
        Lidars.Add(Lidar);
    }
    IConsoleVariable* Cache = IConsoleManager::Get().FindConsoleVariable(TEXT("ubot.Sensor.Lidar.DirectionCache"));
    const int32 SavedCache = Cache->GetInt();
    ON_SCOPE_EXIT { Cache->Set(SavedCache, ECVF_SetByCode); };
    IConsoleVariable* Tasks = IConsoleManager::Get().FindConsoleVariable(TEXT("ubot.Sensor.Lidar.MaxQueryTasks"));
    const int32 SavedTasks = Tasks->GetInt();
    ON_SCOPE_EXIT { Tasks->Set(SavedTasks, ECVF_SetByCode); };
    const FString TasksEnv = FPlatformMisc::GetEnvironmentVariable(TEXT("UBOT_SENSOR_BENCH_TASKS"));
    const int32 RequestedTasks = TasksEnv.IsEmpty() ? SavedTasks : FMath::Clamp(FCString::Atoi(*TasksEnv), 1, 32);
    const bool bSweep = FPlatformMisc::GetEnvironmentVariable(TEXT("UBOT_SENSOR_BENCH_SWEEP")) == TEXT("1");
    const bool bPairedOnly = FPlatformMisc::GetEnvironmentVariable(TEXT("UBOT_SENSOR_BENCH_PAIRED_ONLY")) == TEXT("1");
    const int32 ModeCount = bPairedOnly ? 2 : (bSweep ? 6 : 3);
    FString CSV = TEXT("round,mode,lidars,rays,wall_seconds,capture_ms,preparation_ms,query_ms,assembly_ms,dispatch_ms,cpu_percent,working_set_bytes,parallel,query_tasks\n");
    for (int32 Round = 0; Round < 3; ++Round)
    {
        for (int32 Order = 0; Order < ModeCount; ++Order)
        {
            const int32 Mode = (Round % 2 ? ModeCount - 1 - Order : Order) + (bPairedOnly ? 1 : 0);
            const int32 TaskCount = bSweep && Mode >= 2 ? (4 << (Mode - 2)) : RequestedTasks;
            Tasks->Set(TaskCount, ECVF_SetByCode);
            Cache->Set(Mode == 0 ? 0 : 1, ECVF_SetByCode);
            for (auto* Lidar : Lidars) { Lidar->QueryMode = Mode >= 2 ? EUBotLidarQueryMode::Parallel : EUBotLidarQueryMode::Serial; }
            const double Start = FPlatformTime::Seconds();
            double Next = Start;
            int32 Frame = 0;
            while (FPlatformTime::Seconds() - Start < Warmup + Duration)
            {
                const double Now = FPlatformTime::Seconds();
                if (Now < Next) { FPlatformProcess::Sleep(static_cast<float>(Next - Now)); }
                Next += 0.1;
                for (auto* Lidar : Lidars)
                {
                    Lidar->Sample3DLidar();
                    const double Elapsed = FPlatformTime::Seconds() - Start;
                    if (Elapsed < Warmup) { continue; }
#if PLATFORM_WINDOWS
                    FPlatformTime::UpdateCPUTime(0);
#endif
                    const auto& T = Lidar->GetLastCaptureTimings();
                    CSV += FString::Printf(TEXT("%d,%d,%d,%d,%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f,%llu,%d,%d\n"),
                        Round, Mode, LidarCount, 32*Columns, Elapsed, Lidar->GetLatest3DLidarFrame()->CaptureDurationMilliseconds,
                        T.PreparationMilliseconds, T.QueryMilliseconds, T.AssemblyMilliseconds, T.DispatchMilliseconds,
                        FPlatformTime::GetCPUTime().CPUTimePct, FPlatformMemory::GetStats().UsedPhysical, T.bUsedParallel ? 1 : 0, TaskCount);
                }
                // Frames are immutable UObject instances. GC is outside the measured capture scopes.
                if (++Frame % 20 == 0) { CollectGarbage(RF_NoFlags); }
            }
            FFileHelper::SaveStringToFile(CSV, *Output);
            AddInfo(FString::Printf(TEXT("Benchmark round %d mode %d complete (%d lidars, %d rays)."), Round, Mode, LidarCount, 32*Columns));
        }
    }
    return true;
}

#endif
