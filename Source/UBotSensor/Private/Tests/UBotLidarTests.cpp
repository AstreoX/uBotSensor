#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Components/BoxComponent.h"
#include "Engine/CollisionProfile.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "UBot2DLidarFrame.h"
#include "UBot2DLidarSensorComponent.h"
#include "UBot3DLidarSensorComponent.h"
#include "UBotPointCloudFrame.h"
#include "UBotSensorMath.h"

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

#endif
