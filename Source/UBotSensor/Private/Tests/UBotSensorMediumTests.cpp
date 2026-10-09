#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Engine/Engine.h"
#include "Engine/World.h"
#include "Math/RandomStream.h"
#include "UBotPerceptionMedium.h"
#include "UBotSensorMedium.h"
#include "UBotSensorMediumTestTypes.h"

namespace UBotSensorMediumTests
{
    FUBotMediumCell MakeCell(const FVector& PositionCentimeters, float RadiusCentimeters, float Density, float EdgeFactor = 1.0f)
    {
        FUBotMediumCell Cell;
        Cell.WorldPositionCentimeters = PositionCentimeters;
        Cell.RadiusCentimeters = RadiusCentimeters;
        Cell.Density = Density;
        Cell.EdgeFactor = EdgeFactor;
        return Cell;
    }

    // Hand-built snapshot (no source), finalized like the collected ones.
    FUBotMediumSnapshot MakeSnapshot(std::initializer_list<FUBotMediumCell> Cells)
    {
        FUBotMediumSnapshot Snapshot;
        Snapshot.Cells = Cells;
        UBot::Medium::FinalizeSnapshot(Snapshot);
        return Snapshot;
    }

    // Transient game world for tests that need spawned actors; destroyed when the scope ends.
    class FScopedTestWorld
    {
    public:
        UE_NONCOPYABLE(FScopedTestWorld);

        FScopedTestWorld()
            : World(UWorld::CreateWorld(EWorldType::Game, false))
        {
            if (World && GEngine)
            {
                // InitializeActorsForPlay looks its world context up in the engine.
                GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
                // AActor::ProcessEvent ignores calls until the world's actors are initialized, which would
                // turn IUBotPerceptionMedium::Execute_GetMediumSnapshot and the legacy reflection calls
                // on the spawned actors into silent no-ops.
                World->InitializeActorsForPlay(FURL());
            }
        }

        ~FScopedTestWorld()
        {
            if (World)
            {
                if (GEngine)
                {
                    GEngine->DestroyWorldContext(World);
                }
                World->DestroyWorld(false);
            }
        }

        UWorld* Get() const
        {
            return World;
        }

        template <typename TActor>
        TActor* Spawn(const TCHAR* Name) const
        {
            if (!World)
            {
                return nullptr;
            }
            FActorSpawnParameters Params;
            Params.Name = FName(Name);
            Params.NameMode = FActorSpawnParameters::ESpawnActorNameMode::Requested;
            return World->SpawnActor<TActor>(Params);
        }

    private:
        UWorld* World = nullptr;
    };

    bool AreCellsEqual(const FUBotMediumCell& A, const FUBotMediumCell& B)
    {
        return A.WorldPositionCentimeters == B.WorldPositionCentimeters
            && A.RadiusCentimeters == B.RadiusCentimeters
            && A.Density == B.Density
            && A.EdgeFactor == B.EdgeFactor;
    }

    // Equivalent = same medium type, activity, cells and bounds. Source and tint are allowed to differ.
    void TestEquivalentSnapshots(FAutomationTestBase& Test, const TCHAR* What, const FUBotMediumSnapshot& A, const FUBotMediumSnapshot& B)
    {
        Test.TestTrue(*FString::Printf(TEXT("%s: same medium type"), What), A.MediumType == B.MediumType);
        Test.TestTrue(*FString::Printf(TEXT("%s: same activity"), What), A.bActive == B.bActive);
        if (!Test.TestEqual(*FString::Printf(TEXT("%s: same cell count"), What), A.Cells.Num(), B.Cells.Num()))
        {
            return;
        }
        for (int32 Index = 0; Index < A.Cells.Num(); ++Index)
        {
            Test.TestTrue(*FString::Printf(TEXT("%s: cell %d is identical"), What, Index), AreCellsEqual(A.Cells[Index], B.Cells[Index]));
        }
        Test.TestTrue(*FString::Printf(TEXT("%s: same bounds"), What), A.BoundsCentimeters == B.BoundsCentimeters);
    }

    // BGRA of one black pixel after looking down +X through the given media.
    TArray<uint8> RenderBlackPixel(const TArray<FUBotMediumSnapshot>& Snapshots, const UBot::SensorMedium::FCameraSettings& Settings)
    {
        TArray<uint8> PixelBytes = { 0, 0, 0, 255 };
        UBot::SensorMedium::ApplyToBGRAPixels(FTransform::Identity, 1, 1, 1.0f, 1.0f, 0.0f, 0.0f, 10.0f, Snapshots, Settings, PixelBytes);
        return PixelBytes;
    }

    // Reference copy of the legacy AgentSensorSmoke math (AgentSmokeSensorInterference.cpp,
    // AgentSensorCore 0.5.0). Only used to prove that the port onto UBot::Medium keeps the numbers.
    namespace LegacyOracle
    {
        struct FSmokeCell
        {
            FVector WorldPosition = FVector::ZeroVector;
            float RadiusCentimeters = 150.0f;
            float Density = 0.0f;
            float EdgeFactor = 1.0f;

            float EffectiveDensity() const
            {
                return FMath::Max(0.0f, Density) * FMath::Clamp(EdgeFactor, 0.0f, 1.0f);
            }
        };

        struct FSmokeField
        {
            FBox Bounds = FBox(ForceInit);
            TArray<FSmokeCell> Cells;
        };

        bool IntersectSegmentSphere(
            const FVector& SegmentStart,
            const FVector& SegmentDirection,
            float SegmentLength,
            const FSmokeCell& Cell,
            float& OutDistanceCentimeters)
        {
            const FVector ToStart = SegmentStart - Cell.WorldPosition;
            const float Radius = FMath::Max(1.0f, Cell.RadiusCentimeters);
            const float C = FVector::DotProduct(ToStart, ToStart) - Radius * Radius;
            if (C <= 0.0f)
            {
                OutDistanceCentimeters = 0.0f;
                return true;
            }

            const float B = FVector::DotProduct(ToStart, SegmentDirection);
            if (B > 0.0f)
            {
                return false;
            }

            const float Discriminant = B * B - C;
            if (Discriminant < 0.0f)
            {
                return false;
            }

            const float Distance = -B - FMath::Sqrt(Discriminant);
            if (Distance < 0.0f || Distance > SegmentLength)
            {
                return false;
            }

            OutDistanceCentimeters = Distance;
            return true;
        }

        bool FindFirstSmokeIntersection(
            const FVector& SegmentStart,
            const FVector& SegmentEnd,
            const TArray<FSmokeField>& Fields,
            float DensityThreshold,
            float& OutDistanceCentimeters,
            float& OutDensity)
        {
            OutDistanceCentimeters = 0.0f;
            OutDensity = 0.0f;
            const FVector Segment = SegmentEnd - SegmentStart;
            const float SegmentLength = Segment.Size();
            if (SegmentLength <= UE_SMALL_NUMBER)
            {
                return false;
            }

            const FVector SegmentDirection = Segment / SegmentLength;
            float BestDistance = TNumericLimits<float>::Max();
            float BestDensity = 0.0f;
            bool bFound = false;
            for (const FSmokeField& Field : Fields)
            {
                if (!Field.Bounds.IsValid)
                {
                    continue;
                }
                const FBox SegmentBounds(SegmentStart.ComponentMin(SegmentEnd), SegmentStart.ComponentMax(SegmentEnd));
                if (!Field.Bounds.Intersect(SegmentBounds))
                {
                    continue;
                }

                for (const FSmokeCell& Cell : Field.Cells)
                {
                    const float EffectiveDensity = Cell.EffectiveDensity();
                    if (EffectiveDensity < DensityThreshold)
                    {
                        continue;
                    }

                    float Distance = 0.0f;
                    if (IntersectSegmentSphere(SegmentStart, SegmentDirection, SegmentLength, Cell, Distance)
                        && Distance < BestDistance)
                    {
                        BestDistance = Distance;
                        BestDensity = EffectiveDensity;
                        bFound = true;
                    }
                }
            }

            if (!bFound)
            {
                return false;
            }

            OutDistanceCentimeters = BestDistance;
            OutDensity = BestDensity;
            return true;
        }

        FVector CameraPixelDirectionLocal(int32 PixelX, int32 PixelY, float Fx, float Fy, float Cx, float Cy)
        {
            if (Fx <= UE_SMALL_NUMBER || Fy <= UE_SMALL_NUMBER)
            {
                return FVector::ForwardVector;
            }
            return FVector(
                1.0,
                (static_cast<float>(PixelX) - Cx) / Fx,
                -(static_cast<float>(PixelY) - Cy) / Fy).GetSafeNormal();
        }

        void ApplySmokeToBGRAPixels(
            const FTransform& SensorTransform,
            int32 Width,
            int32 Height,
            float Fx,
            float Fy,
            float Cx,
            float Cy,
            float MaximumDistanceMeters,
            const TArray<FSmokeField>& Fields,
            float DensityThreshold,
            const FLinearColor& SmokeTint,
            float OpacityScale,
            float MaximumOpacity,
            TArray<uint8>& PixelBytes)
        {
            if (Fields.IsEmpty() || Width <= 0 || Height <= 0 || PixelBytes.Num() != Width * Height * 4 || MaximumDistanceMeters <= 0.0f)
            {
                return;
            }

            const FVector Origin = SensorTransform.GetLocation();
            const float MaximumDistanceCentimeters = MaximumDistanceMeters * 100.0f;
            for (int32 Y = 0; Y < Height; ++Y)
            {
                for (int32 X = 0; X < Width; ++X)
                {
                    const FVector LocalDirection = CameraPixelDirectionLocal(X, Y, Fx, Fy, Cx, Cy);
                    const FVector WorldDirection = SensorTransform.TransformVectorNoScale(LocalDirection).GetSafeNormal();
                    float SmokeDistanceCentimeters = 0.0f;
                    float SmokeDensity = 0.0f;
                    if (!FindFirstSmokeIntersection(
                            Origin,
                            Origin + WorldDirection * MaximumDistanceCentimeters,
                            Fields,
                            DensityThreshold,
                            SmokeDistanceCentimeters,
                            SmokeDensity))
                    {
                        continue;
                    }

                    const float Alpha = FMath::Clamp(SmokeDensity * FMath::Max(0.0f, OpacityScale), 0.0f, MaximumOpacity);
                    const int32 Offset = (Y * Width + X) * 4;
                    const FLinearColor SourceColor(
                        static_cast<float>(PixelBytes[Offset + 2]) / 255.0f,
                        static_cast<float>(PixelBytes[Offset + 1]) / 255.0f,
                        static_cast<float>(PixelBytes[Offset]) / 255.0f,
                        static_cast<float>(PixelBytes[Offset + 3]) / 255.0f);
                    const FLinearColor Blended = FMath::Lerp(SourceColor, SmokeTint, Alpha).GetClamped();
                    PixelBytes[Offset] = static_cast<uint8>(FMath::RoundToInt(Blended.B * 255.0f));
                    PixelBytes[Offset + 1] = static_cast<uint8>(FMath::RoundToInt(Blended.G * 255.0f));
                    PixelBytes[Offset + 2] = static_cast<uint8>(FMath::RoundToInt(Blended.R * 255.0f));
                    PixelBytes[Offset + 3] = static_cast<uint8>(FMath::RoundToInt(SourceColor.A * 255.0f));
                }
            }
        }

        void ApplySmokeToDepthValues(
            const FTransform& SensorTransform,
            int32 Width,
            int32 Height,
            float Fx,
            float Fy,
            float Cx,
            float Cy,
            float MinimumDepthMeters,
            float MaximumDepthMeters,
            const TArray<FSmokeField>& Fields,
            float DensityThreshold,
            TArray<float>& DepthValuesMeters)
        {
            if (Fields.IsEmpty() || Width <= 0 || Height <= 0 || DepthValuesMeters.Num() != Width * Height || MaximumDepthMeters <= MinimumDepthMeters)
            {
                return;
            }

            const FVector Origin = SensorTransform.GetLocation();
            for (int32 Y = 0; Y < Height; ++Y)
            {
                for (int32 X = 0; X < Width; ++X)
                {
                    const int32 Index = Y * Width + X;
                    const float ExistingDepthMeters = DepthValuesMeters[Index];
                    const float SegmentMaxMeters = ExistingDepthMeters > 0.0f
                        ? FMath::Min(ExistingDepthMeters, MaximumDepthMeters)
                        : MaximumDepthMeters;
                    if (SegmentMaxMeters <= MinimumDepthMeters)
                    {
                        continue;
                    }

                    const FVector LocalDirection = CameraPixelDirectionLocal(X, Y, Fx, Fy, Cx, Cy);
                    const FVector WorldDirection = SensorTransform.TransformVectorNoScale(LocalDirection).GetSafeNormal();
                    float SmokeDistanceCentimeters = 0.0f;
                    float SmokeDensity = 0.0f;
                    if (!FindFirstSmokeIntersection(
                            Origin,
                            Origin + WorldDirection * SegmentMaxMeters * 100.0f,
                            Fields,
                            DensityThreshold,
                            SmokeDistanceCentimeters,
                            SmokeDensity))
                    {
                        continue;
                    }

                    const float SmokeDistanceMeters = SmokeDistanceCentimeters * 0.01f;
                    DepthValuesMeters[Index] = SmokeDistanceMeters >= MinimumDepthMeters && SmokeDistanceMeters <= MaximumDepthMeters
                        ? SmokeDistanceMeters
                        : 0.0f;
                }
            }
        }
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorMediumLidarStopsAtEffectiveSmokeTest, "UBotSensor.SmokeInterference.LidarStopsAtEffectiveSmoke", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorMediumLidarStopsAtEffectiveSmokeTest::RunTest(const FString& Parameters)
{
    using namespace UBotSensorMediumTests;

    UBot::SensorMedium::FQuerySettings Settings;
    Settings.DensityThreshold = 0.10f;

    TArray<FUBotMediumSnapshot> Snapshots = { MakeSnapshot({ MakeCell(FVector(300.0, 0.0, 0.0), 100.0f, 1.0f, 1.0f) }) };

    float DistanceCentimeters = 0.0f;
    float Density = 0.0f;
    TestTrue(
        TEXT("Ray reports the first smoke boundary as a hit"),
        UBot::Medium::FindFirstIntersection(
            FVector::ZeroVector,
            FVector(1000.0, 0.0, 0.0),
            Snapshots,
            Settings.DensityThreshold,
            DistanceCentimeters,
            Density));
    TestTrue(TEXT("Smoke entry distance is before the cell center"), FMath::IsNearlyEqual(DistanceCentimeters, 200.0f, 0.001f));
    TestTrue(TEXT("Effective density is exported"), FMath::IsNearlyEqual(Density, 1.0f, 0.001f));

    Snapshots[0].Cells[0].Density = 0.05f;
    TestFalse(
        TEXT("Smoke below the effective threshold is ignored"),
        UBot::Medium::FindFirstIntersection(
            FVector::ZeroVector,
            FVector(1000.0, 0.0, 0.0),
            Snapshots,
            Settings.DensityThreshold,
            DistanceCentimeters,
            Density));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorMediumCameraReadbackAppliesSmokeTest, "UBotSensor.SmokeInterference.CameraReadbackAppliesSmoke", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorMediumCameraReadbackAppliesSmokeTest::RunTest(const FString& Parameters)
{
    using namespace UBotSensorMediumTests;

    const TArray<FUBotMediumSnapshot> Snapshots = { MakeSnapshot({ MakeCell(FVector(200.0, 0.0, 0.0), 100.0f, 1.0f, 1.0f) }) };

    UBot::SensorMedium::FCameraSettings CameraSettings;
    CameraSettings.Tint = FLinearColor(0.5f, 0.5f, 0.5f, 1.0f);
    CameraSettings.OpacityScale = 1.0f;
    CameraSettings.MaximumOpacity = 1.0f;

    TArray<uint8> PixelBytes;
    PixelBytes.Add(20);
    PixelBytes.Add(40);
    PixelBytes.Add(80);
    PixelBytes.Add(255);
    UBot::SensorMedium::ApplyToBGRAPixels(
        FTransform::Identity,
        1,
        1,
        1.0f,
        1.0f,
        0.0f,
        0.0f,
        10.0f,
        Snapshots,
        CameraSettings,
        PixelBytes);
    TestTrue(TEXT("RGB readback is blended toward the smoke tint"), PixelBytes[0] > 20 && PixelBytes[1] > 40 && PixelBytes[2] > 80);

    TArray<float> DepthValuesMeters;
    DepthValuesMeters.Add(10.0f);
    UBot::SensorMedium::ApplyToDepthValues(
        FTransform::Identity,
        1,
        1,
        1.0f,
        1.0f,
        0.0f,
        0.0f,
        0.1f,
        10.0f,
        Snapshots,
        CameraSettings,
        DepthValuesMeters);
    TestTrue(TEXT("Depth readback is clamped to the smoke entry distance"), FMath::IsNearlyEqual(DepthValuesMeters[0], 1.0f, 0.001f));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorMediumMatchesLegacyMathTest, "UBotSensor.Medium.MatchesLegacyMath", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorMediumMatchesLegacyMathTest::RunTest(const FString& Parameters)
{
    using namespace UBotSensorMediumTests;

    // Random smoke fields in front of a tilted camera, fed both to the legacy reference and to the port.
    // Densities stay in 0..1, the documented range of the legacy smoke actors (Core clamps above 1).
    FRandomStream Stream(20261009);
    TArray<LegacyOracle::FSmokeField> Fields;
    TArray<FUBotMediumSnapshot> Snapshots;
    for (int32 FieldIndex = 0; FieldIndex < 3; ++FieldIndex)
    {
        LegacyOracle::FSmokeField Field;
        FUBotMediumSnapshot Snapshot;
        // Legacy actors report one radius for all of their cells.
        const float RadiusCentimeters = Stream.FRandRange(40.0f, 220.0f);
        const int32 CellCount = Stream.RandRange(4, 9);
        float MaximumCellRadiusCentimeters = 0.0f;
        for (int32 CellIndex = 0; CellIndex < CellCount; ++CellIndex)
        {
            LegacyOracle::FSmokeCell Cell;
            Cell.WorldPosition = FVector(Stream.FRandRange(150.0f, 1600.0f), Stream.FRandRange(-500.0f, 500.0f), Stream.FRandRange(-300.0f, 300.0f));
            Cell.RadiusCentimeters = RadiusCentimeters;
            Cell.Density = Stream.FRandRange(0.0f, 1.0f);
            Cell.EdgeFactor = Stream.FRandRange(-0.2f, 1.3f);
            Field.Cells.Add(Cell);
            Field.Bounds += Cell.WorldPosition;
            MaximumCellRadiusCentimeters = FMath::Max(MaximumCellRadiusCentimeters, Cell.RadiusCentimeters);
            Snapshot.Cells.Add(MakeCell(Cell.WorldPosition, Cell.RadiusCentimeters, Cell.Density, Cell.EdgeFactor));
        }
        Field.Bounds = Field.Bounds.ExpandBy(MaximumCellRadiusCentimeters);
        UBot::Medium::FinalizeSnapshot(Snapshot);
        Fields.Add(MoveTemp(Field));
        Snapshots.Add(MoveTemp(Snapshot));
    }

    const FTransform SensorTransform(FRotator(4.0, 8.0, 2.0), FVector(10.0, -20.0, 30.0));
    constexpr int32 Width = 32;
    constexpr int32 Height = 24;
    constexpr float Fx = 16.0f;
    constexpr float Fy = 16.0f;
    constexpr float Cx = 16.0f;
    constexpr float Cy = 12.0f;

    UBot::SensorMedium::FCameraSettings CameraSettings;
    CameraSettings.DensityThreshold = 0.1f;

    TArray<uint8> SourcePixels;
    SourcePixels.SetNumUninitialized(Width * Height * 4);
    for (uint8& Byte : SourcePixels)
    {
        Byte = static_cast<uint8>(Stream.RandRange(0, 255));
    }

    TArray<uint8> ExpectedPixels = SourcePixels;
    LegacyOracle::ApplySmokeToBGRAPixels(
        SensorTransform, Width, Height, Fx, Fy, Cx, Cy, 12.0f, Fields,
        CameraSettings.DensityThreshold, CameraSettings.Tint, CameraSettings.OpacityScale, CameraSettings.MaximumOpacity,
        ExpectedPixels);
    TArray<uint8> ActualPixels = SourcePixels;
    UBot::SensorMedium::ApplyToBGRAPixels(SensorTransform, Width, Height, Fx, Fy, Cx, Cy, 12.0f, Snapshots, CameraSettings, ActualPixels);

    // Modules are built with /fp:fast, so allow one step of rounding difference per byte.
    int32 ChangedPixels = 0;
    int32 MismatchedBytes = 0;
    for (int32 PixelIndex = 0; PixelIndex < Width * Height; ++PixelIndex)
    {
        bool bChanged = false;
        for (int32 Channel = 0; Channel < 4; ++Channel)
        {
            const int32 Offset = PixelIndex * 4 + Channel;
            bChanged |= ExpectedPixels[Offset] != SourcePixels[Offset];
            if (FMath::Abs(static_cast<int32>(ExpectedPixels[Offset]) - static_cast<int32>(ActualPixels[Offset])) > 1)
            {
                if (MismatchedBytes == 0)
                {
                    AddError(FString::Printf(TEXT("First BGRA mismatch at pixel %d channel %d: legacy %d, port %d"),
                        PixelIndex, Channel, static_cast<int32>(ExpectedPixels[Offset]), static_cast<int32>(ActualPixels[Offset])));
                }
                ++MismatchedBytes;
            }
        }
        ChangedPixels += bChanged ? 1 : 0;
    }
    TestTrue(TEXT("The scene blends a meaningful part of the image"), ChangedPixels > Width * Height / 20);
    TestEqual(TEXT("BGRA output matches the legacy implementation"), MismatchedBytes, 0);

    TArray<float> SourceDepths;
    SourceDepths.SetNumUninitialized(Width * Height);
    for (float& Depth : SourceDepths)
    {
        Depth = Stream.FRand() < 0.1f ? 0.0f : Stream.FRandRange(0.2f, 18.0f);
    }

    TArray<float> ExpectedDepths = SourceDepths;
    LegacyOracle::ApplySmokeToDepthValues(SensorTransform, Width, Height, Fx, Fy, Cx, Cy, 0.1f, 15.0f, Fields, CameraSettings.DensityThreshold, ExpectedDepths);
    TArray<float> ActualDepths = SourceDepths;
    UBot::SensorMedium::ApplyToDepthValues(SensorTransform, Width, Height, Fx, Fy, Cx, Cy, 0.1f, 15.0f, Snapshots, CameraSettings, ActualDepths);

    int32 ChangedDepths = 0;
    int32 MismatchedDepths = 0;
    for (int32 Index = 0; Index < ExpectedDepths.Num(); ++Index)
    {
        ChangedDepths += ExpectedDepths[Index] != SourceDepths[Index] ? 1 : 0;
        if (!FMath::IsNearlyEqual(ExpectedDepths[Index], ActualDepths[Index], 1.0e-4f))
        {
            if (MismatchedDepths == 0)
            {
                AddError(FString::Printf(TEXT("First depth mismatch at pixel %d: legacy %.6f m, port %.6f m"), Index, ExpectedDepths[Index], ActualDepths[Index]));
            }
            ++MismatchedDepths;
        }
    }
    TestTrue(TEXT("The scene changes a meaningful part of the depth image"), ChangedDepths > Width * Height / 20);
    TestEqual(TEXT("Depth output matches the legacy implementation"), MismatchedDepths, 0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorMediumLegacyAndInterfacePathsTest, "UBotSensor.Medium.LegacyAndInterfacePaths", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorMediumLegacyAndInterfacePathsTest::RunTest(const FString& Parameters)
{
    using namespace UBotSensorMediumTests;

    FScopedTestWorld TestWorld;
    UWorld* World = TestWorld.Get();
    if (!TestNotNull(TEXT("Transient world"), World))
    {
        return false;
    }

    AUBotSensorMediumTestLegacyActor* Legacy = TestWorld.Spawn<AUBotSensorMediumTestLegacyActor>(TEXT("UBotSensorTest-LegacySmoke"));
    AUBotSensorMediumTestInterfaceActor* Medium = TestWorld.Spawn<AUBotSensorMediumTestInterfaceActor>(TEXT("UBotSensorTest-InterfaceMedium"));
    if (!TestNotNull(TEXT("Legacy smoke actor"), Legacy) || !TestNotNull(TEXT("Interface medium actor"), Medium))
    {
        return false;
    }

    // The same cells on both paths: the legacy actor reports one radius in meters through reflection,
    // the interface actor reports centimeters per cell. The last cell is below the default threshold.
    constexpr int32 CellCount = 3;
    const FVector Positions[CellCount] = { FVector(300.0, 0.0, 0.0), FVector(500.0, 120.0, -40.0), FVector(900.0, 0.0, 0.0) };
    const float Densities[CellCount] = { 0.8f, 0.6f, 0.02f };
    const float EdgeFactors[CellCount] = { 1.0f, 0.5f, 1.0f };
    Legacy->Tags.Add(UBot::SensorMedium::LegacySmokeVolumeTag());
    Legacy->CellVisualRadiusMeters = 1.5f;
    Medium->Snapshot.Tint = FLinearColor(1.0f, 0.0f, 0.0f, 1.0f);
    for (int32 Index = 0; Index < CellCount; ++Index)
    {
        FUBotSensorMediumTestLegacyCell& LegacyCell = Legacy->Cells.AddDefaulted_GetRef();
        LegacyCell.WorldPosition = Positions[Index];
        LegacyCell.Density = Densities[Index];
        LegacyCell.EdgeFactor = EdgeFactors[Index];
        Medium->Snapshot.Cells.Add(MakeCell(Positions[Index], 150.0f, Densities[Index], EdgeFactors[Index]));
    }

    UBot::SensorMedium::FQuerySettings Settings;
    TArray<FUBotMediumSnapshot> Media;
    UBot::SensorMedium::CollectMedia(*World, Settings, Media);
    if (!TestEqual(TEXT("Each discovery path yields one snapshot"), Media.Num(), 2))
    {
        return false;
    }
    const FUBotMediumSnapshot FromInterface = Media[0];
    const FUBotMediumSnapshot FromLegacy = Media[1];
    TestTrue(TEXT("Core interface snapshots come first"), FromInterface.Source.Get() == Medium);
    TestTrue(TEXT("Legacy snapshot is attributed to its actor"), FromLegacy.Source.Get() == Legacy);
    TestEquivalentSnapshots(*this, TEXT("Default settings"), FromInterface, FromLegacy);
    TestEqual(TEXT("Cells below the density threshold are dropped"), FromLegacy.Cells.Num(), 2);
    if (FromLegacy.Cells.Num() == 2)
    {
        TestEqual(TEXT("Legacy radius comes from CellVisualRadiusMeters"), FromLegacy.Cells[0].RadiusCentimeters, 150.0f);
        TestEqual(TEXT("Legacy edge factor is read through reflection"), FromLegacy.Cells[1].EdgeFactor, 0.5f);
    }
    TestTrue(TEXT("Legacy snapshots are smoke"), FromLegacy.MediumType == FName(TEXT("Smoke")));

    float InterfaceDistance = 0.0f;
    float InterfaceDensity = 0.0f;
    float LegacyDistance = 0.0f;
    float LegacyDensity = 0.0f;
    TestTrue(TEXT("Interface medium is hit"), UBot::Medium::FindFirstIntersection(FVector::ZeroVector, FVector(1000.0, 0.0, 0.0), { FromInterface }, Settings.DensityThreshold, InterfaceDistance, InterfaceDensity));
    TestTrue(TEXT("Legacy medium is hit"), UBot::Medium::FindFirstIntersection(FVector::ZeroVector, FVector(1000.0, 0.0, 0.0), { FromLegacy }, Settings.DensityThreshold, LegacyDistance, LegacyDensity));
    TestEqual(TEXT("Entry distance"), LegacyDistance, 150.0f, 0.001f);
    TestEqual(TEXT("Entry density"), LegacyDensity, 0.8f, 0.0001f);
    TestTrue(TEXT("Both paths give the same intersection"), InterfaceDistance == LegacyDistance && InterfaceDensity == LegacyDensity);

    // Radius scale and threshold are applied the same way on both paths.
    Settings.CellRadiusScale = 2.0f;
    Settings.DensityThreshold = 0.5f;
    UBot::SensorMedium::CollectMedia(*World, Settings, Media);
    if (TestEqual(TEXT("Scaled: both paths still yield a snapshot"), Media.Num(), 2))
    {
        TestEquivalentSnapshots(*this, TEXT("Scaled radius, higher threshold"), Media[0], Media[1]);
        TestTrue(TEXT("Scaled: only the dense cell is left at twice the radius"),
            Media[1].Cells.Num() == 1 && Media[1].Cells[0].RadiusCentimeters == 300.0f);
    }

    // Radius fallbacks of the legacy path.
    Settings = UBot::SensorMedium::FQuerySettings();
    Legacy->CellVisualRadiusMeters = 0.0f;
    Legacy->CellSizeMeters = 2.0f;
    UBot::SensorMedium::CollectMedia(*World, Settings, Media);
    TestTrue(TEXT("CellSizeMeters is the first radius fallback"),
        Media.Num() == 2 && !Media[1].Cells.IsEmpty() && Media[1].Cells[0].RadiusCentimeters == 200.0f);
    Legacy->CellSizeMeters = 0.0f;
    Settings.DefaultCellRadiusCentimeters = 80.0f;
    UBot::SensorMedium::CollectMedia(*World, Settings, Media);
    TestTrue(TEXT("DefaultCellRadiusCentimeters is the last radius fallback"),
        Media.Num() == 2 && !Media[1].Cells.IsEmpty() && Media[1].Cells[0].RadiusCentimeters == 80.0f);
    Settings = UBot::SensorMedium::FQuerySettings();
    Legacy->CellVisualRadiusMeters = 1.5f;
    Legacy->CellSizeMeters = 1.0f;

    // Discovery rules.
    Legacy->bSmokeFieldActive = false;
    UBot::SensorMedium::CollectMedia(*World, Settings, Media);
    TestTrue(TEXT("Inactive legacy smoke is skipped"), Media.Num() == 1 && Media[0].Source.Get() == Medium);
    Legacy->bSmokeFieldActive = true;

    Settings.LegacySmokeVolumeTag = NAME_None;
    UBot::SensorMedium::CollectMedia(*World, Settings, Media);
    TestTrue(TEXT("A None tag disables only the legacy path"), Media.Num() == 1 && Media[0].Source.Get() == Medium);
    Settings = UBot::SensorMedium::FQuerySettings();

    Legacy->Tags.Reset();
    UBot::SensorMedium::CollectMedia(*World, Settings, Media);
    TestTrue(TEXT("Untagged actors are not legacy media"), Media.Num() == 1 && Media[0].Source.Get() == Medium);
    const FName CustomTag(TEXT("UBotSensorTest.Smoke"));
    Legacy->Tags.Add(CustomTag);
    Settings.LegacySmokeVolumeTag = CustomTag;
    UBot::SensorMedium::CollectMedia(*World, Settings, Media);
    TestTrue(TEXT("The legacy tag is configurable"), Media.Num() == 2 && Media[1].Source.Get() == Legacy);
    Settings = UBot::SensorMedium::FQuerySettings();
    Legacy->Tags.Reset();
    Legacy->Tags.Add(UBot::SensorMedium::LegacySmokeVolumeTag());

    // The interface actor also exposes GetSmokeCellSnapshots, so only the interface check keeps it out of the legacy path.
    Medium->Tags.Add(UBot::SensorMedium::LegacySmokeVolumeTag());
    UBot::SensorMedium::CollectMedia(*World, Settings, Media);
    TestTrue(TEXT("A tagged interface medium is collected once"),
        Media.Num() == 2 && Media[0].Source.Get() == Medium && Media[1].Source.Get() == Legacy);
    Medium->Tags.Reset();

    Medium->Snapshot.bActive = false;
    UBot::SensorMedium::CollectMedia(*World, Settings, Media);
    TestTrue(TEXT("Inactive interface media are skipped"), Media.Num() == 1 && Media[0].Source.Get() == Legacy);
    Medium->Snapshot.bActive = true;

    // Camera tint: Core interface snapshots blend towards their own tint, legacy and hand-built ones
    // towards the camera settings tint. The nearest medium decides when tints differ.
    UBot::SensorMedium::FCameraSettings CameraSettings;
    CameraSettings.Tint = FLinearColor(0.0f, 0.0f, 1.0f, 1.0f);
    CameraSettings.OpacityScale = 1.0f;
    CameraSettings.MaximumOpacity = 1.0f;

    const TArray<uint8> InterfacePixel = RenderBlackPixel({ FromInterface }, CameraSettings);
    TestTrue(TEXT("Interface medium uses its own tint"), InterfacePixel[2] > 150 && InterfacePixel[0] == 0 && InterfacePixel[3] == 255);
    const TArray<uint8> LegacyPixel = RenderBlackPixel({ FromLegacy }, CameraSettings);
    TestTrue(TEXT("Legacy medium uses the camera tint"), LegacyPixel[0] > 150 && LegacyPixel[2] == 0 && LegacyPixel[3] == 255);

    FUBotMediumSnapshot FartherInterface = FromInterface;
    for (FUBotMediumCell& Cell : FartherInterface.Cells)
    {
        Cell.WorldPositionCentimeters.X += 400.0;
    }
    UBot::Medium::FinalizeSnapshot(FartherInterface);
    const TArray<uint8> LegacyInFront = RenderBlackPixel({ FartherInterface, FromLegacy }, CameraSettings);
    TestTrue(TEXT("Nearer legacy medium decides the colour"), LegacyInFront[0] > 150 && LegacyInFront[2] == 0);

    FUBotMediumSnapshot FartherLegacy = FromLegacy;
    for (FUBotMediumCell& Cell : FartherLegacy.Cells)
    {
        Cell.WorldPositionCentimeters.X += 400.0;
    }
    UBot::Medium::FinalizeSnapshot(FartherLegacy);
    const TArray<uint8> InterfaceInFront = RenderBlackPixel({ FartherLegacy, FromInterface }, CameraSettings);
    TestTrue(TEXT("Nearer interface medium decides the colour"), InterfaceInFront[2] > 150 && InterfaceInFront[0] == 0);
    return true;
}

#endif
