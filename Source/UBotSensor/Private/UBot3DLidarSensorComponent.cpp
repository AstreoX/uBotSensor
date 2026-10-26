#include "UBot3DLidarSensorComponent.h"

#include "CollisionQueryParams.h"
#include "Async/ParallelFor.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformProcess.h"
#include "Modules/ModuleManager.h"
#include "Physics/Experimental/PhysScene_Chaos.h"
#include "PhysicsEngine/ClusterUnionComponent.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"
#include "DrawDebugHelpers.h"
#include "Engine/HitResult.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "HAL/PlatformTime.h"
#include "UBotPerceptionMedium.h"
#include "UBotPointCloudFrame.h"
#include "UBotSensorMath.h"
#include "UBotSensorMedium.h"
#include "UObject/ConstructorHelpers.h"
#include "UObject/UObjectIterator.h"
#if WITH_EDITOR
#include "CollisionAnalyzerModule.h"
#include "ICollisionAnalyzer.h"
#endif

namespace UBot3DLidarPrivate
{
    // Diagnostic baseline switch; no change to the sensor model or published frame layout.
    TAutoConsoleVariable<int32> UseDirectionCache(TEXT("ubot.Sensor.Lidar.DirectionCache"), 1, TEXT("Reuse local ray directions; 0 recomputes the baseline directions each scan."));
    // Conservative validated boundary: 32x720 passed native and full-pipeline paired gates.
    // Small scans showed an unstable median benefit, so Auto leaves them serial.
    TAutoConsoleVariable<int32> ParallelMinRays(TEXT("ubot.Sensor.Lidar.ParallelMinRays"), 23040, TEXT("Minimum rays for Auto parallel queries. Parallel mode bypasses this size threshold."));
    TAutoConsoleVariable<int32> MaxQueryTasks(TEXT("ubot.Sensor.Lidar.MaxQueryTasks"), 8, TEXT("Maximum simultaneous LiDAR query blocks; avoids contending on the scene lock with one task per channel."));
    // Return strength of a surface hit: 255 when the surface faces the ray, falling to 0 at grazing
    // or back-facing incidence.
    float SurfaceIntensity(const FVector& RayDirectionWorld, const FVector& ImpactNormal)
    {
        return 255.0f * FMath::Clamp(static_cast<float>(FVector::DotProduct(-RayDirectionWorld, ImpactNormal)), 0.0f, 1.0f);
    }
}

UUBot3DLidarSensorComponent::UUBot3DLidarSensorComponent()
{
    static ConstructorHelpers::FObjectFinder<UStaticMesh> SensorMesh(TEXT("/UBotSensor/Models/SM_UBotSensor_Lidar3D.SM_UBotSensor_Lidar3D"));
    if (SensorMesh.Succeeded())
    {
        SetStaticMesh(SensorMesh.Object);
    }
    SensorId = TEXT("lidar_3d");
    FrameId = TEXT("lidar_3d");
    UpdateRateHz = 10.0f;
}

FName UUBot3DLidarSensorComponent::GetSensorTypeName() const
{
    static const FName SensorTypeName(TEXT("Lidar3D"));
    return SensorTypeName;
}

bool UUBot3DLidarSensorComponent::CaptureSensorData(FUBotFrameHeader& Header, FString& OutErrorMessage)
{
    check(IsInGameThread());
    TRACE_CPUPROFILER_EVENT_SCOPE(UBot_Lidar_Capture);
    if (bCaptureInProgress)
    {
        OutErrorMessage = TEXT("A 3D LiDAR scan is already in progress.");
        MarkHeader(Header, EUBotSampleStatus::Error, false);
        return false;
    }
    TGuardValue<bool> CaptureGuard(bCaptureInProgress, true);
    UWorld* World = GetWorld();
    if (!World)
    {
        OutErrorMessage = TEXT("3D LiDAR has no valid world.");
        MarkHeader(Header, EUBotSampleStatus::Error, false);
        return false;
    }
    if (MinimumRangeMeters < 0.0f || MaximumRangeMeters <= MinimumRangeMeters || HorizontalFovDegrees <= 0.0f || NumberOfChannels < 1 || HorizontalSamples < 1)
    {
        OutErrorMessage = TEXT("3D LiDAR scan limits are invalid.");
        MarkHeader(Header, EUBotSampleStatus::Error, false);
        return false;
    }
    const int64 TotalRays64 = static_cast<int64>(NumberOfChannels) * static_cast<int64>(HorizontalSamples);
    if (TotalRays64 > MaximumRaysPerScan || TotalRays64 > MAX_int32)
    {
        OutErrorMessage = FString::Printf(TEXT("3D LiDAR requested %lld rays, exceeding Maximum Rays Per Scan (%d)."), TotalRays64, MaximumRaysPerScan);
        MarkHeader(Header, EUBotSampleStatus::Error, false);
        return false;
    }

    const double CaptureStartSeconds = FPlatformTime::Seconds();
    LastTimings = FUBotLidarCaptureTimings();
    const int32 TotalRays = static_cast<int32>(TotalRays64);
    const FTransform SensorTransform = GetComponentTransform();
    const FVector Origin = SensorTransform.GetLocation();
    const float MaximumRangeCentimeters = MaximumRangeMeters * 100.0f;
    FCollisionQueryParams QueryParams(SCENE_QUERY_STAT(UBot3DLidar), bTraceComplex);
    if (bIgnoreOwner && GetOwner())
    {
        QueryParams.AddIgnoredActor(GetOwner());
    }
    for (AActor* IgnoredActor : IgnoredActors)
    {
        if (IgnoredActor)
        {
            QueryParams.AddIgnoredActor(IgnoredActor);
        }
    }

    UBot::SensorMedium::FQuerySettings MediumSettings;
    MediumSettings.LegacySmokeVolumeTag = SmokeVolumeActorTag;
    MediumSettings.DensityThreshold = SmokeDensityThreshold;
    MediumSettings.CellRadiusScale = SmokeCellRadiusScale;
    TArray<FUBotMediumSnapshot> MediumSnapshots;
    if (bEnableSmokeInterference)
    {
        UBot::SensorMedium::CollectMedia(*World, MediumSettings, MediumSnapshots);
    }

    UUBotPointCloudFrame* NewFrame = NewObject<UUBotPointCloudFrame>(this);
    NewFrame->SensorTransformWorld = SensorTransform;
    NewFrame->TotalRayCount = TotalRays;
    NewFrame->ChannelCount = NumberOfChannels;
    NewFrame->HorizontalSampleCount = HorizontalSamples;
    NewFrame->HorizontalFovRadians = FMath::DegreesToRadians(HorizontalFovDegrees);
    NewFrame->VerticalFovUpRadians = FMath::DegreesToRadians(VerticalFovUpDegrees);
    NewFrame->VerticalFovDownRadians = FMath::DegreesToRadians(VerticalFovDownDegrees);
    NewFrame->MinimumRangeMeters = MinimumRangeMeters;
    NewFrame->MaximumRangeMeters = MaximumRangeMeters;
    NewFrame->ScanPeriodSeconds = 1.0f / FMath::Clamp(UpdateRateHz, 0.1f, 240.0f);
    NewFrame->PointsMeters.Reserve(TotalRays);
    NewFrame->RingIndices.Reserve(TotalRays);
    NewFrame->TimeOffsetsSeconds.Reserve(TotalRays);
    NewFrame->Intensities.Reserve(TotalRays);

    DebugEndPoints.Reset();
    DebugHitFlags.Reset();
    RayResults.SetNum(TotalRays, EAllowShrinking::No);
    const bool bCacheDirections = UBot3DLidarPrivate::UseDirectionCache.GetValueOnGameThread() != 0;
    if (!bCacheDirections || CachedChannels != NumberOfChannels || CachedHorizontalSamples != HorizontalSamples
        || CachedHorizontalFov != HorizontalFovDegrees || CachedVerticalUp != VerticalFovUpDegrees || CachedVerticalDown != VerticalFovDownDegrees)
    {
        TRACE_CPUPROFILER_EVENT_SCOPE(UBot_Lidar_Directions);
        LocalDirections.SetNumUninitialized(TotalRays, EAllowShrinking::No);
        for (int32 Ring = 0; Ring < NumberOfChannels; ++Ring)
        {
            const float Vertical = UBot::SensorMath::LidarVerticalAngleDegrees(Ring, NumberOfChannels, VerticalFovDownDegrees, VerticalFovUpDegrees);
            for (int32 Column = 0; Column < HorizontalSamples; ++Column)
            {
                LocalDirections[Ring * HorizontalSamples + Column] = UBot::SensorMath::LidarDirectionLocal(
                    UBot::SensorMath::LidarHorizontalAngleDegrees(Column, HorizontalSamples, HorizontalFovDegrees), Vertical);
            }
        }
        CachedChannels = NumberOfChannels; CachedHorizontalSamples = HorizontalSamples;
        CachedHorizontalFov = HorizontalFovDegrees; CachedVerticalUp = VerticalFovUpDegrees; CachedVerticalDown = VerticalFovDownDegrees;
    }
    FPhysScene* Scene = World->GetPhysicsScene();
    bool bAnalyzerRecording = false;
#if WITH_EDITOR
    if (FModuleManager::Get().IsModuleLoaded(TEXT("CollisionAnalyzer")))
    {
        if (ICollisionAnalyzer* Analyzer = FCollisionAnalyzerModule::Get()) { bAnalyzerRecording = Analyzer->IsRecording(); }
    }
#endif
    bool bSafeParallelContext = Scene && Scene->GetSolver() && !Scene->GetSolver()->IsGameThreadFrozen() && !bAnalyzerRecording;
#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
    // Engine scene-query debug drawing is game-thread-only. Preserve its diagnostic behavior.
    bSafeParallelContext &= !World->DebugDrawSceneQueries(QueryParams.TraceTag);
#endif
    // Cluster-union extension queries use component-owned GT state. Until their equivalence is
    // validated, retain the original serial path for such worlds, including explicit Parallel.
    for (TObjectIterator<UClusterUnionComponent> It; bSafeParallelContext && It; ++It)
    {
        if (It->IsRegistered() && It->GetWorld() == World) { bSafeParallelContext = false; }
    }
    const int32 QueryTasks = FMath::Clamp(UBot3DLidarPrivate::MaxQueryTasks.GetValueOnGameThread(), 1, NumberOfChannels);
    const bool bParallel = bSafeParallelContext && QueryTasks > 1 && FPlatformProcess::SupportsMultithreading() &&
        (QueryMode == EUBotLidarQueryMode::Parallel || (QueryMode == EUBotLidarQueryMode::Auto
            && TotalRays >= UBot3DLidarPrivate::ParallelMinRays.GetValueOnGameThread()));
    LastTimings.bUsedParallel = bParallel;
    LastTimings.PreparationMilliseconds = (FPlatformTime::Seconds() - CaptureStartSeconds) * 1000;
    const double QueryStart = FPlatformTime::Seconds();
    {
        TRACE_CPUPROFILER_EVENT_SCOPE(UBot_Lidar_Queries);
        // UE's own async traces use this same public RaycastSingle path. Each query owns its scene
        // read lock; the joining game thread holds no outer physics lock. Worlds requiring GT
        // component extension queries were excluded above; suppress that branch in workers.
        FCollisionQueryParams WorkerParams = QueryParams;
        WorkerParams.bTraceIntoSubComponents = false;
        // This const getter can lazily normalize its component list. Finish that on the GT.
        WorkerParams.GetIgnoredComponents();
        const int32 Columns = HorizontalSamples;
        const int32 Channels = NumberOfChannels;
        const ECollisionChannel Channel = TraceChannel;
        const FVector* Directions = LocalDirections.GetData();
        FRayResult* Results = RayResults.GetData();
        const auto TraceRing = [&](int32 Ring)
        {
            for (int32 Column = 0; Column < Columns; ++Column)
            {
                const int32 Index = Ring * Columns + Column;
                FRayResult& Result = Results[Index];
                Result.WorldDirection = SensorTransform.TransformVectorNoScale(Directions[Index]).GetSafeNormal();
                FHitResult Hit;
                Result.bHit = World->LineTraceSingleByChannel(Hit, Origin,
                    Origin + Result.WorldDirection * MaximumRangeCentimeters, Channel, bParallel ? WorkerParams : QueryParams);
                Result.DistanceCentimeters = Result.bHit ? Hit.Distance : MaximumRangeCentimeters;
                Result.ImpactNormal = Hit.ImpactNormal;
            }
        };
        if (bParallel)
        {
            ParallelFor(QueryTasks, [&](int32 Task)
            {
                const int32 First = Channels * Task / QueryTasks, End = Channels * (Task + 1) / QueryTasks;
                for (int32 Ring = First; Ring < End; ++Ring) { TraceRing(Ring); }
            }, EParallelForFlags::Unbalanced);
        }
        else { for (int32 Ring = 0; Ring < Channels; ++Ring) { TraceRing(Ring); } }
    }
    LastTimings.QueryMilliseconds = (FPlatformTime::Seconds() - QueryStart) * 1000;
    const double AssemblyStart = FPlatformTime::Seconds();
    TRACE_CPUPROFILER_EVENT_SCOPE(UBot_Lidar_AssembleAndPublish);
    const int32 DebugStride = FMath::Max(1, FMath::DivideAndRoundUp(TotalRays, 128));
    for (int32 RingIndex = 0; RingIndex < NumberOfChannels; ++RingIndex)
    {
        for (int32 HorizontalIndex = 0; HorizontalIndex < HorizontalSamples; ++HorizontalIndex)
        {
            const int32 RayIndex = RingIndex * HorizontalSamples + HorizontalIndex;
            const FVector& LocalDirection = LocalDirections[RayIndex];
            const FRayResult& Result = RayResults[RayIndex];
            const FVector& WorldDirection = Result.WorldDirection;
            const bool bHit = Result.bHit;
            float RangeMeters = bHit ? Result.DistanceCentimeters * 0.01f : MaximumRangeMeters;
            float SmokeDistanceCentimeters = 0.0f;
            float SmokeDensity = 0.0f;
            const bool bSmokeHit = UBot::Medium::FindFirstIntersection(
                Origin,
                Origin + WorldDirection * Result.DistanceCentimeters,
                MediumSnapshots,
                MediumSettings.DensityThreshold,
                SmokeDistanceCentimeters,
                SmokeDensity);
            if (bSmokeHit)
            {
                RangeMeters = SmokeDistanceCentimeters * 0.01f;
            }
            if ((bHit || bSmokeHit) && bNoiseEnabled && RangeNoiseStdDevMeters > 0.0f)
            {
                RangeMeters += UBot::SensorMath::GaussianSample(RandomStream, RangeNoiseStdDevMeters);
            }
            const bool bValidHit = (bHit || bSmokeHit) && RangeMeters >= MinimumRangeMeters && RangeMeters <= MaximumRangeMeters;
            if (bValidHit)
            {
                // Only a surface return carries an intensity; returns truncated by a medium report 0.
                const float Intensity = (bHit && !bSmokeHit) ? UBot3DLidarPrivate::SurfaceIntensity(WorldDirection, Result.ImpactNormal) : 0.0f;
                NewFrame->PointsMeters.Add(LocalDirection * RangeMeters);
                NewFrame->RingIndices.Add(RingIndex);
                NewFrame->TimeOffsetsSeconds.Add(0.0f);
                NewFrame->Intensities.Add(Intensity);
            }
            if (bDebugVisualization && RayIndex % DebugStride == 0)
            {
                DebugEndPoints.Add(Origin + WorldDirection * (bValidHit ? RangeMeters * 100.0f : MaximumRangeCentimeters));
                DebugHitFlags.Add(bValidHit ? 1 : 0);
            }
        }
    }

    MarkHeader(Header, EUBotSampleStatus::Valid, true);
    NewFrame->Header = Header;
    NewFrame->HitPointCount = NewFrame->PointsMeters.Num();
    NewFrame->CaptureDurationMilliseconds = static_cast<float>((FPlatformTime::Seconds() - CaptureStartSeconds) * 1000.0);
    LastTimings.AssemblyMilliseconds = (FPlatformTime::Seconds() - AssemblyStart) * 1000;
    LatestFrame = NewFrame;
    const double DispatchStart = FPlatformTime::Seconds();
    {
        TRACE_CPUPROFILER_EVENT_SCOPE(UBot_Lidar_Dispatch);
        On3DLidarFrameReady.Broadcast(NewFrame);
    }
    LastTimings.DispatchMilliseconds = (FPlatformTime::Seconds() - DispatchStart) * 1000;
    if (bDebugVisualization)
    {
        DrawRuntimeDebug(DebugEndPoints, DebugHitFlags);
    }
    return true;
}

void UUBot3DLidarSensorComponent::OnSamplingStarted()
{
    RandomStream.Initialize(RandomSeed);
}

void UUBot3DLidarSensorComponent::DrawRuntimeDebug(const TArray<FVector>& EndPoints, const TArray<uint8>& HitFlags) const
{
    if (!GetWorld())
    {
        return;
    }
    const float Duration = 1.0f / FMath::Clamp(UpdateRateHz, 0.1f, 240.0f);
    for (int32 Index = 0; Index < EndPoints.Num(); ++Index)
    {
        const bool bHit = HitFlags.IsValidIndex(Index) && HitFlags[Index] != 0;
        DrawDebugLine(GetWorld(), GetComponentLocation(), EndPoints[Index], bHit ? FColor::Green : FColor::Red, false, Duration, 0, 0.75f);
    }
}
