#include "UBot3DLidarSensorComponent.h"

#include "CollisionQueryParams.h"
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

namespace UBot3DLidarPrivate
{
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

    TArray<FVector> DebugEndPoints;
    TArray<uint8> DebugHitFlags;
    const int32 DebugStride = FMath::Max(1, FMath::DivideAndRoundUp(TotalRays, 128));
    for (int32 RingIndex = 0; RingIndex < NumberOfChannels; ++RingIndex)
    {
        const float VerticalAngle = UBot::SensorMath::LidarVerticalAngleDegrees(RingIndex, NumberOfChannels, VerticalFovDownDegrees, VerticalFovUpDegrees);
        for (int32 HorizontalIndex = 0; HorizontalIndex < HorizontalSamples; ++HorizontalIndex)
        {
            const int32 RayIndex = RingIndex * HorizontalSamples + HorizontalIndex;
            const float HorizontalAngle = UBot::SensorMath::LidarHorizontalAngleDegrees(HorizontalIndex, HorizontalSamples, HorizontalFovDegrees);
            const FVector LocalDirection = UBot::SensorMath::LidarDirectionLocal(HorizontalAngle, VerticalAngle);
            const FVector WorldDirection = SensorTransform.TransformVectorNoScale(LocalDirection).GetSafeNormal();
            const FVector TraceEnd = Origin + WorldDirection * MaximumRangeCentimeters;
            FHitResult Hit;
            const bool bHit = World->LineTraceSingleByChannel(Hit, Origin, TraceEnd, TraceChannel, QueryParams);
            float RangeMeters = bHit ? Hit.Distance * 0.01f : MaximumRangeMeters;
            float SmokeDistanceCentimeters = 0.0f;
            float SmokeDensity = 0.0f;
            const bool bSmokeHit = UBot::Medium::FindFirstIntersection(
                Origin,
                Origin + WorldDirection * (bHit ? Hit.Distance : MaximumRangeCentimeters),
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
                const float Intensity = (bHit && !bSmokeHit) ? UBot3DLidarPrivate::SurfaceIntensity(WorldDirection, Hit.ImpactNormal) : 0.0f;
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
    LatestFrame = NewFrame;
    On3DLidarFrameReady.Broadcast(NewFrame);
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
