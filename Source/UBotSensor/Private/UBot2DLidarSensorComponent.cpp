#include "UBot2DLidarSensorComponent.h"

#include "CollisionQueryParams.h"
#include "DrawDebugHelpers.h"
#include "Engine/HitResult.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "UBot2DLidarFrame.h"
#include "UBotPerceptionMedium.h"
#include "UBotSensorMedium.h"
#include "UObject/ConstructorHelpers.h"

namespace UBot2DLidarPrivate
{
    // Return strength of a surface hit: 255 when the surface faces the ray, falling to 0 at grazing
    // or back-facing incidence.
    float SurfaceIntensity(const FVector& RayDirectionWorld, const FVector& ImpactNormal)
    {
        return 255.0f * FMath::Clamp(static_cast<float>(FVector::DotProduct(-RayDirectionWorld, ImpactNormal)), 0.0f, 1.0f);
    }
}

UUBot2DLidarSensorComponent::UUBot2DLidarSensorComponent()
{
    static ConstructorHelpers::FObjectFinder<UStaticMesh> SensorMesh(TEXT("/UBotSensor/Models/SM_UBotSensor_Lidar2D.SM_UBotSensor_Lidar2D"));
    if (SensorMesh.Succeeded())
    {
        SetStaticMesh(SensorMesh.Object);
    }
    SensorId = TEXT("lidar_2d");
    FrameId = TEXT("lidar_2d");
    UpdateRateHz = 10.0f;
}

FName UUBot2DLidarSensorComponent::GetSensorTypeName() const
{
    static const FName SensorTypeName(TEXT("Lidar2D"));
    return SensorTypeName;
}

bool UUBot2DLidarSensorComponent::CaptureSensorData(FUBotFrameHeader& Header, FString& OutErrorMessage)
{
    if (!GetWorld())
    {
        OutErrorMessage = TEXT("2D LiDAR has no valid world.");
        MarkHeader(Header, EUBotSampleStatus::Error, false);
        return false;
    }
    if (StartAngleDegrees >= EndAngleDegrees || MinimumRangeMeters < 0.0f || MaximumRangeMeters <= MinimumRangeMeters)
    {
        OutErrorMessage = TEXT("2D LiDAR scan limits are invalid.");
        MarkHeader(Header, EUBotSampleStatus::Error, false);
        return false;
    }

    const int32 SampleCount = FMath::Clamp(HorizontalSamples, 2, 4096);
    const float AngleIncrementDegrees = (EndAngleDegrees - StartAngleDegrees) / static_cast<float>(SampleCount - 1);
    const FVector Origin = GetComponentLocation();
    const FTransform SensorTransform = GetComponentTransform();
    const float MaximumRangeCentimeters = MaximumRangeMeters * 100.0f;

    FCollisionQueryParams QueryParams(SCENE_QUERY_STAT(UBot2DLidar), bTraceComplex);
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
        UBot::SensorMedium::CollectMedia(*GetWorld(), MediumSettings, MediumSnapshots);
    }

    UUBot2DLidarFrame* NewFrame = NewObject<UUBot2DLidarFrame>(this);
    NewFrame->SensorTransformWorld = SensorTransform;
    NewFrame->AngleMinRadians = FMath::DegreesToRadians(StartAngleDegrees);
    NewFrame->AngleMaxRadians = FMath::DegreesToRadians(EndAngleDegrees);
    NewFrame->AngleIncrementRadians = FMath::DegreesToRadians(AngleIncrementDegrees);
    NewFrame->RangeMinMeters = MinimumRangeMeters;
    NewFrame->RangeMaxMeters = MaximumRangeMeters;
    NewFrame->ScanPeriodSeconds = 1.0f / FMath::Clamp(UpdateRateHz, 0.1f, 240.0f);
    NewFrame->RangesMeters.Reserve(SampleCount);
    NewFrame->HitFlags.Reserve(SampleCount);
    NewFrame->Intensities.Reserve(SampleCount);

    TArray<FVector> DebugEndPoints;
    DebugEndPoints.Reserve(SampleCount);
    for (int32 Index = 0; Index < SampleCount; ++Index)
    {
        const float AngleDegrees = StartAngleDegrees + AngleIncrementDegrees * Index;
        const float AngleRadians = FMath::DegreesToRadians(AngleDegrees);
        const FVector LocalDirection(FMath::Cos(AngleRadians), FMath::Sin(AngleRadians), 0.0f);
        const FVector WorldDirection = SensorTransform.TransformVectorNoScale(LocalDirection).GetSafeNormal();
        const FVector TraceEnd = Origin + WorldDirection * MaximumRangeCentimeters;

        FHitResult Hit;
        const bool bHit = GetWorld()->LineTraceSingleByChannel(Hit, Origin, TraceEnd, TraceChannel, QueryParams);
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
            RangeMeters += GaussianNoise(RangeNoiseStdDevMeters);
        }
        RangeMeters = FMath::Clamp(RangeMeters, MinimumRangeMeters, MaximumRangeMeters);
        // Only a surface return carries an intensity; misses and returns truncated by a medium report 0.
        const float Intensity = (bHit && !bSmokeHit) ? UBot2DLidarPrivate::SurfaceIntensity(WorldDirection, Hit.ImpactNormal) : 0.0f;
        NewFrame->RangesMeters.Add(RangeMeters);
        NewFrame->HitFlags.Add((bHit || bSmokeHit) ? 1 : 0);
        NewFrame->Intensities.Add(Intensity);
        DebugEndPoints.Add(Origin + WorldDirection * RangeMeters * 100.0f);
    }

    MarkHeader(Header, EUBotSampleStatus::Valid, true);
    NewFrame->Header = Header;
    LatestFrame = NewFrame;
    On2DLidarFrameReady.Broadcast(NewFrame);
    if (bDebugVisualization)
    {
        DrawRuntimeDebug(DebugEndPoints, NewFrame->HitFlags);
    }
    return true;
}

void UUBot2DLidarSensorComponent::OnSamplingStarted()
{
    RandomStream.Initialize(RandomSeed);
}

float UUBot2DLidarSensorComponent::GaussianNoise(float StandardDeviation)
{
    const float UniformA = FMath::Max(RandomStream.GetFraction(), UE_SMALL_NUMBER);
    const float UniformB = RandomStream.GetFraction();
    return StandardDeviation * FMath::Sqrt(-2.0f * FMath::Loge(UniformA)) * FMath::Cos(2.0f * PI * UniformB);
}

void UUBot2DLidarSensorComponent::DrawRuntimeDebug(const TArray<FVector>& EndPoints, const TArray<uint8>& HitFlags) const
{
    if (!GetWorld() || EndPoints.IsEmpty())
    {
        return;
    }
    const int32 Step = FMath::Max(1, EndPoints.Num() / 32);
    const float Duration = 1.0f / FMath::Clamp(UpdateRateHz, 0.1f, 240.0f);
    for (int32 Index = 0; Index < EndPoints.Num(); Index += Step)
    {
        const bool bHit = HitFlags.IsValidIndex(Index) && HitFlags[Index] != 0;
        DrawDebugLine(GetWorld(), GetComponentLocation(), EndPoints[Index], bHit ? FColor::Green : FColor::Red, false, Duration, 0, 1.0f);
    }
}
