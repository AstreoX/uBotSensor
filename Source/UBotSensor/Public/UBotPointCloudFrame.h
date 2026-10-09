#pragma once

#include "CoreMinimal.h"
#include "UBotTypes.h"
#include "UBotPointCloudFrame.generated.h"

UCLASS(BlueprintType)
class UBOTSENSOR_API UUBotPointCloudFrame : public UObject
{
    GENERATED_BODY()

public:
    UPROPERTY(BlueprintReadOnly, Category = "3D LiDAR")
    FUBotFrameHeader Header;

    UPROPERTY(BlueprintReadOnly, Category = "3D LiDAR")
    FTransform SensorTransformWorld = FTransform::Identity;

    UPROPERTY(BlueprintReadOnly, Category = "3D LiDAR")
    int32 TotalRayCount = 0;

    UPROPERTY(BlueprintReadOnly, Category = "3D LiDAR")
    int32 HitPointCount = 0;

    UPROPERTY(BlueprintReadOnly, Category = "3D LiDAR")
    int32 ChannelCount = 0;

    UPROPERTY(BlueprintReadOnly, Category = "3D LiDAR")
    int32 HorizontalSampleCount = 0;

    UPROPERTY(BlueprintReadOnly, Category = "3D LiDAR")
    float HorizontalFovRadians = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "3D LiDAR")
    float VerticalFovUpRadians = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "3D LiDAR")
    float VerticalFovDownRadians = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "3D LiDAR")
    float MinimumRangeMeters = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "3D LiDAR")
    float MaximumRangeMeters = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "3D LiDAR")
    float ScanPeriodSeconds = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "3D LiDAR")
    float CaptureDurationMilliseconds = 0.0f;

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|3D LiDAR")
    int32 GetPointCount() const { return PointsMeters.Num(); }

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|3D LiDAR")
    int32 GetTotalRayCount() const { return TotalRayCount; }

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|3D LiDAR")
    bool GetPointByIndex(int32 Index, FVector& PointMeters, int32& RingIndex, float& TimeOffsetSeconds, float& Intensity, bool& bHasIntensity) const;

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|3D LiDAR", meta = (DisplayName = "Copy All Points (High Cost)"))
    TArray<FVector> CopyAllPoints() const { return PointsMeters; }

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|3D LiDAR", meta = (DisplayName = "Copy All Ring Indices (High Cost)"))
    TArray<int32> CopyAllRingIndices() const { return RingIndices; }

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|3D LiDAR", meta = (DisplayName = "Copy All Time Offsets (High Cost)"))
    TArray<float> CopyAllTimeOffsets() const { return TimeOffsetsSeconds; }

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|3D LiDAR", meta = (DisplayName = "Copy All Intensities (High Cost)"))
    TArray<float> CopyAllIntensities() const { return Intensities; }

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|3D LiDAR")
    bool HasIntensityData() const { return Intensities.Num() == PointsMeters.Num() && !PointsMeters.IsEmpty(); }

    /**
     * One return intensity per point (0..255), aligned with GetPointsMeters(). Empty for clouds that
     * carry no intensity, such as depth camera clouds; check HasIntensityData().
     */
    UFUNCTION(BlueprintPure, Category = "uBot Sensor|3D LiDAR")
    const TArray<float>& GetIntensities() const { return Intensities; }

    const TArray<FVector>& GetPointsMeters() const { return PointsMeters; }
    const TArray<int32>& GetRingIndices() const { return RingIndices; }
    const TArray<float>& GetTimeOffsetsSeconds() const { return TimeOffsetsSeconds; }

private:
    friend class UUBot3DLidarSensorComponent;
    friend class UUBotImageFrame;

    UPROPERTY(Transient)
    TArray<FVector> PointsMeters;

    UPROPERTY(Transient)
    TArray<int32> RingIndices;

    UPROPERTY(Transient)
    TArray<float> TimeOffsetsSeconds;

    UPROPERTY(Transient)
    TArray<float> Intensities;
};
