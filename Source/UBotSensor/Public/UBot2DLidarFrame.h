#pragma once

#include "CoreMinimal.h"
#include "UBotTypes.h"
#include "UBot2DLidarFrame.generated.h"

UCLASS(BlueprintType)
class UBOTSENSOR_API UUBot2DLidarFrame : public UObject
{
    GENERATED_BODY()

public:
    UPROPERTY(BlueprintReadOnly, Category = "2D LiDAR")
    FUBotFrameHeader Header;

    UPROPERTY(BlueprintReadOnly, Category = "2D LiDAR")
    FTransform SensorTransformWorld = FTransform::Identity;

    UPROPERTY(BlueprintReadOnly, Category = "2D LiDAR")
    float AngleMinRadians = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "2D LiDAR")
    float AngleMaxRadians = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "2D LiDAR")
    float AngleIncrementRadians = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "2D LiDAR")
    float RangeMinMeters = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "2D LiDAR")
    float RangeMaxMeters = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "2D LiDAR")
    float ScanPeriodSeconds = 0.0f;

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|2D LiDAR")
    int32 GetSampleCount() const { return RangesMeters.Num(); }

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|2D LiDAR")
    bool GetRangeByIndex(int32 Index, float& RangeMeters, bool& bHit) const;

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|2D LiDAR", meta = (DisplayName = "Copy All Ranges (High Cost)"))
    TArray<float> CopyAllRanges() const { return RangesMeters; }

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|2D LiDAR", meta = (DisplayName = "Copy All Hit Flags (High Cost)"))
    TArray<bool> CopyAllHitFlags() const;

    /** One return intensity per sample, 0..255 (0 for samples without a surface return). */
    UFUNCTION(BlueprintPure, Category = "uBot Sensor|2D LiDAR")
    const TArray<float>& GetIntensities() const { return Intensities; }

    const TArray<float>& GetRangesMeters() const { return RangesMeters; }
    const TArray<uint8>& GetHitFlags() const { return HitFlags; }

private:
    friend class UUBot2DLidarSensorComponent;

    UPROPERTY(Transient)
    TArray<float> RangesMeters;

    UPROPERTY(Transient)
    TArray<uint8> HitFlags;

    UPROPERTY(Transient)
    TArray<float> Intensities;
};
