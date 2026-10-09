#pragma once

#include "CoreMinimal.h"
#include "Engine/EngineTypes.h"
#include "Math/RandomStream.h"
#include "UBotSensorComponent.h"
#include "UBotTypes.h"
#include "UBot2DLidarSensorComponent.generated.h"

class AActor;
class UUBot2DLidarFrame;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FUBot2DLidarFrameEvent, UUBot2DLidarFrame*, Frame);

UCLASS(Blueprintable, ClassGroup = (uBot), meta = (BlueprintSpawnableComponent, DisplayName = "uBot 2D LiDAR Sensor"))
class UBOTSENSOR_API UUBot2DLidarSensorComponent : public UUBotSensorComponent
{
    GENERATED_BODY()

public:
    UUBot2DLidarSensorComponent();

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "2D LiDAR|Scan", meta = (ClampMin = "0.0"))
    float MinimumRangeMeters = 0.05f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "2D LiDAR|Scan", meta = (ClampMin = "0.01"))
    float MaximumRangeMeters = 30.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "2D LiDAR|Scan", meta = (ClampMin = "-360.0", ClampMax = "360.0"))
    float StartAngleDegrees = -135.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "2D LiDAR|Scan", meta = (ClampMin = "-360.0", ClampMax = "360.0"))
    float EndAngleDegrees = 135.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "2D LiDAR|Scan", meta = (ClampMin = "2", ClampMax = "4096", UIMin = "2", UIMax = "1080"))
    int32 HorizontalSamples = 541;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "2D LiDAR|Collision")
    TEnumAsByte<ECollisionChannel> TraceChannel = ECC_Visibility;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "2D LiDAR|Collision")
    bool bTraceComplex = false;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "2D LiDAR|Collision")
    bool bIgnoreOwner = true;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "2D LiDAR|Collision")
    TArray<TObjectPtr<AActor>> IgnoredActors;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "2D LiDAR|Smoke")
    bool bEnableSmokeInterference = true;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "2D LiDAR|Smoke")
    FName SmokeVolumeActorTag = TEXT("AgentSensor.SmokeVolume");

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "2D LiDAR|Smoke", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float SmokeDensityThreshold = 0.05f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "2D LiDAR|Smoke", meta = (ClampMin = "0.01"))
    float SmokeCellRadiusScale = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "2D LiDAR|Noise", meta = (ClampMin = "0.0"))
    float RangeNoiseStdDevMeters = 0.0f;

#if WITH_EDITORONLY_DATA
    UPROPERTY(EditAnywhere, Category = "2D LiDAR|Editor Preview")
    FLinearColor RangePreviewColor = FLinearColor(0.05f, 0.65f, 1.0f, 1.0f);

    UPROPERTY(EditAnywhere, Category = "2D LiDAR|Editor Preview", meta = (ClampMin = "0.0", ClampMax = "1.0", UIMin = "0.0", UIMax = "0.5"))
    float RangePreviewOpacity = 0.12f;

    UPROPERTY(EditAnywhere, Category = "2D LiDAR|Editor Preview", meta = (ClampMin = "0.1", UIMin = "1.0", UIMax = "25.0"))
    float DistanceGridIntervalMeters = 5.0f;

    UPROPERTY(EditAnywhere, Category = "2D LiDAR|Editor Preview", meta = (ClampMin = "1.0", ClampMax = "90.0", UIMin = "5.0", UIMax = "45.0"))
    float AngularGridIntervalDegrees = 15.0f;

    UPROPERTY(EditAnywhere, Category = "2D LiDAR|Editor Preview")
    bool bShowRangeLabels = true;
#endif

    UPROPERTY(BlueprintAssignable, Category = "2D LiDAR")
    FUBot2DLidarFrameEvent On2DLidarFrameReady;

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|2D LiDAR")
    UUBot2DLidarFrame* GetLatest2DLidarFrame() const { return LatestFrame; }

    UFUNCTION(BlueprintCallable, Category = "uBot Sensor|2D LiDAR")
    bool Sample2DLidar() { return SampleSensor(); }

    virtual FName GetSensorTypeName() const override;

protected:
    virtual bool CaptureSensorData(FUBotFrameHeader& Header, FString& OutErrorMessage) override;
    virtual void OnSamplingStarted() override;

private:
    float GaussianNoise(float StandardDeviation);
    void DrawRuntimeDebug(const TArray<FVector>& EndPoints, const TArray<uint8>& HitFlags) const;

    UPROPERTY(Transient)
    TObjectPtr<UUBot2DLidarFrame> LatestFrame;

    FRandomStream RandomStream;
};
