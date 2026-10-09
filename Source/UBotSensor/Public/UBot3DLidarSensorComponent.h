#pragma once

#include "CoreMinimal.h"
#include "Engine/EngineTypes.h"
#include "Math/RandomStream.h"
#include "UBotSensorComponent.h"
#include "UBotTypes.h"
#include "UBot3DLidarSensorComponent.generated.h"

class AActor;
class UUBotPointCloudFrame;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FUBot3DLidarFrameEvent, UUBotPointCloudFrame*, Frame);

UCLASS(Blueprintable, ClassGroup = (uBot), meta = (BlueprintSpawnableComponent, DisplayName = "uBot 3D LiDAR Sensor"))
class UBOTSENSOR_API UUBot3DLidarSensorComponent : public UUBotSensorComponent
{
    GENERATED_BODY()

public:
    UUBot3DLidarSensorComponent();

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "3D LiDAR|Scan", meta = (ClampMin = "0.0"))
    float MinimumRangeMeters = 0.1f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "3D LiDAR|Scan", meta = (ClampMin = "0.01"))
    float MaximumRangeMeters = 100.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "3D LiDAR|Scan", meta = (ClampMin = "0.01", ClampMax = "360.0"))
    float HorizontalFovDegrees = 360.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "3D LiDAR|Scan", meta = (ClampMin = "0.0", ClampMax = "90.0"))
    float VerticalFovUpDegrees = 15.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "3D LiDAR|Scan", meta = (ClampMin = "0.0", ClampMax = "90.0"))
    float VerticalFovDownDegrees = 15.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "3D LiDAR|Scan", meta = (ClampMin = "1", ClampMax = "256", UIMax = "128"))
    int32 NumberOfChannels = 16;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "3D LiDAR|Scan", meta = (ClampMin = "1", ClampMax = "8192", UIMax = "2048"))
    int32 HorizontalSamples = 360;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "3D LiDAR|Safety", meta = (ClampMin = "1", ClampMax = "1048576"))
    int32 MaximumRaysPerScan = 65536;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "3D LiDAR|Collision")
    TEnumAsByte<ECollisionChannel> TraceChannel = ECC_Visibility;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "3D LiDAR|Collision")
    bool bTraceComplex = false;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "3D LiDAR|Collision")
    bool bIgnoreOwner = true;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "3D LiDAR|Collision")
    TArray<TObjectPtr<AActor>> IgnoredActors;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "3D LiDAR|Smoke")
    bool bEnableSmokeInterference = true;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "3D LiDAR|Smoke")
    FName SmokeVolumeActorTag = TEXT("AgentSensor.SmokeVolume");

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "3D LiDAR|Smoke", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float SmokeDensityThreshold = 0.05f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "3D LiDAR|Smoke", meta = (ClampMin = "0.01"))
    float SmokeCellRadiusScale = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "3D LiDAR|Noise", meta = (ClampMin = "0.0"))
    float RangeNoiseStdDevMeters = 0.0f;

#if WITH_EDITORONLY_DATA
    UPROPERTY(EditAnywhere, Category = "3D LiDAR|Editor Preview")
    FLinearColor RangePreviewColor = FLinearColor(0.05f, 0.65f, 1.0f, 1.0f);

    UPROPERTY(EditAnywhere, Category = "3D LiDAR|Editor Preview", meta = (ClampMin = "0.0", ClampMax = "1.0", UIMin = "0.0", UIMax = "0.5"))
    float RangePreviewOpacity = 0.12f;

    UPROPERTY(EditAnywhere, Category = "3D LiDAR|Editor Preview", meta = (ClampMin = "0.1", UIMin = "1.0", UIMax = "25.0"))
    float DistanceGridIntervalMeters = 5.0f;

    UPROPERTY(EditAnywhere, Category = "3D LiDAR|Editor Preview", meta = (ClampMin = "1.0", ClampMax = "90.0", UIMin = "5.0", UIMax = "45.0"))
    float AngularGridIntervalDegrees = 15.0f;

    UPROPERTY(EditAnywhere, Category = "3D LiDAR|Editor Preview")
    bool bShowRangeLabels = true;
#endif

    UPROPERTY(BlueprintAssignable, Category = "3D LiDAR")
    FUBot3DLidarFrameEvent On3DLidarFrameReady;

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|3D LiDAR")
    UUBotPointCloudFrame* GetLatest3DLidarFrame() const { return LatestFrame; }

    UFUNCTION(BlueprintCallable, Category = "uBot Sensor|3D LiDAR")
    bool Sample3DLidar() { return SampleSensor(); }

    virtual FName GetSensorTypeName() const override;

protected:
    virtual bool CaptureSensorData(FUBotFrameHeader& Header, FString& OutErrorMessage) override;
    virtual void OnSamplingStarted() override;

private:
    void DrawRuntimeDebug(const TArray<FVector>& EndPoints, const TArray<uint8>& HitFlags) const;

    UPROPERTY(Transient)
    TObjectPtr<UUBotPointCloudFrame> LatestFrame;

    FRandomStream RandomStream;
};
