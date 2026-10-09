#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "UBotSensorTypes.h"
#include "UBotTypes.h"
#include "UBotImageFrame.generated.h"

class UTexture2D;
class UTextureRenderTarget2D;
class UUBotCameraSensorComponent;
class UUBotPointCloudFrame;

/**
 * One camera capture. The GPU image lives in RawCaptureRenderTarget; CPU pixel data is only filled
 * on demand by RequestPixelReadback (synchronous, high cost) while the frame is still the latest
 * capture of its source camera.
 */
UCLASS(BlueprintType)
class UBOTSENSOR_API UUBotImageFrame : public UObject
{
    GENERATED_BODY()

public:
    UPROPERTY(BlueprintReadOnly, Category = "Camera")
    FUBotFrameHeader Header;

    UPROPERTY(BlueprintReadOnly, Category = "Camera")
    int32 Width = 0;

    UPROPERTY(BlueprintReadOnly, Category = "Camera")
    int32 Height = 0;

    UPROPERTY(BlueprintReadOnly, Category = "Camera")
    EUBotImageEncoding Encoding = EUBotImageEncoding::BGRA8;

    UPROPERTY(BlueprintReadOnly, Category = "Camera")
    int32 RowStrideBytes = 0;

    UPROPERTY(BlueprintReadOnly, Category = "Camera")
    float HorizontalFovRadians = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Camera")
    float VerticalFovRadians = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Camera")
    float Fx = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Camera")
    float Fy = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Camera")
    float Cx = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Camera")
    float Cy = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "Camera")
    FTransform SensorTransformWorld = FTransform::Identity;

    UPROPERTY(BlueprintReadOnly, Category = "Camera|Synchronization")
    FString CaptureGroupId;

    UPROPERTY(BlueprintReadOnly, Category = "Camera|Synchronization")
    int64 CaptureSequence = 0;

    UPROPERTY(BlueprintReadOnly, Category = "Camera|Synchronization")
    double CaptureTimestampSeconds = 0.0;

    UPROPERTY(BlueprintReadOnly, Category = "Camera|Synchronization")
    bool bSynchronized = false;

    UPROPERTY(BlueprintReadOnly, Category = "Camera")
    TObjectPtr<UTextureRenderTarget2D> RawCaptureRenderTarget;

    UFUNCTION(BlueprintCallable, Category = "uBot Sensor|Camera")
    bool RequestPixelReadback(FString& ErrorMessage);

    UFUNCTION(BlueprintCallable, Category = "uBot Sensor|Camera")
    void ClearCPUPixelData();

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|Camera")
    bool HasCPUPixelData() const;

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|Camera")
    int32 GetPixelDataSizeBytes() const;

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|Camera", meta = (DisplayName = "Copy Pixel Bytes (High Cost)"))
    TArray<uint8> CopyPixelBytes() const { return PixelBytes; }

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|Camera", meta = (DisplayName = "Copy Depth Values (High Cost)"))
    TArray<float> CopyDepthValues() const { return DepthValuesMeters; }

    const TArray<uint8>& GetPixelBytes() const { return PixelBytes; }
    const TArray<float>& GetDepthValuesMeters() const { return DepthValuesMeters; }

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|Camera")
    bool GetBGRAPixelByCoordinates(int32 X, int32 Y, FColor& Color) const;

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|Camera")
    bool GetDepthMetersByCoordinates(int32 X, int32 Y, float& DepthMeters) const;

    UFUNCTION(BlueprintCallable, Category = "uBot Sensor|Depth Camera")
    UTexture2D* UpdateDepthPreview(EUBotDepthPreviewMode PreviewMode, float PreviewMinimumDepthMeters, float PreviewMaximumDepthMeters, FString& ErrorMessage);

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|Depth Camera")
    UTexture2D* GetDepthPreviewTexture() const { return DepthPreviewTexture; }

    UFUNCTION(BlueprintCallable, Category = "uBot Sensor|Depth Camera")
    UUBotPointCloudFrame* GeneratePointCloud(FString& ErrorMessage) const;

private:
    friend class UUBotCameraSensorComponent;
    friend class UUBotDepthCameraSensorComponent;
    // Lets the automation tests inject CPU pixel data without a GPU readback.
    friend struct FUBotImageFrameTestAccess;

    bool IsValidPixel(int32 X, int32 Y) const;

    UPROPERTY(Transient)
    TArray<uint8> PixelBytes;

    UPROPERTY(Transient)
    TArray<float> DepthValuesMeters;

    UPROPERTY(Transient)
    TObjectPtr<UTexture2D> DepthPreviewTexture;

    UPROPERTY(Transient)
    TWeakObjectPtr<UUBotCameraSensorComponent> SourceSensor;

    /** Value of the source camera's capture generation when this frame was captured (stale-frame check). */
    uint64 CaptureGeneration = 0;
};
