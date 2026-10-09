#include "UBotImageFrame.h"

#include "Engine/Texture2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "UBotCameraSensorComponent.h"
#include "UBotPointCloudFrame.h"
#include "UBotSensorMath.h"

bool UUBotImageFrame::RequestPixelReadback(FString& ErrorMessage)
{
    if (HasCPUPixelData())
    {
        return true;
    }
    if (!SourceSensor.IsValid())
    {
        ErrorMessage = TEXT("Image frame source sensor is no longer valid.");
        return false;
    }
    return SourceSensor->ReadbackFrame(this, ErrorMessage);
}

void UUBotImageFrame::ClearCPUPixelData()
{
    PixelBytes.Reset();
    DepthValuesMeters.Reset();
}

bool UUBotImageFrame::HasCPUPixelData() const
{
    return Encoding == EUBotImageEncoding::BGRA8 ? PixelBytes.Num() == Width * Height * 4 : DepthValuesMeters.Num() == Width * Height;
}

int32 UUBotImageFrame::GetPixelDataSizeBytes() const
{
    return Encoding == EUBotImageEncoding::BGRA8 ? PixelBytes.Num() : DepthValuesMeters.Num() * static_cast<int32>(sizeof(float));
}

bool UUBotImageFrame::GetBGRAPixelByCoordinates(int32 X, int32 Y, FColor& Color) const
{
    Color = FColor::Black;
    if (Encoding != EUBotImageEncoding::BGRA8 || !IsValidPixel(X, Y) || PixelBytes.Num() != Width * Height * 4)
    {
        return false;
    }
    const int32 Offset = (Y * Width + X) * 4;
    Color = FColor(PixelBytes[Offset + 2], PixelBytes[Offset + 1], PixelBytes[Offset], PixelBytes[Offset + 3]);
    return true;
}

bool UUBotImageFrame::GetDepthMetersByCoordinates(int32 X, int32 Y, float& DepthMeters) const
{
    DepthMeters = 0.0f;
    if (Encoding != EUBotImageEncoding::Depth32FC1 || !IsValidPixel(X, Y) || DepthValuesMeters.Num() != Width * Height)
    {
        return false;
    }
    DepthMeters = DepthValuesMeters[Y * Width + X];
    return true;
}

UTexture2D* UUBotImageFrame::UpdateDepthPreview(EUBotDepthPreviewMode PreviewMode, float PreviewMinimumDepthMeters, float PreviewMaximumDepthMeters, FString& ErrorMessage)
{
    if (Encoding != EUBotImageEncoding::Depth32FC1)
    {
        ErrorMessage = TEXT("Depth preview is only available for Depth32FC1 frames.");
        return nullptr;
    }
    if (!HasCPUPixelData() && !RequestPixelReadback(ErrorMessage))
    {
        return nullptr;
    }
    if (PreviewMaximumDepthMeters <= PreviewMinimumDepthMeters)
    {
        ErrorMessage = TEXT("Depth preview range is invalid.");
        return nullptr;
    }

    TArray<uint8> PreviewBytes;
    PreviewBytes.SetNumZeroed(Width * Height * 4);
    for (int32 Index = 0; Index < DepthValuesMeters.Num(); ++Index)
    {
        const float Depth = DepthValuesMeters[Index];
        const FColor Color = UBot::SensorMath::DepthPreviewColor(Depth, PreviewMinimumDepthMeters, PreviewMaximumDepthMeters, PreviewMode);
        const int32 Offset = Index * 4;
        PreviewBytes[Offset] = Color.B;
        PreviewBytes[Offset + 1] = Color.G;
        PreviewBytes[Offset + 2] = Color.R;
        PreviewBytes[Offset + 3] = Color.A;
    }
    DepthPreviewTexture = UTexture2D::CreateTransient(Width, Height, PF_B8G8R8A8, NAME_None, PreviewBytes);
    if (DepthPreviewTexture)
    {
        DepthPreviewTexture->SRGB = true;
        DepthPreviewTexture->UpdateResource();
    }
    return DepthPreviewTexture;
}

UUBotPointCloudFrame* UUBotImageFrame::GeneratePointCloud(FString& ErrorMessage) const
{
    if (Encoding != EUBotImageEncoding::Depth32FC1 || !HasCPUPixelData())
    {
        ErrorMessage = TEXT("A CPU-readable Depth32FC1 frame is required to generate a point cloud.");
        return nullptr;
    }
    UUBotPointCloudFrame* Frame = NewObject<UUBotPointCloudFrame>(const_cast<UUBotImageFrame*>(this));
    Frame->Header = Header;
    Frame->SensorTransformWorld = SensorTransformWorld;
    Frame->TotalRayCount = Width * Height;
    Frame->ChannelCount = Height;
    Frame->HorizontalSampleCount = Width;
    Frame->HorizontalFovRadians = HorizontalFovRadians;
    Frame->VerticalFovUpRadians = VerticalFovRadians * 0.5f;
    Frame->VerticalFovDownRadians = VerticalFovRadians * 0.5f;
    Frame->ScanPeriodSeconds = 0.0f;
    // A depth camera measures no return intensity, so Intensities stays empty (HasIntensityData() is
    // false), exactly as in the legacy plugin. Points, rings and time offsets stay aligned.
    Frame->PointsMeters.Reserve(Width * Height);
    Frame->RingIndices.Reserve(Width * Height);
    Frame->TimeOffsetsSeconds.Reserve(Width * Height);
    for (int32 Y = 0; Y < Height; ++Y)
    {
        for (int32 X = 0; X < Width; ++X)
        {
            const float Depth = DepthValuesMeters[Y * Width + X];
            if (Depth <= 0.0f)
            {
                continue;
            }
            Frame->PointsMeters.Add(UBot::SensorMath::DepthPixelToPointMeters(X, Y, Depth, Fx, Fy, Cx, Cy));
            Frame->RingIndices.Add(Y);
            Frame->TimeOffsetsSeconds.Add(0.0f);
        }
    }
    Frame->HitPointCount = Frame->PointsMeters.Num();
    return Frame;
}

bool UUBotImageFrame::IsValidPixel(int32 X, int32 Y) const
{
    return X >= 0 && X < Width && Y >= 0 && Y < Height;
}
