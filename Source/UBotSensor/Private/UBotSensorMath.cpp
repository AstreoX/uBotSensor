#include "UBotSensorMath.h"

namespace UBot::SensorMath
{
    FVector QuaternionDeltaToAngularVelocity(const FQuat& Previous, const FQuat& Current, double DeltaSeconds)
    {
        if (DeltaSeconds <= UE_SMALL_NUMBER)
        {
            return FVector::ZeroVector;
        }

        FQuat Delta = Current * Previous.Inverse();
        Delta.Normalize();
        if (Delta.W < 0.0)
        {
            Delta = FQuat(-Delta.X, -Delta.Y, -Delta.Z, -Delta.W);
        }

        FVector Axis = FVector::ForwardVector;
        double Angle = 0.0;
        Delta.ToAxisAndAngle(Axis, Angle);
        return Axis.GetSafeNormal() * (Angle / DeltaSeconds);
    }

    float GaussianSample(FRandomStream& RandomStream, float StandardDeviation)
    {
        if (StandardDeviation <= 0.0f)
        {
            return 0.0f;
        }
        const float UniformA = FMath::Max(RandomStream.GetFraction(), UE_SMALL_NUMBER);
        const float UniformB = RandomStream.GetFraction();
        return StandardDeviation * FMath::Sqrt(-2.0f * FMath::Loge(UniformA)) * FMath::Cos(2.0f * UE_PI * UniformB);
    }

    FVector GaussianVector(FRandomStream& RandomStream, float StandardDeviation)
    {
        return FVector(GaussianSample(RandomStream, StandardDeviation), GaussianSample(RandomStream, StandardDeviation), GaussianSample(RandomStream, StandardDeviation));
    }

    TArray<double> MakeDiagonalCovariance6(const FVector& PositionOrLinearVariance, const FVector& RotationOrAngularVariance)
    {
        TArray<double> Result;
        Result.Init(0.0, 36);
        Result[0] = PositionOrLinearVariance.X;
        Result[7] = PositionOrLinearVariance.Y;
        Result[14] = PositionOrLinearVariance.Z;
        Result[21] = RotationOrAngularVariance.X;
        Result[28] = RotationOrAngularVariance.Y;
        Result[35] = RotationOrAngularVariance.Z;
        return Result;
    }

    float LidarHorizontalAngleDegrees(int32 SampleIndex, int32 SampleCount, float HorizontalFovDegrees)
    {
        if (SampleCount <= 1)
        {
            return 0.0f;
        }
        const float ClampedFov = FMath::Clamp(HorizontalFovDegrees, 0.01f, 360.0f);
        const bool bFullRevolution = FMath::IsNearlyEqual(ClampedFov, 360.0f, 0.001f);
        const float Increment = ClampedFov / static_cast<float>(bFullRevolution ? SampleCount : SampleCount - 1);
        return -0.5f * ClampedFov + Increment * FMath::Clamp(SampleIndex, 0, SampleCount - 1);
    }

    float LidarVerticalAngleDegrees(int32 RingIndex, int32 ChannelCount, float VerticalFovDownDegrees, float VerticalFovUpDegrees)
    {
        if (ChannelCount <= 1)
        {
            return 0.0f;
        }
        const float Alpha = static_cast<float>(FMath::Clamp(RingIndex, 0, ChannelCount - 1)) / static_cast<float>(ChannelCount - 1);
        return FMath::Lerp(-FMath::Max(0.0f, VerticalFovDownDegrees), FMath::Max(0.0f, VerticalFovUpDegrees), Alpha);
    }

    FVector LidarDirectionLocal(float HorizontalAngleDegrees, float VerticalAngleDegrees)
    {
        const float Azimuth = FMath::DegreesToRadians(HorizontalAngleDegrees);
        const float Elevation = FMath::DegreesToRadians(VerticalAngleDegrees);
        const float CosElevation = FMath::Cos(Elevation);
        return FVector(CosElevation * FMath::Cos(Azimuth), CosElevation * FMath::Sin(Azimuth), FMath::Sin(Elevation)).GetSafeNormal();
    }

    float CameraVerticalFovRadians(int32 Width, int32 Height, float HorizontalFovDegrees)
    {
        const float SafeWidth = static_cast<float>(FMath::Max(1, Width));
        const float SafeHeight = static_cast<float>(FMath::Max(1, Height));
        const float HalfHorizontalRadians = 0.5f * FMath::DegreesToRadians(FMath::Clamp(HorizontalFovDegrees, 1.0f, 170.0f));
        return 2.0f * FMath::Atan(FMath::Tan(HalfHorizontalRadians) * SafeHeight / SafeWidth);
    }

    void CameraIntrinsics(int32 Width, int32 Height, float HorizontalFovDegrees, float& OutFx, float& OutFy, float& OutCx, float& OutCy)
    {
        const float SafeWidth = static_cast<float>(FMath::Max(1, Width));
        const float SafeHeight = static_cast<float>(FMath::Max(1, Height));
        const float HalfHorizontalRadians = 0.5f * FMath::DegreesToRadians(FMath::Clamp(HorizontalFovDegrees, 1.0f, 170.0f));
        OutFx = SafeWidth / (2.0f * FMath::Tan(HalfHorizontalRadians));
        OutFy = OutFx;
        OutCx = (SafeWidth - 1.0f) * 0.5f;
        OutCy = (SafeHeight - 1.0f) * 0.5f;
    }

    FVector DepthPixelToPointMeters(int32 PixelX, int32 PixelY, float DepthMeters, float Fx, float Fy, float Cx, float Cy)
    {
        if (DepthMeters <= 0.0f || Fx <= UE_SMALL_NUMBER || Fy <= UE_SMALL_NUMBER)
        {
            return FVector::ZeroVector;
        }
        return FVector(
            DepthMeters,
            (static_cast<float>(PixelX) - Cx) * DepthMeters / Fx,
            -(static_cast<float>(PixelY) - Cy) * DepthMeters / Fy);
    }

    float FilterSceneDepthCentimeters(float DepthCentimeters, float MinimumDepthMeters, float MaximumDepthMeters)
    {
        const float DepthMeters = DepthCentimeters * 0.01f;
        return FMath::IsFinite(DepthMeters) && DepthMeters >= MinimumDepthMeters && DepthMeters <= MaximumDepthMeters ? DepthMeters : 0.0f;
    }

    FColor DepthPreviewColor(float DepthMeters, float MinimumDepthMeters, float MaximumDepthMeters, EUBotDepthPreviewMode PreviewMode)
    {
        if (DepthMeters <= 0.0f || MaximumDepthMeters <= MinimumDepthMeters)
        {
            return FColor::Black;
        }
        const float Alpha = FMath::Clamp((DepthMeters - MinimumDepthMeters) / (MaximumDepthMeters - MinimumDepthMeters), 0.0f, 1.0f);
        if (PreviewMode == EUBotDepthPreviewMode::Grayscale)
        {
            const uint8 Value = static_cast<uint8>(FMath::RoundToInt((1.0f - Alpha) * 255.0f));
            return FColor(Value, Value, Value, 255);
        }
        return FLinearColor::MakeFromHSV8(static_cast<uint8>(Alpha * 170.0f), 255, 255).ToFColor(false);
    }
}
