#pragma once

#include "CoreMinimal.h"
#include "Math/RandomStream.h"
#include "UBotSensorTypes.h"

/**
 * Pure math shared by the uBot sensors. Unit conversions (for example centimeters to meters) are in
 * UBot::Units (UBotUnits.h, UBotCore).
 */
namespace UBot::SensorMath
{
    UBOTSENSOR_API FVector QuaternionDeltaToAngularVelocity(const FQuat& Previous, const FQuat& Current, double DeltaSeconds);

    /** Velocity of a point fixed to a rigid body, world frame: OriginVelocity + AngularVelocity x OffsetMeters (offset = point - body origin, world axes, meters). */
    UBOTSENSOR_API FVector RigidPointVelocity(const FVector& OriginVelocity, const FVector& AngularVelocity, const FVector& OffsetMeters);

    /** Kinematic acceleration of a point fixed to a rigid body, world frame: OriginAcceleration + AngularAcceleration x Offset + AngularVelocity x (AngularVelocity x Offset). */
    UBOTSENSOR_API FVector RigidPointAcceleration(const FVector& OriginAcceleration, const FVector& AngularVelocity, const FVector& AngularAcceleration, const FVector& OffsetMeters);

    UBOTSENSOR_API float GaussianSample(FRandomStream& RandomStream, float StandardDeviation);
    UBOTSENSOR_API FVector GaussianVector(FRandomStream& RandomStream, float StandardDeviation);
    UBOTSENSOR_API TArray<double> MakeDiagonalCovariance6(const FVector& PositionOrLinearVariance, const FVector& RotationOrAngularVariance);
    UBOTSENSOR_API float LidarHorizontalAngleDegrees(int32 SampleIndex, int32 SampleCount, float HorizontalFovDegrees);
    UBOTSENSOR_API float LidarVerticalAngleDegrees(int32 RingIndex, int32 ChannelCount, float VerticalFovDownDegrees, float VerticalFovUpDegrees);
    UBOTSENSOR_API FVector LidarDirectionLocal(float HorizontalAngleDegrees, float VerticalAngleDegrees);
    UBOTSENSOR_API float CameraVerticalFovRadians(int32 Width, int32 Height, float HorizontalFovDegrees);
    UBOTSENSOR_API void CameraIntrinsics(int32 Width, int32 Height, float HorizontalFovDegrees, float& OutFx, float& OutFy, float& OutCx, float& OutCy);
    UBOTSENSOR_API FVector DepthPixelToPointMeters(int32 PixelX, int32 PixelY, float DepthMeters, float Fx, float Fy, float Cx, float Cy);
    UBOTSENSOR_API float FilterSceneDepthCentimeters(float DepthCentimeters, float MinimumDepthMeters, float MaximumDepthMeters);
    UBOTSENSOR_API FColor DepthPreviewColor(float DepthMeters, float MinimumDepthMeters, float MaximumDepthMeters, EUBotDepthPreviewMode PreviewMode);
}
