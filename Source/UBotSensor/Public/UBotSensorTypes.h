#pragma once

#include "CoreMinimal.h"
// The frame header (FUBotFrameHeader) and the sample status (EUBotSampleStatus) shared by every
// sensor live in UBotCore. (A plain comment is dropped by UHT in front of an #include; in front of
// the UENUM it would become the enum's tooltip, which the docs export.)
#include "UBotTypes.h"
#include "UBotSensorTypes.generated.h"

UENUM(BlueprintType)
enum class EUBotPoseReferenceFrame : uint8
{
    World,
    Parent,
    CustomComponent
};

UENUM(BlueprintType)
enum class EUBotImageEncoding : uint8
{
    BGRA8,
    Depth32FC1
};

UENUM(BlueprintType)
enum class EUBotDepthPreviewMode : uint8
{
    Grayscale,
    PseudoColor
};

USTRUCT(BlueprintType)
struct UBOTSENSOR_API FUBotPoseSensorData
{
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly, Category = "uBot Sensor")
    FUBotFrameHeader Header;

    UPROPERTY(BlueprintReadOnly, Category = "uBot Sensor")
    FString ReferenceFrameId;

    UPROPERTY(BlueprintReadOnly, Category = "uBot Sensor")
    FVector PositionMeters = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "uBot Sensor")
    FQuat OrientationQuaternion = FQuat::Identity;

    UPROPERTY(BlueprintReadOnly, Category = "uBot Sensor")
    FVector LinearVelocityMetersPerSecond = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "uBot Sensor")
    FVector AngularVelocityRadPerSecond = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "uBot Sensor")
    bool bHasVelocity = false;
};

USTRUCT(BlueprintType)
struct UBOTSENSOR_API FUBotImuSensorData
{
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly, Category = "uBot Sensor")
    FUBotFrameHeader Header;

    UPROPERTY(BlueprintReadOnly, Category = "uBot Sensor")
    FQuat OrientationQuaternion = FQuat::Identity;

    UPROPERTY(BlueprintReadOnly, Category = "uBot Sensor")
    FVector AngularVelocityRadPerSecond = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "uBot Sensor")
    FVector LinearAccelerationMetersPerSecondSquared = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "uBot Sensor")
    FVector OrientationVariance = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "uBot Sensor")
    FVector AngularVelocityVariance = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "uBot Sensor")
    FVector LinearAccelerationVariance = FVector::ZeroVector;
};

USTRUCT(BlueprintType)
struct UBOTSENSOR_API FUBotOdometrySensorData
{
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly, Category = "uBot Sensor")
    FUBotFrameHeader Header;

    UPROPERTY(BlueprintReadOnly, Category = "uBot Sensor")
    FString OdometryFrameId = TEXT("odom");

    UPROPERTY(BlueprintReadOnly, Category = "uBot Sensor")
    FString ChildFrameId = TEXT("base_link");

    UPROPERTY(BlueprintReadOnly, Category = "uBot Sensor")
    FVector PositionMeters = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "uBot Sensor")
    FQuat OrientationQuaternion = FQuat::Identity;

    UPROPERTY(BlueprintReadOnly, Category = "uBot Sensor")
    FVector LinearVelocityLocalMetersPerSecond = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "uBot Sensor")
    FVector AngularVelocityLocalRadPerSecond = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "uBot Sensor")
    TArray<double> PoseCovariance;

    UPROPERTY(BlueprintReadOnly, Category = "uBot Sensor")
    TArray<double> TwistCovariance;

    UPROPERTY(BlueprintReadOnly, Category = "uBot Sensor")
    bool bHasVelocity = false;
};
