#pragma once

#include "CoreMinimal.h"

class USceneComponent;

// Private to UBotSensor (not exported). Reads the state of the nearest provided body at or above a point
// (UBot::BodyState, UBotCore) and maps it to the point, so Pose, Odometry and IMU share one definition of
// "the motion of this sensor according to the body state source".
namespace UBot::SensorBodyMotion
{
    enum class EReadResult : uint8
    {
        /** No body state source provides the point or one of its attach parents. */
        NoSource,

        /** A body is provided, but its source has no state for it yet. */
        NoState,

        /** The motion of the point is available. */
        Ok
    };

    /**
     * Motion of a point P fixed to the provided body B, from the body state X. Mount (from the current
     * component transforms): r_B = B.Rot^-1 * (P.Loc - B.Loc), q_BS = B.Rot^-1 * P.Rot. At the state time:
     * r_W = X.Orientation * r_B, q_WS = X.Orientation * q_BS. All SI, UE world axes.
     */
    struct FPointMotion
    {
        /** X.TimestampSeconds: the uBot sim time the state belongs to. */
        double StateTimeSeconds = 0.0;

        /** X.Position + r_W. */
        FVector PositionMeters = FVector::ZeroVector;

        /** q_WS. */
        FQuat Orientation = FQuat::Identity;

        /** X.v + omega x r_W. */
        FVector LinearVelocity = FVector::ZeroVector;

        /** omega: the same for every point of the rigid body. */
        FVector AngularVelocity = FVector::ZeroVector;

        /** r_W. */
        FVector OffsetWorldMeters = FVector::ZeroVector;

        /** X.v: velocity of the body origin (for differencing). */
        FVector BodyLinearVelocity = FVector::ZeroVector;

        /** Acceleration of the body origin, as the state gives it; valid when bHasLinearAcceleration. */
        FVector BodyLinearAcceleration = FVector::ZeroVector;
        bool bHasLinearAcceleration = false;

        /** Angular acceleration of the body, as the state gives it; valid when bHasAngularAcceleration. */
        FVector BodyAngularAcceleration = FVector::ZeroVector;
        bool bHasAngularAcceleration = false;
    };

    /**
     * Reads the state of the nearest provided body at or above Point and maps it to Point. Nothing is
     * cached: a binding can come and go at any time. Out is only written when this returns Ok.
     */
    EReadResult Read(const USceneComponent& Point, FPointMotion& Out);

    /** How far in the future (s) a state may lie and still count as describing the sample time. */
    inline constexpr double FutureStateToleranceSeconds = 1.0e-6;

    /**
     * True when a state of the given age (sample time minus state time) describes the sample: not from the
     * future beyond the tolerance and not older than MaximumAgeSeconds.
     */
    inline bool IsStateAgeAcceptable(double AgeSeconds, double MaximumAgeSeconds)
    {
        return AgeSeconds >= -FutureStateToleranceSeconds && AgeSeconds <= MaximumAgeSeconds;
    }
}
