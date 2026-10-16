#include "UBotSensorBodyMotion.h"

#include "Components/SceneComponent.h"
#include "UBotBodyState.h"
#include "UBotSensorMath.h"
#include "UBotUnits.h"

namespace UBot::SensorBodyMotion
{
    EReadResult Read(const USceneComponent& Point, FPointMotion& Out)
    {
        FUBotBodyState State;
        const USceneComponent* Body = nullptr;
        if (!UBot::BodyState::GetNearestBodyState(&Point, State, &Body))
        {
            // The body is reported also when it has no state yet, and is null when nothing provides one.
            return Body != nullptr ? EReadResult::NoState : EReadResult::NoSource;
        }

        // Mount of the point on the body, from the current component transforms.
        const FQuat BodyRotation = Body->GetComponentQuat().GetNormalized();
        const FQuat PointRotation = Point.GetComponentQuat().GetNormalized();
        const FVector OffsetBodyMeters = UBot::Units::CentimetersToMeters(BodyRotation.UnrotateVector(Point.GetComponentLocation() - Body->GetComponentLocation()));
        const FQuat MountRotation = (BodyRotation.Inverse() * PointRotation).GetNormalized();

        // The mount carried along by the state.
        const FQuat BodyOrientation = State.Orientation.GetNormalized();
        const FVector OffsetWorldMeters = BodyOrientation.RotateVector(OffsetBodyMeters);

        Out.StateTimeSeconds = State.TimestampSeconds;
        Out.PositionMeters = State.PositionMeters + OffsetWorldMeters;
        Out.Orientation = (BodyOrientation * MountRotation).GetNormalized();
        Out.LinearVelocity = UBot::SensorMath::RigidPointVelocity(State.LinearVelocityMetersPerSecond, State.AngularVelocityRadiansPerSecond, OffsetWorldMeters);
        Out.AngularVelocity = State.AngularVelocityRadiansPerSecond;
        Out.OffsetWorldMeters = OffsetWorldMeters;
        Out.BodyLinearVelocity = State.LinearVelocityMetersPerSecond;
        Out.BodyLinearAcceleration = State.LinearAccelerationMetersPerSecondSquared;
        Out.bHasLinearAcceleration = State.bHasLinearAcceleration;
        Out.BodyAngularAcceleration = State.AngularAccelerationRadiansPerSecondSquared;
        Out.bHasAngularAcceleration = State.bHasAngularAcceleration;
        return EReadResult::Ok;
    }
}
