#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "UBotSensorMath.h"
#include "UBotSensorTypes.h"
#include "UBotUnits.h"

// Legacy AgentSensorCore.Math.UnitConversion covered AgentSensorMath::WorldCentimetersToMeters, which
// UBot::Units::CentimetersToMeters replaces. The same values now pin the Core helper as used by the
// pose, IMU and odometry sensors (UE transform centimeters -> sensor meters).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorMathUnitConversionTest, "UBotSensor.Math.UnitConversion", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorMathUnitConversionTest::RunTest(const FString& Parameters)
{
    const FVector Meters = UBot::Units::CentimetersToMeters(FVector(100.0, -250.0, 50.0));
    TestEqual(TEXT("X centimeters convert to meters"), Meters.X, 1.0);
    TestEqual(TEXT("Y centimeters convert to meters"), Meters.Y, -2.5);
    TestEqual(TEXT("Z centimeters convert to meters"), Meters.Z, 0.5);
    TestEqual(TEXT("Scalar centimeters convert to meters"), UBot::Units::CentimetersToMeters(-981.0), -9.81, 1.e-12);

    // Gravity as the IMU derives it: UWorld::GetGravityZ() is cm/s^2 along +Z.
    const FVector Gravity = UBot::Units::CentimetersToMeters(-980.665f * FVector::UpVector);
    TestTrue(TEXT("Gravity converts to m/s^2"), Gravity.Equals(FVector(0.0, 0.0, -9.80665), 1.e-5));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorMathAngularVelocityTest, "UBotSensor.Math.AngularVelocity", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorMathAngularVelocityTest::RunTest(const FString& Parameters)
{
    const FQuat Previous = FQuat::Identity;
    const FQuat Current(FVector::UpVector, UE_PI * 0.5);
    const FVector AngularVelocity = UBot::SensorMath::QuaternionDeltaToAngularVelocity(Previous, Current, 0.5);
    TestTrue(TEXT("Angular velocity is around PI radians per second"), FMath::IsNearlyEqual(AngularVelocity.Z, UE_PI, 0.001));
    TestTrue(TEXT("Angular velocity has no X component"), FMath::IsNearlyZero(AngularVelocity.X, 0.001));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorMathQuaternionDeltaSmallAnglesTest, "UBotSensor.Math.QuaternionDeltaSmallAngles", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorMathQuaternionDeltaSmallAnglesTest::RunTest(const FString& Parameters)
{
    // A turn of less than 2e-4 rad has a quaternion vector part below 1e-4, where FQuat::ToAxisAndAngle falls back to the X
    // axis: the rate came out about X whatever the axis of the turn. Rates from far below that to far above it, about axes
    // that are not X, on a tilted attitude, must come out about their own axis. The function returns the rate in the world
    // frame, so the turn is applied in the world frame: Current = FQuat(Axis, Rate * DeltaSeconds) * Previous, and the
    // expected rate is exact up to round-off. Previous has W > 0 (angle 0.7) and W < 0 (angle 3.5); either input is also
    // passed with its sign flipped, and the result must not depend on it.
    //
    // Tolerance, relative to the rate. The products that build Current and recover the delta have components of at most 1,
    // each with a round-off of a few 1e-16, so the vector part of the recovered delta is off by at most about 4e-15 and the
    // rotation vector (twice the vector part for small angles) by 1e-14 rad, i.e. 1e-12 rad/s at DeltaSeconds = 0.01. At the
    // smallest rate, 1e-4, that is 1e-8 relative; the constructor's sine and cosine add far less at these angles. 1e-6 leaves
    // a margin of about 100 and is still far below the old error (the full rate about the wrong axis, relative error 1).
    // DeltaSeconds = 0.01 puts the old 2e-4 rad limit at 0.02 rad/s (rates 0.0199 and 0.0201 straddle it); 0.1 reaches
    // turns of 0.3 rad, where the vector part is far from small.
    const FVector Axes[] = { FVector::ZAxisVector, FVector::YAxisVector, FVector(0.0, 1.0, 1.0).GetSafeNormal(), FVector(-1.0, 2.0, -0.5).GetSafeNormal() };
    const double Angles[] = { 0.7, 3.5 };
    const double Steps[] = { 0.01, 0.1 };
    const double Rates[] = { 1.0e-4, 1.0e-3, 0.01, 0.0199, 0.0201, 0.1, 0.5, 3.0 };
    for (const double PreviousAngle : Angles)
    {
        const FQuat Previous = FQuat(FVector(1.0, 2.0, 3.0).GetSafeNormal(), PreviousAngle);
        TestTrue(TEXT("The two attitudes cover both signs of W"), PreviousAngle < 1.0 ? Previous.W > 0.0 : Previous.W < 0.0);
        for (const FVector& Axis : Axes)
        {
            for (const double DeltaSeconds : Steps)
            {
                for (const double Rate : Rates)
                {
                    const FVector Expected = Axis * Rate;
                    const FQuat Current = FQuat(Axis, Rate * DeltaSeconds) * Previous;
                    for (const double PreviousSign : { 1.0, -1.0 })
                    {
                        for (const double CurrentSign : { 1.0, -1.0 })
                        {
                            const FQuat PreviousInput(PreviousSign * Previous.X, PreviousSign * Previous.Y, PreviousSign * Previous.Z, PreviousSign * Previous.W);
                            const FQuat CurrentInput(CurrentSign * Current.X, CurrentSign * Current.Y, CurrentSign * Current.Z, CurrentSign * Current.W);
                            const FVector Recovered = UBot::SensorMath::QuaternionDeltaToAngularVelocity(PreviousInput, CurrentInput, DeltaSeconds);
                            TestTrue(FString::Printf(TEXT("Rate %g about %s over %g s (previous angle %g, signs %g, %g) is recovered (got %s)"), Rate, *Axis.ToString(), DeltaSeconds, PreviousAngle, PreviousSign, CurrentSign, *Recovered.ToString()),
                                (Recovered - Expected).Size() <= 1.0e-6 * Rate);
                        }
                    }
                }
            }
        }
    }

    // No rotation at all is no rate (also when one of the two quaternions has the opposite sign), and no time is no rate.
    // The product Tilted * Tilted^-1 is the identity only up to round-off (a vector part of about 1e-17, the rounding error
    // of a product that a fused multiply-add leaves behind), so zero is checked against the round-off bound: at most a few
    // 1e-16 per component, 2 * 4e-16 / 0.01 = 8e-14 rad/s. 1e-12 rad/s is above that and still 8 orders of magnitude below
    // the smallest rate above.
    const FQuat Tilted = FQuat(FVector(1.0, 2.0, 3.0).GetSafeNormal(), 0.7);
    const FQuat TiltedOpposite(-Tilted.X, -Tilted.Y, -Tilted.Z, -Tilted.W);
    TestTrue(TEXT("No rotation gives no rate"), UBot::SensorMath::QuaternionDeltaToAngularVelocity(Tilted, Tilted, 0.01).IsNearlyZero(1.0e-12));
    TestTrue(TEXT("No rotation gives no rate with opposite signs"), UBot::SensorMath::QuaternionDeltaToAngularVelocity(Tilted, TiltedOpposite, 0.01).IsNearlyZero(1.0e-12));
    TestTrue(TEXT("No time gives exactly zero"), UBot::SensorMath::QuaternionDeltaToAngularVelocity(Tilted, FQuat(FVector::ZAxisVector, 0.5) * Tilted, 0.0) == FVector::ZeroVector);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorMathRigidPointKinematicsTest, "UBotSensor.Math.RigidPointKinematics", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorMathRigidPointKinematicsTest::RunTest(const FString& Parameters)
{
    // Directions and signs. UE yaws right: a positive FQuat angle about +Z turns +X toward +Y.
    TestTrue(TEXT("A positive rotation about +Z turns +X toward +Y"), FQuat(FVector::UpVector, 0.1).RotateVector(FVector::ForwardVector).Y > 0.0);

    const FVector Omega(0.0, 0.0, 2.0);
    const FVector Alpha(0.0, 0.0, 4.0);
    const FVector Offset(1.5, 0.0, 0.0);
    TestTrue(TEXT("Omega +Z with the point on +X moves the point toward +Y (w x r)"),
        UBot::SensorMath::RigidPointVelocity(FVector::ZeroVector, Omega, Offset).Equals(FVector(0.0, 3.0, 0.0), 1.e-12));
    TestTrue(TEXT("The velocity of the origin is added"),
        UBot::SensorMath::RigidPointVelocity(FVector(1.0, 2.0, 3.0), Omega, Offset).Equals(FVector(1.0, 5.0, 3.0), 1.e-12));
    TestTrue(TEXT("Centripetal acceleration is -w^2 r"),
        UBot::SensorMath::RigidPointAcceleration(FVector::ZeroVector, Omega, FVector::ZeroVector, Offset).Equals(FVector(-6.0, 0.0, 0.0), 1.e-12));
    TestTrue(TEXT("Tangential acceleration is alpha x r"),
        UBot::SensorMath::RigidPointAcceleration(FVector::ZeroVector, FVector::ZeroVector, Alpha, Offset).Equals(FVector(0.0, 6.0, 0.0), 1.e-12));
    TestTrue(TEXT("All three terms add up to the acceleration of the origin plus the lever arm terms"),
        UBot::SensorMath::RigidPointAcceleration(FVector(0.5, -1.0, 2.0), Omega, Alpha, Offset).Equals(FVector(-5.5, 5.0, 2.0), 1.e-12));
    TestTrue(TEXT("No offset: the point moves with the origin"),
        UBot::SensorMath::RigidPointAcceleration(FVector(0.5, -1.0, 2.0), Omega, Alpha, FVector::ZeroVector).Equals(FVector(0.5, -1.0, 2.0), 1.e-12));

    // Against finite differences of a point carried by a rigid body: the origin accelerates, and the body
    // turns about a tilted fixed axis with an angular acceleration, the point rotated with FQuat.
    const FVector P0(1.0, -2.0, 0.5);
    const FVector V0(0.4, 0.3, -0.2);
    const FVector A0(0.5, -0.25, 0.1);
    const FVector Axis = FVector(1.0, 2.0, 3.0).GetSafeNormal();
    const double Theta0 = 0.3;
    const double Rate0 = 1.2;
    const double AngularAcceleration = 0.8;
    const FVector OffsetBody(0.3, -0.2, 0.1);
    const auto Theta = [&](double T) { return Theta0 + Rate0 * T + 0.5 * AngularAcceleration * T * T; };
    const auto PointPosition = [&](double T)
    {
        return P0 + V0 * T + A0 * (0.5 * T * T) + FQuat(Axis, Theta(T)).RotateVector(OffsetBody);
    };

    // Central differences with step H: the truncation error is H^2/6 |p'''| (velocity) and H^2/12 |p''''|
    // (acceleration), both below 1e-6 for these rates (|p'''| < 4, |p''''| < 10), the round-off error
    // about 1e-8; 1e-5 leaves room for both.
    constexpr double Time = 0.7;
    constexpr double H = 5.0e-4;
    const FVector FiniteVelocity = (PointPosition(Time + H) - PointPosition(Time - H)) / (2.0 * H);
    const FVector FiniteAcceleration = (PointPosition(Time + H) - 2.0 * PointPosition(Time) + PointPosition(Time - H)) / (H * H);

    const FVector OffsetWorld = FQuat(Axis, Theta(Time)).RotateVector(OffsetBody);
    const FVector AngularVelocity = Axis * (Rate0 + AngularAcceleration * Time);
    const FVector AngularAccelerationVector = Axis * AngularAcceleration;
    const FVector Velocity = UBot::SensorMath::RigidPointVelocity(V0 + A0 * Time, AngularVelocity, OffsetWorld);
    const FVector Acceleration = UBot::SensorMath::RigidPointAcceleration(A0, AngularVelocity, AngularAccelerationVector, OffsetWorld);

    TestTrue(FString::Printf(TEXT("RigidPointVelocity matches the finite difference (got %s, expected %s)"), *Velocity.ToString(), *FiniteVelocity.ToString()), Velocity.Equals(FiniteVelocity, 1.e-5));
    TestTrue(FString::Printf(TEXT("RigidPointAcceleration matches the finite difference (got %s, expected %s)"), *Acceleration.ToString(), *FiniteAcceleration.ToString()), Acceleration.Equals(FiniteAcceleration, 1.e-5));

    // The lever arm terms are large enough for the comparison to mean something.
    TestTrue(TEXT("The rotation contributes to the velocity"), (Velocity - (V0 + A0 * Time)).Size() > 0.1);
    TestTrue(TEXT("The rotation contributes to the acceleration"), (Acceleration - A0).Size() > 0.5);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorOdometryCovarianceTest, "UBotSensor.Odometry.Covariance", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorOdometryCovarianceTest::RunTest(const FString& Parameters)
{
    const TArray<double> Covariance = UBot::SensorMath::MakeDiagonalCovariance6(FVector(1.0, 2.0, 3.0), FVector(4.0, 5.0, 6.0));
    TestEqual(TEXT("Covariance has 36 values"), Covariance.Num(), 36);
    TestEqual(TEXT("X position variance"), Covariance[0], 1.0);
    TestEqual(TEXT("Z position variance"), Covariance[14], 3.0);
    TestEqual(TEXT("Yaw variance"), Covariance[35], 6.0);
    TestEqual(TEXT("Off diagonal remains zero"), Covariance[1], 0.0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorCameraIntrinsicsTest, "UBotSensor.Camera.Intrinsics", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorCameraIntrinsicsTest::RunTest(const FString& Parameters)
{
    float Fx = 0.0f;
    float Fy = 0.0f;
    float Cx = 0.0f;
    float Cy = 0.0f;
    UBot::SensorMath::CameraIntrinsics(640, 480, 90.0f, Fx, Fy, Cx, Cy);
    TestTrue(TEXT("90 degree camera focal length is half width"), FMath::IsNearlyEqual(Fx, 320.0f, 0.001f));
    TestEqual(TEXT("Square pixels use equal focal lengths"), Fy, Fx);
    TestEqual(TEXT("Principal point X uses pixel center convention"), Cx, 319.5f);
    TestEqual(TEXT("Principal point Y uses pixel center convention"), Cy, 239.5f);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorCameraVerticalFovTest, "UBotSensor.Camera.VerticalFOV", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorCameraVerticalFovTest::RunTest(const FString& Parameters)
{
    const float VerticalDegrees = FMath::RadiansToDegrees(UBot::SensorMath::CameraVerticalFovRadians(640, 480, 90.0f));
    TestTrue(TEXT("4:3 vertical FOV is about 73.74 degrees"), FMath::IsNearlyEqual(VerticalDegrees, 73.7398f, 0.01f));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorDepthProjectionTest, "UBotSensor.Depth.PointProjection", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorDepthProjectionTest::RunTest(const FString& Parameters)
{
    const FVector CenterPoint = UBot::SensorMath::DepthPixelToPointMeters(320, 240, 2.0f, 320.0f, 320.0f, 320.0f, 240.0f);
    TestTrue(TEXT("Center depth projects onto local X"), CenterPoint.Equals(FVector(2.0, 0.0, 0.0), 0.001));
    const FVector RightPoint = UBot::SensorMath::DepthPixelToPointMeters(480, 240, 2.0f, 320.0f, 320.0f, 320.0f, 240.0f);
    TestTrue(TEXT("Right image pixel has positive local Y"), FMath::IsNearlyEqual(RightPoint.Y, 1.0, 0.001));
    const FVector UpperPoint = UBot::SensorMath::DepthPixelToPointMeters(320, 80, 2.0f, 320.0f, 320.0f, 320.0f, 240.0f);
    TestTrue(TEXT("Upper image pixel has positive local Z"), FMath::IsNearlyEqual(UpperPoint.Z, 1.0, 0.001));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorDepthRangeTest, "UBotSensor.Depth.RangeFiltering", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorDepthRangeTest::RunTest(const FString& Parameters)
{
    TestEqual(TEXT("Scene depth converts centimeters to meters"), UBot::SensorMath::FilterSceneDepthCentimeters(250.0f, 0.1f, 100.0f), 2.5f);
    TestEqual(TEXT("Near out-of-range depth becomes invalid"), UBot::SensorMath::FilterSceneDepthCentimeters(5.0f, 0.1f, 100.0f), 0.0f);
    TestEqual(TEXT("Far out-of-range depth becomes invalid"), UBot::SensorMath::FilterSceneDepthCentimeters(15000.0f, 0.1f, 100.0f), 0.0f);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorDepthPreviewTest, "UBotSensor.Depth.PreviewColors", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorDepthPreviewTest::RunTest(const FString& Parameters)
{
    const FColor NearGray = UBot::SensorMath::DepthPreviewColor(1.0f, 1.0f, 11.0f, EUBotDepthPreviewMode::Grayscale);
    const FColor FarGray = UBot::SensorMath::DepthPreviewColor(11.0f, 1.0f, 11.0f, EUBotDepthPreviewMode::Grayscale);
    TestTrue(TEXT("Near grayscale depth is brighter"), NearGray.R > FarGray.R);
    TestEqual(TEXT("Invalid depth preview is black"), UBot::SensorMath::DepthPreviewColor(0.0f, 1.0f, 11.0f, EUBotDepthPreviewMode::PseudoColor), FColor::Black);
    TestTrue(TEXT("Pseudo color produces an opaque color"), UBot::SensorMath::DepthPreviewColor(6.0f, 1.0f, 11.0f, EUBotDepthPreviewMode::PseudoColor).A == 255);
    return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
