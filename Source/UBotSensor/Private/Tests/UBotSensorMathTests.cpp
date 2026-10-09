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
