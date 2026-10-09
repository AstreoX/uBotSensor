#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Engine/EngineBaseTypes.h"
#include "UBotCameraCaptureCoordinatorComponent.h"
#include "UBotCameraCaptureSubsystem.h"
#include "UBotCameraSensorComponent.h"
#include "UBotDepthCameraSensorComponent.h"
#include "UBotImageFrame.h"
#include "UBotPointCloudFrame.h"
#include "UBotRgbCameraSensorComponent.h"
#include "UBotSensorInterface.h"
#include "UBotSensorMath.h"
#include "UBotSensorTypes.h"
#include "UBotTypes.h"
#include "UObject/Package.h"

// Befriended by UUBotImageFrame: fills CPU pixel data directly so the frame API can be tested
// without a render target or GPU readback.
struct FUBotImageFrameTestAccess
{
    static TArray<uint8>& PixelBytes(UUBotImageFrame& Frame)
    {
        return Frame.PixelBytes;
    }

    static TArray<float>& DepthValuesMeters(UUBotImageFrame& Frame)
    {
        return Frame.DepthValuesMeters;
    }
};

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotCameraPixelBoundsTest, "UBotSensor.Camera.PixelBounds", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotCameraPixelBoundsTest::RunTest(const FString& Parameters)
{
    UUBotImageFrame* Frame = NewObject<UUBotImageFrame>();
    Frame->Width = 2;
    Frame->Height = 2;
    FColor Color = FColor::White;
    float Depth = -1.0f;
    TestFalse(TEXT("Image without CPU data rejects BGRA access"), Frame->GetBGRAPixelByCoordinates(0, 0, Color));
    TestEqual(TEXT("Rejected BGRA pixel resets color"), Color, FColor::Black);
    TestFalse(TEXT("Wrong encoding rejects depth access"), Frame->GetDepthMetersByCoordinates(0, 0, Depth));
    TestEqual(TEXT("Rejected depth resets value"), Depth, 0.0f);
    TestFalse(TEXT("Out of bounds pixel is rejected"), Frame->GetBGRAPixelByCoordinates(2, 0, Color));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotCameraImageFrameBGRATest, "UBotSensor.Camera.ImageFrameBGRA", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotCameraImageFrameBGRATest::RunTest(const FString& Parameters)
{
    UUBotImageFrame* Frame = NewObject<UUBotImageFrame>();
    Frame->Width = 2;
    Frame->Height = 1;
    Frame->Encoding = EUBotImageEncoding::BGRA8;
    TestFalse(TEXT("Fresh frame has no CPU data"), Frame->HasCPUPixelData());
    TestEqual(TEXT("Fresh frame reports zero bytes"), Frame->GetPixelDataSizeBytes(), 0);

    // Two pixels in B, G, R, A byte order.
    FUBotImageFrameTestAccess::PixelBytes(*Frame) = { 10, 20, 30, 255, 40, 50, 60, 128 };
    TestTrue(TEXT("Width * Height * 4 bytes count as CPU data"), Frame->HasCPUPixelData());
    TestEqual(TEXT("BGRA size is the byte count"), Frame->GetPixelDataSizeBytes(), 8);

    FColor Color = FColor::White;
    TestTrue(TEXT("First pixel is readable"), Frame->GetBGRAPixelByCoordinates(0, 0, Color));
    TestEqual(TEXT("First pixel maps BGRA bytes to FColor"), Color, FColor(30, 20, 10, 255));
    TestTrue(TEXT("Second pixel is readable"), Frame->GetBGRAPixelByCoordinates(1, 0, Color));
    TestEqual(TEXT("Second pixel uses row-major offset"), Color, FColor(60, 50, 40, 128));

    TestFalse(TEXT("Negative X is rejected"), Frame->GetBGRAPixelByCoordinates(-1, 0, Color));
    TestEqual(TEXT("Rejected pixel resets color"), Color, FColor::Black);
    TestFalse(TEXT("Y past the last row is rejected"), Frame->GetBGRAPixelByCoordinates(0, 1, Color));

    float Depth = -1.0f;
    TestFalse(TEXT("BGRA frame rejects depth access"), Frame->GetDepthMetersByCoordinates(0, 0, Depth));
    TestEqual(TEXT("Rejected depth resets value"), Depth, 0.0f);

    const TArray<uint8> Copy = Frame->CopyPixelBytes();
    TestEqual(TEXT("Copy has every byte"), Copy.Num(), 8);
    TestEqual(TEXT("Copy keeps the byte order"), static_cast<int32>(Copy[2]), 30);
    TestEqual(TEXT("Getter exposes the same bytes"), static_cast<int32>(Frame->GetPixelBytes()[4]), 40);
    TestEqual(TEXT("BGRA frame has no depth values"), Frame->CopyDepthValues().Num(), 0);

    FString ErrorMessage;
    TestTrue(TEXT("Readback request succeeds without GPU work when CPU data exists"), Frame->RequestPixelReadback(ErrorMessage));
    TestTrue(TEXT("Successful readback request leaves the error empty"), ErrorMessage.IsEmpty());

    TestNull(TEXT("BGRA frame cannot build a point cloud"), Frame->GeneratePointCloud(ErrorMessage));
    TestFalse(TEXT("Point cloud rejection explains why"), ErrorMessage.IsEmpty());

    ErrorMessage.Reset();
    TestNull(TEXT("BGRA frame has no depth preview"), Frame->UpdateDepthPreview(EUBotDepthPreviewMode::Grayscale, 0.1f, 10.0f, ErrorMessage));
    TestEqual(TEXT("Depth preview rejection explains the encoding"), ErrorMessage, FString(TEXT("Depth preview is only available for Depth32FC1 frames.")));

    FUBotImageFrameTestAccess::DepthValuesMeters(*Frame) = { 1.0f, 2.0f };
    FUBotImageFrameTestAccess::PixelBytes(*Frame).SetNum(4);
    TestFalse(TEXT("Depth values do not count as BGRA CPU data"), Frame->HasCPUPixelData());
    TestEqual(TEXT("BGRA size ignores depth values"), Frame->GetPixelDataSizeBytes(), 4);
    TestFalse(TEXT("Truncated pixel data is rejected"), Frame->GetBGRAPixelByCoordinates(0, 0, Color));

    Frame->ClearCPUPixelData();
    TestFalse(TEXT("Cleared frame has no CPU data"), Frame->HasCPUPixelData());
    TestEqual(TEXT("Cleared frame reports zero bytes"), Frame->GetPixelDataSizeBytes(), 0);
    TestEqual(TEXT("Clearing also drops depth values"), Frame->GetDepthValuesMeters().Num(), 0);

    ErrorMessage.Reset();
    TestFalse(TEXT("Readback without a source camera fails"), Frame->RequestPixelReadback(ErrorMessage));
    TestEqual(TEXT("Readback failure names the missing source"), ErrorMessage, FString(TEXT("Image frame source sensor is no longer valid.")));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotCameraImageFrameDepthTest, "UBotSensor.Camera.ImageFrameDepth", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotCameraImageFrameDepthTest::RunTest(const FString& Parameters)
{
    UUBotImageFrame* Frame = NewObject<UUBotImageFrame>();
    Frame->Width = 3;
    Frame->Height = 2;
    Frame->Encoding = EUBotImageEncoding::Depth32FC1;
    Frame->Fx = 2.0f;
    Frame->Fy = 2.0f;
    Frame->Cx = 1.0f;
    Frame->Cy = 0.5f;
    Frame->HorizontalFovRadians = 1.2f;
    Frame->VerticalFovRadians = 0.8f;
    Frame->SensorTransformWorld = FTransform(FVector(100.0, 200.0, 300.0));
    Frame->Header.SensorId = TEXT("depth_test");
    Frame->Header.FrameId = TEXT("depth_test_frame");
    Frame->Header.Sequence = 7;
    Frame->Header.TimestampSeconds = 4.25;
    Frame->Header.bValid = true;
    Frame->Header.Status = EUBotSampleStatus::Valid;

    // Row 0: 1, invalid, 2. Row 1: invalid (negative), 3, 4.
    FUBotImageFrameTestAccess::DepthValuesMeters(*Frame) = { 1.0f, 0.0f, 2.0f, -1.0f, 3.0f, 4.0f };
    TestTrue(TEXT("Width * Height depth values count as CPU data"), Frame->HasCPUPixelData());
    TestEqual(TEXT("Depth size is four bytes per value"), Frame->GetPixelDataSizeBytes(), 24);

    float Depth = -1.0f;
    TestTrue(TEXT("Last depth pixel is readable"), Frame->GetDepthMetersByCoordinates(2, 1, Depth));
    TestEqual(TEXT("Depth lookup is row-major"), Depth, 4.0f);
    TestTrue(TEXT("Invalid depth pixels are still addressable"), Frame->GetDepthMetersByCoordinates(1, 0, Depth));
    TestEqual(TEXT("Invalid depth reads as zero"), Depth, 0.0f);
    TestFalse(TEXT("Depth X past the last column is rejected"), Frame->GetDepthMetersByCoordinates(3, 0, Depth));
    TestEqual(TEXT("Rejected depth resets value"), Depth, 0.0f);

    FColor Color = FColor::White;
    TestFalse(TEXT("Depth frame rejects BGRA access"), Frame->GetBGRAPixelByCoordinates(0, 0, Color));
    TestEqual(TEXT("Rejected BGRA access resets color"), Color, FColor::Black);
    TestEqual(TEXT("Depth copy has every value"), Frame->CopyDepthValues().Num(), 6);

    FString ErrorMessage;
    UUBotPointCloudFrame* Cloud = Frame->GeneratePointCloud(ErrorMessage);
    TestNotNull(TEXT("CPU depth frame builds a point cloud"), Cloud);
    if (Cloud)
    {
        TestTrue(TEXT("Point cloud is owned by the image frame"), Cloud->GetOuter() == Frame);
        TestEqual(TEXT("Point cloud keeps the sensor id"), Cloud->Header.SensorId, FString(TEXT("depth_test")));
        TestEqual(TEXT("Point cloud keeps the sequence"), Cloud->Header.Sequence, static_cast<int64>(7));
        TestEqual(TEXT("Point cloud keeps the timestamp"), Cloud->Header.TimestampSeconds, 4.25);
        TestTrue(TEXT("Point cloud keeps the sensor transform"), Cloud->SensorTransformWorld.Equals(Frame->SensorTransformWorld));
        TestEqual(TEXT("Every pixel is a ray"), Cloud->GetTotalRayCount(), 6);
        TestEqual(TEXT("Rows become channels"), Cloud->ChannelCount, 2);
        TestEqual(TEXT("Columns become horizontal samples"), Cloud->HorizontalSampleCount, 3);
        TestEqual(TEXT("Horizontal FOV is copied"), Cloud->HorizontalFovRadians, 1.2f);
        TestEqual(TEXT("Vertical FOV is split up"), Cloud->VerticalFovUpRadians, 0.4f);
        TestEqual(TEXT("Vertical FOV is split down"), Cloud->VerticalFovDownRadians, 0.4f);
        TestEqual(TEXT("Only positive depths become points"), Cloud->GetPointCount(), 4);
        TestEqual(TEXT("Hit count matches the point count"), Cloud->HitPointCount, 4);
        TestFalse(TEXT("A depth camera reports no intensities"), Cloud->HasIntensityData());

        const TArray<FVector>& Points = Cloud->GetPointsMeters();
        const TArray<int32>& Rings = Cloud->GetRingIndices();
        const TArray<float>& TimeOffsets = Cloud->GetTimeOffsetsSeconds();
        TestEqual(TEXT("Ring indices stay aligned with points"), Rings.Num(), Points.Num());
        TestEqual(TEXT("Time offsets stay aligned with points"), TimeOffsets.Num(), Points.Num());
        if (Points.Num() == 4 && Rings.Num() == 4 && TimeOffsets.Num() == 4)
        {
            TestEqual(TEXT("First point ring"), Rings[0], 0);
            TestEqual(TEXT("Second point ring"), Rings[1], 0);
            TestEqual(TEXT("Third point ring"), Rings[2], 1);
            TestEqual(TEXT("Fourth point ring"), Rings[3], 1);
            TestEqual(TEXT("Depth images have no time offsets"), TimeOffsets[3], 0.0f);
            TestEqual(TEXT("Depth lies along local X"), Points[0].X, 1.0);
            TestTrue(TEXT("Points use the depth projection"), Points[1].Equals(UBot::SensorMath::DepthPixelToPointMeters(2, 0, 2.0f, 2.0f, 2.0f, 1.0f, 0.5f), 1.0e-4));
            TestTrue(TEXT("Second row point uses the depth projection"), Points[2].Equals(UBot::SensorMath::DepthPixelToPointMeters(1, 1, 3.0f, 2.0f, 2.0f, 1.0f, 0.5f), 1.0e-4));
        }
    }

    ErrorMessage.Reset();
    TestNull(TEXT("Empty preview range is rejected"), Frame->UpdateDepthPreview(EUBotDepthPreviewMode::PseudoColor, 5.0f, 5.0f, ErrorMessage));
    TestEqual(TEXT("Preview range rejection explains why"), ErrorMessage, FString(TEXT("Depth preview range is invalid.")));
    TestNull(TEXT("Rejected preview creates no texture"), Frame->GetDepthPreviewTexture());

    FUBotImageFrameTestAccess::PixelBytes(*Frame) = { 1, 2, 3, 4 };
    FUBotImageFrameTestAccess::DepthValuesMeters(*Frame).SetNum(5);
    TestFalse(TEXT("Truncated depth data is not CPU data"), Frame->HasCPUPixelData());
    TestEqual(TEXT("Depth size ignores BGRA bytes"), Frame->GetPixelDataSizeBytes(), 20);
    TestFalse(TEXT("Truncated depth data is rejected"), Frame->GetDepthMetersByCoordinates(0, 0, Depth));

    ErrorMessage.Reset();
    TestNull(TEXT("Point cloud requires complete CPU depth data"), Frame->GeneratePointCloud(ErrorMessage));
    TestEqual(TEXT("Point cloud rejection explains why"), ErrorMessage, FString(TEXT("A CPU-readable Depth32FC1 frame is required to generate a point cloud.")));

    ErrorMessage.Reset();
    TestNull(TEXT("Preview without CPU data and without a source camera fails"), Frame->UpdateDepthPreview(EUBotDepthPreviewMode::Grayscale, 0.1f, 10.0f, ErrorMessage));
    TestEqual(TEXT("Preview reports the failed readback"), ErrorMessage, FString(TEXT("Image frame source sensor is no longer valid.")));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotCameraTriggerConfigurationTest, "UBotSensor.Camera.TriggerConfiguration", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotCameraTriggerConfigurationTest::RunTest(const FString& Parameters)
{
    // Regression for the timer -> tick migration: the legacy camera constructor disabled ticking
    // because sampling ran from a looping timer. Sampling now runs from the sensor tick, so a camera
    // that cannot tick would only ever take its first sample.
    const UUBotRgbCameraSensorComponent* Rgb = GetDefault<UUBotRgbCameraSensorComponent>();
    const UUBotDepthCameraSensorComponent* Depth = GetDefault<UUBotDepthCameraSensorComponent>();
    const UUBotCameraSensorComponent* Cameras[] = { Rgb, Depth };
    for (const UUBotCameraSensorComponent* Camera : Cameras)
    {
        const FString ClassName = Camera->GetClass()->GetName();
        TestTrue(FString::Printf(TEXT("%s can tick (sampling runs from TickComponent)"), *ClassName), Camera->PrimaryComponentTick.bCanEverTick);
        TestFalse(FString::Printf(TEXT("%s does not tick before StartSensor"), *ClassName), Camera->PrimaryComponentTick.bStartWithTickEnabled);
        TestTrue(FString::Printf(TEXT("%s samples in TG_PostPhysics"), *ClassName), Camera->PrimaryComponentTick.TickGroup == TG_PostPhysics);
        TestFalse(FString::Printf(TEXT("%s does not sample while paused"), *ClassName), Camera->PrimaryComponentTick.bTickEvenWhenPaused);
        TestEqual(FString::Printf(TEXT("%s keeps the 30 Hz default"), *ClassName), Camera->UpdateRateHz, 30.0f);
        TestTrue(FString::Printf(TEXT("%s implements IUBotSensor"), *ClassName), Camera->GetClass()->ImplementsInterface(UUBotSensor::StaticClass()));
        TestTrue(FString::Printf(TEXT("%s is free running without a capture group"), *ClassName), Camera->CaptureGroupId.IsEmpty());
    }

    TestEqual(TEXT("RGB camera type key"), Rgb->GetSensorTypeName(), FName(TEXT("RGBCamera")));
    TestEqual(TEXT("Depth camera type key"), Depth->GetSensorTypeName(), FName(TEXT("DepthCamera")));
    TestEqual(TEXT("RGB camera default sensor id"), Rgb->SensorId, FString(TEXT("rgb_camera")));
    TestEqual(TEXT("Depth camera default sensor id"), Depth->SensorId, FString(TEXT("depth_camera")));

    const UUBotCameraCaptureCoordinatorComponent* Coordinator = GetDefault<UUBotCameraCaptureCoordinatorComponent>();
    TestTrue(TEXT("Coordinator can tick (captures run from TickComponent)"), Coordinator->PrimaryComponentTick.bCanEverTick);
    TestFalse(TEXT("Coordinator does not tick before StartCaptureGroup"), Coordinator->PrimaryComponentTick.bStartWithTickEnabled);
    TestTrue(TEXT("Coordinator captures in TG_PostPhysics"), Coordinator->PrimaryComponentTick.TickGroup == TG_PostPhysics);
    TestFalse(TEXT("Coordinator does not capture while paused"), Coordinator->PrimaryComponentTick.bTickEvenWhenPaused);
    TestFalse(TEXT("Coordinator is not running by default"), Coordinator->IsCaptureGroupRunning());
    TestEqual(TEXT("Coordinator has dropped no captures"), Coordinator->GetDroppedCaptureCount(), static_cast<int64>(0));
    TestEqual(TEXT("Coordinator keeps the 30 Hz default"), Coordinator->UpdateRateHz, 30.0f);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotCameraCaptureGroupRegistryTest, "UBotSensor.Camera.CaptureGroupRegistry", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotCameraCaptureGroupRegistryTest::RunTest(const FString& Parameters)
{
    const FString GroupId = TEXT("UBotSensorTestGroup");
    UUBotCameraCaptureSubsystem* Subsystem = NewObject<UUBotCameraCaptureSubsystem>(GetTransientPackage());

    FString ErrorMessage;
    UUBotCameraCaptureCoordinatorComponent* Unnamed = NewObject<UUBotCameraCaptureCoordinatorComponent>(GetTransientPackage());
    TestFalse(TEXT("Coordinator without a group id is rejected"), Subsystem->RegisterCoordinator(Unnamed, ErrorMessage));
    TestFalse(TEXT("Rejected coordinator gets an error"), ErrorMessage.IsEmpty());
    TestFalse(TEXT("Null coordinator is rejected"), Subsystem->RegisterCoordinator(nullptr, ErrorMessage));

    UUBotCameraCaptureCoordinatorComponent* First = NewObject<UUBotCameraCaptureCoordinatorComponent>(GetTransientPackage());
    UUBotCameraCaptureCoordinatorComponent* Second = NewObject<UUBotCameraCaptureCoordinatorComponent>(GetTransientPackage());
    First->CaptureGroupId = GroupId;
    Second->CaptureGroupId = GroupId;
    ErrorMessage.Reset();
    TestTrue(TEXT("First coordinator registers"), Subsystem->RegisterCoordinator(First, ErrorMessage));
    TestTrue(TEXT("Group has a coordinator"), Subsystem->HasCoordinator(GroupId));
    TestFalse(TEXT("Second coordinator for the same group is rejected"), Subsystem->RegisterCoordinator(Second, ErrorMessage));
    TestTrue(TEXT("Duplicate coordinator error names the conflict"), ErrorMessage.Contains(TEXT("already has a coordinator")));
    ErrorMessage.Reset();
    TestTrue(TEXT("Registering the same coordinator again is accepted"), Subsystem->RegisterCoordinator(First, ErrorMessage));
    Subsystem->UnregisterCoordinator(Second);
    TestTrue(TEXT("Unregistering a foreign coordinator keeps the owner"), Subsystem->HasCoordinator(GroupId));
    Subsystem->UnregisterCoordinator(First);
    TestFalse(TEXT("Unregistering the owner clears the group"), Subsystem->HasCoordinator(GroupId));

    UUBotRgbCameraSensorComponent* Ungrouped = NewObject<UUBotRgbCameraSensorComponent>(GetTransientPackage());
    Subsystem->RegisterCamera(Ungrouped);
    TestEqual(TEXT("Cameras without a group id are ignored"), Subsystem->GetRegisteredCameraCount(FString()), 0);

    UUBotRgbCameraSensorComponent* Rgb = NewObject<UUBotRgbCameraSensorComponent>(GetTransientPackage());
    UUBotDepthCameraSensorComponent* Depth = NewObject<UUBotDepthCameraSensorComponent>(GetTransientPackage());
    Rgb->CaptureGroupId = GroupId;
    Depth->CaptureGroupId = GroupId;
    Subsystem->RegisterCamera(Rgb);
    Subsystem->RegisterCamera(Rgb);
    Subsystem->RegisterCamera(Depth);
    TestEqual(TEXT("Each camera is registered once"), Subsystem->GetRegisteredCameraCount(GroupId), 2);

    // Synchronized capture of cameras that cannot render (no owner, no world): every camera reports a
    // failure, no CaptureScene happens and no image frame is published.
    ErrorMessage.Reset();
    TestFalse(TEXT("Group capture fails when a camera cannot capture"), Subsystem->CaptureGroup(GroupId, 5, 12.5, ErrorMessage));
    TestTrue(TEXT("Group failure names the group"), ErrorMessage.Contains(GroupId));
    TestNull(TEXT("Failed capture publishes no frame"), Rgb->GetLatestImageFrame());
    TestTrue(TEXT("Failed capture does not advance the capture generation"), Rgb->GetCaptureGeneration() == 0);
    const FUBotFrameHeader FailedHeader = Rgb->GetLatestFrameHeader();
    TestFalse(TEXT("Failed synchronized sample is invalid"), FailedHeader.bValid);
    TestTrue(TEXT("Failed synchronized sample reports an error"), FailedHeader.Status == EUBotSampleStatus::Error);
    TestEqual(TEXT("Synchronized sample carries the group timestamp"), FailedHeader.TimestampSeconds, 12.5);

    Subsystem->UnregisterCamera(Depth);
    TestEqual(TEXT("Unregistering removes only that camera"), Subsystem->GetRegisteredCameraCount(GroupId), 1);
    Subsystem->UnregisterCamera(Rgb);
    TestEqual(TEXT("Empty group has no cameras"), Subsystem->GetRegisteredCameraCount(GroupId), 0);

    ErrorMessage.Reset();
    TestFalse(TEXT("Capturing an empty group fails"), Subsystem->CaptureGroup(GroupId, 6, 13.0, ErrorMessage));
    TestTrue(TEXT("Empty group error says no cameras are registered"), ErrorMessage.Contains(TEXT("has no registered cameras")));
    return true;
}

#endif
