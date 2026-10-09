#pragma once

#include "CoreMinimal.h"
#include "UBotCameraSensorComponent.h"
#include "UBotSensorSchedulingTestTypes.generated.h"

// Test-only reflected types for UBotSensor.Scheduling.* (UHT cannot see types declared in a .cpp, and it does
// not allow WITH_DEV_AUTOMATION_TESTS guards, so these are compiled in every configuration).

// Befriended by UUBotCameraSensorComponent: exposes the pending synchronized-capture values that
// UUBotCameraSensorComponent::CaptureSynchronized stores for the next CaptureSensorData.
struct FUBotCameraSensorTestAccess
{
    static bool IsSynchronizedCapturePending(const UUBotCameraSensorComponent& Camera)
    {
        return Camera.bPendingSynchronizedCapture;
    }

    static int64 PendingGroupSequence(const UUBotCameraSensorComponent& Camera)
    {
        return Camera.PendingGroupSequence;
    }

    static double PendingGroupTimestampSeconds(const UUBotCameraSensorComponent& Camera)
    {
        return Camera.PendingGroupTimestampSeconds;
    }

    // The real CaptureSensorData consumes the pending capture; the test camera does the same.
    static void ClearSynchronizedCapturePending(UUBotCameraSensorComponent& Camera)
    {
        Camera.bPendingSynchronizedCapture = false;
    }
};

/** One CaptureSensorData call of a UUBotSensorSchedulingTestCamera. */
struct FUBotSchedulingTestCapture
{
    /** Header.TimestampSeconds as created by the base component: uBot sim time of the call. */
    double SampleTimestampSeconds = 0.0;

    /** Header.Sequence: increases by one per sample of this camera. */
    int64 SampleSequence = 0;

    /** True when the call came from CaptureSynchronized (a capture coordinator or capture group). */
    bool bSynchronized = false;

    /** Group sequence and timestamp handed to CaptureSynchronized (0 for free-running samples). */
    int64 GroupSequence = 0;
    double GroupTimestampSeconds = 0.0;
};

/**
 * Camera that records its captures and never renders: CaptureSensorData neither calls Super nor touches a
 * SceneCapture or render target, so it runs headless without a GPU. Everything else, including the
 * sampling tick, the capture group registration and CaptureSynchronized, is the real camera code.
 */
UCLASS(Transient, NotBlueprintable, NotPlaceable, HideDropdown)
class UUBotSensorSchedulingTestCamera : public UUBotCameraSensorComponent
{
    GENERATED_BODY()

public:
    TArray<FUBotSchedulingTestCapture> Captures;

protected:
    virtual bool CaptureSensorData(FUBotFrameHeader& Header, FString& OutErrorMessage) override
    {
        FUBotSchedulingTestCapture& Capture = Captures.AddDefaulted_GetRef();
        Capture.SampleTimestampSeconds = Header.TimestampSeconds;
        Capture.SampleSequence = Header.Sequence;
        Capture.bSynchronized = FUBotCameraSensorTestAccess::IsSynchronizedCapturePending(*this);
        if (Capture.bSynchronized)
        {
            Capture.GroupSequence = FUBotCameraSensorTestAccess::PendingGroupSequence(*this);
            Capture.GroupTimestampSeconds = FUBotCameraSensorTestAccess::PendingGroupTimestampSeconds(*this);
            FUBotCameraSensorTestAccess::ClearSynchronizedCapturePending(*this);
        }
        MarkHeader(Header, EUBotSampleStatus::Valid, true);
        return true;
    }

    // The base declares these as PURE_VIRTUAL; they are never called without a scene capture, but a
    // concrete test class should not depend on that.
    virtual EUBotImageEncoding GetImageEncoding() const override { return EUBotImageEncoding::BGRA8; }
    virtual ETextureRenderTargetFormat GetRequiredRenderTargetFormat() const override { return RTF_RGBA8; }
    virtual ESceneCaptureSource GetCaptureSource() const override { return SCS_FinalColorLDR; }
};
