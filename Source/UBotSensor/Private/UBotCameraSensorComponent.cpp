#include "UBotCameraSensorComponent.h"

#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "TextureResource.h"
#include "TimerManager.h"
#include "UBotCameraCaptureSubsystem.h"
#include "UBotImageFrame.h"
#include "UBotPerceptionMedium.h"
#include "UBotSensorMath.h"
#include "UBotSensorMedium.h"

UUBotCameraSensorComponent::UUBotCameraSensorComponent()
{
    // The legacy camera disabled ticking here because its base class sampled from a looping
    // FTimerManager timer. Sampling now runs from the base component tick (TG_PostPhysics,
    // FUBotSampleScheduler), so the tick settings of UUBotSensorComponent must be kept: with
    // bCanEverTick == false a free-running camera would only ever take the first sample.
    UpdateRateHz = 30.0f;
}

void UUBotCameraSensorComponent::BeginPlay()
{
    RegisterWithCaptureGroup();
    Super::BeginPlay();
    if (!CaptureGroupId.IsEmpty() && GetWorld())
    {
        // One-shot check on the next world tick, once every coordinator has had its BeginPlay.
        // This is not a sampling timer, so it does not go through FUBotSampleScheduler.
        GetWorld()->GetTimerManager().SetTimerForNextTick(FTimerDelegate::CreateUObject(this, &UUBotCameraSensorComponent::CheckCoordinatorAvailability));
    }
}

void UUBotCameraSensorComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    UnregisterFromCaptureGroup();
    Super::EndPlay(EndPlayReason);
}

bool UUBotCameraSensorComponent::CaptureSensorData(FUBotFrameHeader& Header, FString& OutErrorMessage)
{
    if (bPendingSynchronizedCapture)
    {
        OverrideHeaderTimestamp(Header, PendingGroupTimestampSeconds);
    }
    if (!EnsureCaptureResources(OutErrorMessage))
    {
        MarkHeader(Header, EUBotSampleStatus::Error, false);
        return false;
    }

    SceneCaptureComponent->CaptureScene();
    ++CaptureGeneration;
    UTextureRenderTarget2D* RenderTarget = GetPreviewRenderTarget();
    UUBotImageFrame* Frame = NewObject<UUBotImageFrame>(this);
    Frame->Header = Header;
    Frame->Width = RenderTarget->SizeX;
    Frame->Height = RenderTarget->SizeY;
    Frame->Encoding = GetImageEncoding();
    Frame->RowStrideBytes = Frame->Width * 4;
    Frame->HorizontalFovRadians = FMath::DegreesToRadians(HorizontalFovDegrees);
    Frame->VerticalFovRadians = UBot::SensorMath::CameraVerticalFovRadians(Frame->Width, Frame->Height, HorizontalFovDegrees);
    UBot::SensorMath::CameraIntrinsics(Frame->Width, Frame->Height, HorizontalFovDegrees, Frame->Fx, Frame->Fy, Frame->Cx, Frame->Cy);
    Frame->SensorTransformWorld = GetComponentTransform();
    Frame->CaptureGroupId = bPendingSynchronizedCapture ? CaptureGroupId : FString();
    Frame->CaptureSequence = bPendingSynchronizedCapture ? PendingGroupSequence : 0;
    Frame->CaptureTimestampSeconds = bPendingSynchronizedCapture ? PendingGroupTimestampSeconds : Header.TimestampSeconds;
    Frame->bSynchronized = bPendingSynchronizedCapture;
    Frame->RawCaptureRenderTarget = RenderTarget;
    Frame->SourceSensor = this;
    Frame->CaptureGeneration = CaptureGeneration;
    if (Frame->Encoding == EUBotImageEncoding::Depth32FC1)
    {
        Frame->RowStrideBytes = Frame->Width * static_cast<int32>(sizeof(float));
    }
    bPendingSynchronizedCapture = false;

    MarkHeader(Header, EUBotSampleStatus::Valid, true);
    Frame->Header = Header;
    LatestImageFrame = Frame;
    OnImageFrameReady.Broadcast(Frame);
    return true;
}

bool UUBotCameraSensorComponent::ShouldAutoStartSensor() const
{
    return CaptureGroupId.IsEmpty();
}

bool UUBotCameraSensorComponent::CanStartSensor() const
{
    // Grouped cameras never run their own schedule (no tick sampling); the coordinator drives them.
    return CaptureGroupId.IsEmpty();
}

void UUBotCameraSensorComponent::ConfigureSceneCapture(USceneCaptureComponent2D& CaptureComponent)
{
}

bool UUBotCameraSensorComponent::ProcessReadback(UUBotImageFrame& Frame, FString& ErrorMessage)
{
    FTextureRenderTargetResource* Resource = Frame.RawCaptureRenderTarget ? Frame.RawCaptureRenderTarget->GameThread_GetRenderTargetResource() : nullptr;
    if (!Resource)
    {
        ErrorMessage = TEXT("Camera render target resource is unavailable.");
        return false;
    }
    TArray<FColor> Colors;
    FReadSurfaceDataFlags ReadFlags(RCM_UNorm);
    ReadFlags.SetLinearToGamma(false);
    if (!Resource->ReadPixels(Colors, ReadFlags) || Colors.Num() != Frame.Width * Frame.Height)
    {
        ErrorMessage = TEXT("RGB camera pixel readback failed.");
        return false;
    }
    Frame.PixelBytes.SetNumUninitialized(Colors.Num() * 4);
    for (int32 Index = 0; Index < Colors.Num(); ++Index)
    {
        const int32 Offset = Index * 4;
        Frame.PixelBytes[Offset] = Colors[Index].B;
        Frame.PixelBytes[Offset + 1] = Colors[Index].G;
        Frame.PixelBytes[Offset + 2] = Colors[Index].R;
        Frame.PixelBytes[Offset + 3] = Colors[Index].A;
    }
    if (bEnableSmokeInterference && GetWorld())
    {
        UBot::SensorMedium::FCameraSettings MediumSettings;
        MediumSettings.LegacySmokeVolumeTag = SmokeVolumeActorTag;
        MediumSettings.DensityThreshold = SmokeDensityThreshold;
        MediumSettings.CellRadiusScale = SmokeCellRadiusScale;
        MediumSettings.Tint = SmokeInterferenceTint;
        MediumSettings.OpacityScale = SmokeInterferenceOpacityScale;
        MediumSettings.MaximumOpacity = MaximumSmokeInterferenceOpacity;

        TArray<FUBotMediumSnapshot> Media;
        UBot::SensorMedium::CollectMedia(*GetWorld(), MediumSettings, Media);
        UBot::SensorMedium::ApplyToBGRAPixels(
            Frame.SensorTransformWorld,
            Frame.Width,
            Frame.Height,
            Frame.Fx,
            Frame.Fy,
            Frame.Cx,
            Frame.Cy,
            PreviewDistanceMeters,
            Media,
            MediumSettings,
            Frame.PixelBytes);
    }
    return true;
}

UTextureRenderTarget2D* UUBotCameraSensorComponent::GetPreviewRenderTarget() const
{
    return ExternalRenderTarget ? ExternalRenderTarget : AutomaticRenderTarget;
}

bool UUBotCameraSensorComponent::ReadbackFrame(UUBotImageFrame* Frame, FString& ErrorMessage)
{
    if (!Frame || Frame->SourceSensor.Get() != this)
    {
        ErrorMessage = TEXT("Image frame does not belong to this camera sensor.");
        return false;
    }
    if (Frame->HasCPUPixelData())
    {
        return true;
    }
    if (Frame->CaptureGeneration != CaptureGeneration || Frame->RawCaptureRenderTarget != GetPreviewRenderTarget())
    {
        ErrorMessage = TEXT("Image frame is stale because the render target has been captured again.");
        return false;
    }
    return ProcessReadback(*Frame, ErrorMessage);
}

bool UUBotCameraSensorComponent::CaptureSynchronized(int64 GroupSequence, double GroupTimestampSeconds)
{
    PendingGroupSequence = GroupSequence;
    PendingGroupTimestampSeconds = GroupTimestampSeconds;
    bPendingSynchronizedCapture = true;
    return SampleSensor();
}

bool UUBotCameraSensorComponent::EnsureCaptureResources(FString& ErrorMessage)
{
    if (!EnsureSceneCaptureComponent(ErrorMessage))
    {
        return false;
    }
    const int32 SafeWidth = FMath::Clamp(ResolutionWidth, 16, 4096);
    const int32 SafeHeight = FMath::Clamp(ResolutionHeight, 16, 4096);
    UTextureRenderTarget2D* Target = ExternalRenderTarget;
    if (!Target)
    {
        if (!AutomaticRenderTarget || AutomaticRenderTarget->SizeX != SafeWidth || AutomaticRenderTarget->SizeY != SafeHeight || AutomaticRenderTarget->RenderTargetFormat != GetRequiredRenderTargetFormat())
        {
            AutomaticRenderTarget = NewObject<UTextureRenderTarget2D>(this);
            AutomaticRenderTarget->RenderTargetFormat = GetRequiredRenderTargetFormat();
            AutomaticRenderTarget->ClearColor = FLinearColor::Black;
            AutomaticRenderTarget->bAutoGenerateMips = false;
            AutomaticRenderTarget->InitAutoFormat(SafeWidth, SafeHeight);
            AutomaticRenderTarget->UpdateResourceImmediate(true);
        }
        Target = AutomaticRenderTarget;
    }
    if (!Target || !ValidateRenderTarget(*Target, ErrorMessage))
    {
        return false;
    }
    SceneCaptureComponent->TextureTarget = Target;
    SceneCaptureComponent->FOVAngle = FMath::Clamp(HorizontalFovDegrees, 1.0f, 170.0f);
    SceneCaptureComponent->CaptureSource = GetCaptureSource();
    ConfigureSceneCapture(*SceneCaptureComponent);
    return true;
}

bool UUBotCameraSensorComponent::EnsureSceneCaptureComponent(FString& ErrorMessage)
{
    if (SceneCaptureComponent && SceneCaptureComponent->IsRegistered())
    {
        return true;
    }
    AActor* Owner = GetOwner();
    if (!Owner || !GetWorld())
    {
        ErrorMessage = TEXT("Camera sensor requires a valid owner and world.");
        return false;
    }
    if (!SceneCaptureComponent)
    {
        SceneCaptureComponent = NewObject<USceneCaptureComponent2D>(Owner, NAME_None, RF_Transient);
        Owner->AddInstanceComponent(SceneCaptureComponent);
        SceneCaptureComponent->SetupAttachment(this);
        // Captures happen only on demand from CaptureSensorData.
        SceneCaptureComponent->bCaptureEveryFrame = false;
        SceneCaptureComponent->bCaptureOnMovement = false;
        SceneCaptureComponent->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_RenderScenePrimitives;
    }
    if (!SceneCaptureComponent->IsRegistered())
    {
        SceneCaptureComponent->RegisterComponent();
    }
    return SceneCaptureComponent->IsRegistered();
}

bool UUBotCameraSensorComponent::ValidateRenderTarget(const UTextureRenderTarget2D& RenderTarget, FString& ErrorMessage) const
{
    if (RenderTarget.SizeX < 1 || RenderTarget.SizeY < 1)
    {
        ErrorMessage = TEXT("Camera render target has invalid dimensions.");
        return false;
    }
    if (RenderTarget.RenderTargetFormat != GetRequiredRenderTargetFormat())
    {
        ErrorMessage = TEXT("Camera external render target format is incompatible with the sensor encoding.");
        return false;
    }
    return true;
}

void UUBotCameraSensorComponent::RegisterWithCaptureGroup()
{
    if (!CaptureGroupId.IsEmpty() && GetWorld())
    {
        if (UUBotCameraCaptureSubsystem* Subsystem = GetWorld()->GetSubsystem<UUBotCameraCaptureSubsystem>())
        {
            Subsystem->RegisterCamera(this);
        }
    }
}

void UUBotCameraSensorComponent::UnregisterFromCaptureGroup()
{
    if (GetWorld())
    {
        if (UUBotCameraCaptureSubsystem* Subsystem = GetWorld()->GetSubsystem<UUBotCameraCaptureSubsystem>())
        {
            Subsystem->UnregisterCamera(this);
        }
    }
}

void UUBotCameraSensorComponent::CheckCoordinatorAvailability()
{
    if (CaptureGroupId.IsEmpty() || !GetWorld() || bReportedMissingCoordinator)
    {
        return;
    }
    if (UUBotCameraCaptureSubsystem* Subsystem = GetWorld()->GetSubsystem<UUBotCameraCaptureSubsystem>())
    {
        if (!Subsystem->HasCoordinator(CaptureGroupId))
        {
            bReportedMissingCoordinator = true;
            OnSensorError.Broadcast(FString::Printf(TEXT("Camera capture group '%s' has no coordinator; automatic capture is disabled."), *CaptureGroupId));
        }
    }
}
