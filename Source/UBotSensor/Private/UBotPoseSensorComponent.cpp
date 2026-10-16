#include "UBotPoseSensorComponent.h"

#include "Components/SceneComponent.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/Actor.h"
#include "UBotSensorBodyMotion.h"
#include "UBotSensorMath.h"
#include "UBotUnits.h"
#include "UObject/ConstructorHelpers.h"

UUBotPoseSensorComponent::UUBotPoseSensorComponent()
{
    static ConstructorHelpers::FObjectFinder<UStaticMesh> SensorMesh(TEXT("/UBotSensor/Models/SM_UBotSensor_Pose.SM_UBotSensor_Pose"));
    if (SensorMesh.Succeeded())
    {
        SetStaticMesh(SensorMesh.Object);
    }
    SensorId = TEXT("pose_sensor");
    FrameId = TEXT("pose_sensor");
    UpdateRateHz = 30.0f;
}

FName UUBotPoseSensorComponent::GetSensorTypeName() const
{
    static const FName TypeName(TEXT("Pose"));
    return TypeName;
}

bool UUBotPoseSensorComponent::CaptureSensorData(FUBotFrameHeader& Header, FString& OutErrorMessage)
{
    namespace SensorBodyMotion = UBot::SensorBodyMotion;

    bLatestSampleUsedBodyState = false;
    LatestBodyStateAgeSeconds = 0.0;

    FTransform CurrentPose;
    FString ResolvedFrameId;
    if (!ResolvePoseInReferenceFrame(CurrentPose, ResolvedFrameId, OutErrorMessage))
    {
        MarkHeader(Header, EUBotSampleStatus::Error, false);
        LatestPoseData = FUBotPoseSensorData();
        LatestPoseData.Header = Header;
        OnPoseDataUpdated.Broadcast(LatestPoseData);
        return false;
    }

    LatestPoseData = FUBotPoseSensorData();
    LatestPoseData.Header = Header;
    LatestPoseData.ReferenceFrameId = ResolvedFrameId;
    LatestPoseData.PositionMeters = UBot::Units::CentimetersToMeters(CurrentPose.GetLocation());
    LatestPoseData.OrientationQuaternion = CurrentPose.GetRotation().GetNormalized();

    // State mode: the World frame, with a body state source that moves the sensor's body. Then the
    // velocities are the state's, mapped to the sensor point; a provided body without a state yet gives a
    // pose without velocity. Otherwise (difference mode, the only mode before 0.1.2) they are finite
    // differences of the previous and the current pose.
    SensorBodyMotion::FPointMotion Motion;
    SensorBodyMotion::EReadResult ReadResult = SensorBodyMotion::EReadResult::NoSource;
    if (bUseBodyState && ReferenceFrame == EUBotPoseReferenceFrame::World)
    {
        ReadResult = SensorBodyMotion::Read(*this, Motion);
    }

    if (ReadResult == SensorBodyMotion::EReadResult::Ok)
    {
        bLatestSampleUsedBodyState = true;
        LatestBodyStateAgeSeconds = Header.TimestampSeconds - Motion.StateTimeSeconds;
        if (bIncludeVelocity)
        {
            LatestPoseData.LinearVelocityMetersPerSecond = Motion.LinearVelocity;
            LatestPoseData.AngularVelocityRadPerSecond = Motion.AngularVelocity;
            LatestPoseData.bHasVelocity = true;
        }
    }
    else if (ReadResult == SensorBodyMotion::EReadResult::NoSource && bIncludeVelocity && bHasPreviousPose)
    {
        const double DeltaSeconds = Header.TimestampSeconds - PreviousTimestampSeconds;
        if (DeltaSeconds > UE_SMALL_NUMBER)
        {
            LatestPoseData.LinearVelocityMetersPerSecond = UBot::Units::CentimetersToMeters(CurrentPose.GetLocation() - PreviousPose.GetLocation()) / DeltaSeconds;
            LatestPoseData.AngularVelocityRadPerSecond = UBot::SensorMath::QuaternionDeltaToAngularVelocity(PreviousPose.GetRotation(), CurrentPose.GetRotation(), DeltaSeconds);
            LatestPoseData.bHasVelocity = true;
        }
    }

    PreviousPose = CurrentPose;
    PreviousTimestampSeconds = Header.TimestampSeconds;
    bHasPreviousPose = true;
    MarkHeader(Header, EUBotSampleStatus::Valid, true);
    LatestPoseData.Header = Header;
    OnPoseDataUpdated.Broadcast(LatestPoseData);
    return true;
}

void UUBotPoseSensorComponent::OnSamplingStarted()
{
    bHasPreviousPose = false;
    PreviousTimestampSeconds = 0.0;
}

bool UUBotPoseSensorComponent::ResolvePoseInReferenceFrame(FTransform& OutPose, FString& OutResolvedFrameId, FString& OutErrorMessage) const
{
    const FTransform SensorWorld = GetComponentTransform();
    switch (ReferenceFrame)
    {
        case EUBotPoseReferenceFrame::World:
            OutPose = SensorWorld;
            OutResolvedFrameId = ReferenceFrameId.IsEmpty() ? TEXT("world") : ReferenceFrameId;
            return true;

        case EUBotPoseReferenceFrame::Parent:
        {
            const USceneComponent* Parent = GetAttachParent();
            if (!Parent)
            {
                OutErrorMessage = TEXT("Pose Sensor has no parent component for Parent reference frame.");
                return false;
            }
            OutPose = SensorWorld.GetRelativeTransform(Parent->GetComponentTransform());
            OutResolvedFrameId = ReferenceFrameId.IsEmpty() ? Parent->GetName() : ReferenceFrameId;
            return true;
        }

        case EUBotPoseReferenceFrame::CustomComponent:
        {
            AActor* Owner = GetOwner();
            USceneComponent* ReferenceComponent = Cast<USceneComponent>(CustomReferenceComponent.GetComponent(Owner));
            if (!ReferenceComponent)
            {
                OutErrorMessage = TEXT("Pose Sensor custom reference component is invalid.");
                return false;
            }
            OutPose = SensorWorld.GetRelativeTransform(ReferenceComponent->GetComponentTransform());
            OutResolvedFrameId = ReferenceFrameId.IsEmpty() ? ReferenceComponent->GetName() : ReferenceFrameId;
            return true;
        }
    }

    OutErrorMessage = TEXT("Pose Sensor reference frame is unsupported.");
    return false;
}
