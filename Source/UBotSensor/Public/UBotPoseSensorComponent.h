#pragma once

#include "CoreMinimal.h"
#include "Engine/EngineTypes.h"
#include "UBotSensorComponent.h"
#include "UBotSensorTypes.h"
#include "UBotTypes.h"
#include "UBotPoseSensorComponent.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FUBotPoseDataEvent, const FUBotPoseSensorData&, Data);

UCLASS(Blueprintable, ClassGroup = (uBot), meta = (BlueprintSpawnableComponent, DisplayName = "uBot Pose Sensor"))
class UBOTSENSOR_API UUBotPoseSensorComponent : public UUBotSensorComponent
{
    GENERATED_BODY()

public:
    UUBotPoseSensorComponent();

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pose Sensor")
    EUBotPoseReferenceFrame ReferenceFrame = EUBotPoseReferenceFrame::World;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pose Sensor", meta = (EditCondition = "ReferenceFrame == EUBotPoseReferenceFrame::CustomComponent"))
    FComponentReference CustomReferenceComponent;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pose Sensor")
    FString ReferenceFrameId = TEXT("world");

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pose Sensor")
    bool bIncludeVelocity = true;

    UPROPERTY(BlueprintAssignable, Category = "Pose Sensor")
    FUBotPoseDataEvent OnPoseDataUpdated;

    UFUNCTION(BlueprintPure, Category = "uBot Sensor|Pose Sensor")
    FUBotPoseSensorData GetLatestPoseData() const { return LatestPoseData; }

    UFUNCTION(BlueprintCallable, Category = "uBot Sensor|Pose Sensor")
    bool SamplePose() { return SampleSensor(); }

    /** Returns "Pose". */
    virtual FName GetSensorTypeName() const override;

protected:
    virtual bool CaptureSensorData(FUBotFrameHeader& Header, FString& OutErrorMessage) override;
    virtual void OnSamplingStarted() override;

private:
    bool ResolvePoseInReferenceFrame(FTransform& OutPose, FString& OutResolvedFrameId, FString& OutErrorMessage) const;

    UPROPERTY(Transient)
    FUBotPoseSensorData LatestPoseData;

    FTransform PreviousPose = FTransform::Identity;
    double PreviousTimestampSeconds = 0.0;
    bool bHasPreviousPose = false;

// Added in 0.1.2. Everything from here on follows all earlier members on purpose: released UBotROS binaries
// access this class at compiled offsets, so new members go at the very end and no virtual function is added.
public:
    /**
     * When a body state source (a physics backend binding, a vehicle's own integrator) moves the body this
     * sensor is attached to, read velocities and accelerations from that state instead of differentiating
     * poses. The parts between that body and the sensor count as rigid; turn this off for a sensor on a part
     * that moves on its own, such as a gimbal. Only for the World reference frame; Parent and Custom
     * Component frames always differentiate the relative pose.
     */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Pose Sensor")
    bool bUseBodyState = true;

    /** True when the latest sample took its motion from a body state source. */
    UFUNCTION(BlueprintPure, Category = "uBot Sensor|Pose Sensor")
    bool IsUsingBodyState() const { return bLatestSampleUsedBodyState; }

    /** Sample time minus the time of the body state the latest sample used (how far the physics step lay behind the sample), s; 0 when it used none. */
    UFUNCTION(BlueprintPure, Category = "uBot Sensor|Pose Sensor")
    double GetBodyStateAgeSeconds() const { return LatestBodyStateAgeSeconds; }

private:
    bool bLatestSampleUsedBodyState = false;
    double LatestBodyStateAgeSeconds = 0.0;
};
