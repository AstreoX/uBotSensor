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
};
