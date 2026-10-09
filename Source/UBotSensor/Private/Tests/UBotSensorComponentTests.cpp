#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Engine/Engine.h"
#include "Engine/EngineBaseTypes.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "GameFramework/PlayerState.h"
#include "GameFramework/WorldSettings.h"
#include "UBot2DLidarSensorComponent.h"
#include "UBot3DLidarSensorComponent.h"
#include "UBotCameraCaptureCoordinatorComponent.h"
#include "UBotCameraCaptureSubsystem.h"
#include "UBotCameraSensorComponent.h"
#include "UBotDepthCameraSensorComponent.h"
#include "UBotImuSensorComponent.h"
#include "UBotOdometrySensorComponent.h"
#include "UBotPoseSensorComponent.h"
#include "UBotRgbCameraSensorComponent.h"
#include "UBotSensorComponent.h"
#include "UBotSensorInterface.h"
#include "UBotSensorSchedulingTestTypes.h"
#include "UBotSensorTypes.h"
#include "UBotTypes.h"
#include "UObject/Package.h"

namespace UBotSensorComponentTests
{
    constexpr float FrameSeconds = 0.05f;

    // Transient game world, initialized for play and ticked by hand; torn down when the scope ends.
    class FScopedSensorTestWorld
    {
    public:
        UE_NONCOPYABLE(FScopedSensorTestWorld);

        FScopedSensorTestWorld()
            : World(UWorld::CreateWorld(EWorldType::Game, false))
            , CachedFrameCounter(GFrameCounter)
        {
            if (World && GEngine)
            {
                // UWorld::Tick may run a conditional garbage collection.
                World->AddToRoot();
                // UWorld::Tick looks its world context up in the engine.
                GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
                // Spawned actors are initialized, so EndPlay is routed when they are destroyed.
                World->InitializeActorsForPlay(FURL());
            }
        }

        ~FScopedSensorTestWorld()
        {
            if (!World)
            {
                return;
            }
            if (Pauser)
            {
                SetPaused(false);
            }
            if (World->AreActorsInitialized())
            {
                for (TActorIterator<AActor> It(World); It; ++It)
                {
                    It->RouteEndPlay(EEndPlayReason::LevelTransition);
                }
            }
            if (GEngine)
            {
                GEngine->DestroyWorldContext(World);
            }
            World->DestroyWorld(false);
            World->RemoveFromRoot();
            GFrameCounter = CachedFrameCounter;
        }

        UWorld* Get() const
        {
            return World;
        }

        void Tick(float DeltaSeconds) const
        {
            World->Tick(LEVELTICK_All, DeltaSeconds);
            // A tick function runs at most once per GFrameCounter value (FTickFunction::QueueTickFunction),
            // and the engine loop is not advancing the counter while a test ticks the world by hand.
            // Same as FTestWorldWrapper::TickTestWorld.
            ++GFrameCounter;
        }

        /**
         * Spawns an actor whose root is a new IMU at UpdateRateHz and begins play on it. BeginPlay
         * registers the component tick function and auto-starts the sensor (one immediate sample).
         */
        UUBotImuSensorComponent* SpawnImu(float UpdateRateHz) const
        {
            AActor* Owner = World ? World->SpawnActor<AActor>() : nullptr;
            if (!Owner)
            {
                return nullptr;
            }
            UUBotImuSensorComponent* Imu = NewObject<UUBotImuSensorComponent>(Owner, TEXT("TestImu"));
            Imu->UpdateRateHz = UpdateRateHz;
            Owner->SetRootComponent(Imu);
            Imu->RegisterComponent();
            Owner->DispatchBeginPlay();
            return Imu;
        }

        struct FSpawnedCamera
        {
            AActor* Owner = nullptr;
            UUBotSensorSchedulingTestCamera* Camera = nullptr;
        };

        /**
         * Spawns an actor whose root is a test camera in CaptureGroupId and begins play on it. An empty group id
         * makes a free-running camera: BeginPlay auto-starts it at CameraRateHz (one immediate capture). A grouped
         * camera never starts on its own (CanStartSensor); a coordinator of the same group captures it.
         */
        FSpawnedCamera SpawnTestCamera(const FString& CaptureGroupId, float CameraRateHz) const
        {
            FSpawnedCamera Spawned;
            Spawned.Owner = World ? World->SpawnActor<AActor>() : nullptr;
            if (!Spawned.Owner)
            {
                return Spawned;
            }
            Spawned.Camera = NewObject<UUBotSensorSchedulingTestCamera>(Spawned.Owner, TEXT("TestCamera"));
            Spawned.Camera->CaptureGroupId = CaptureGroupId;
            Spawned.Camera->UpdateRateHz = CameraRateHz;
            Spawned.Owner->SetRootComponent(Spawned.Camera);
            Spawned.Camera->RegisterComponent();
            Spawned.Owner->DispatchBeginPlay();
            return Spawned;
        }

        /**
         * Spawns an actor of its own with a capture coordinator for CaptureGroupId at RateHz and begins play on it.
         * The coordinator auto-starts at BeginPlay. Destroying its actor leaves the cameras of the group alive.
         */
        UUBotCameraCaptureCoordinatorComponent* SpawnCoordinator(const FString& CaptureGroupId, float RateHz) const
        {
            AActor* Owner = World ? World->SpawnActor<AActor>() : nullptr;
            if (!Owner)
            {
                return nullptr;
            }
            UUBotCameraCaptureCoordinatorComponent* Coordinator = NewObject<UUBotCameraCaptureCoordinatorComponent>(Owner, TEXT("TestCoordinator"));
            Coordinator->CaptureGroupId = CaptureGroupId;
            Coordinator->UpdateRateHz = RateHz;
            Coordinator->RegisterComponent();
            Owner->DispatchBeginPlay();
            return Coordinator;
        }

        /**
         * Pauses or resumes the world the way the game's pause does (the world settings' pauser): UWorld::IsPaused()
         * becomes true, sim time stops, and only tick functions with bTickEvenWhenPaused run. Returns whether the
         * world is in the requested state afterwards.
         */
        bool SetPaused(bool bPaused)
        {
            AWorldSettings* Settings = World ? World->GetWorldSettings(false, false) : nullptr;
            if (!Settings)
            {
                return false;
            }
            if (bPaused && !Pauser)
            {
                Pauser = World->SpawnActor<APlayerState>();
            }
            Settings->SetPauserPlayerState(bPaused ? Pauser : nullptr);
            return World->IsPaused() == bPaused;
        }

    private:
        UWorld* World = nullptr;
        APlayerState* Pauser = nullptr;
        uint64 CachedFrameCounter = 0;
    };

    // Ticks the world once and returns how many captures the camera took during that tick.
    int32 TickAndCountCaptures(const FScopedSensorTestWorld& Scope, const UUBotSensorSchedulingTestCamera& Camera, float DeltaSeconds)
    {
        const int32 CapturesBefore = Camera.Captures.Num();
        Scope.Tick(DeltaSeconds);
        return Camera.Captures.Num() - CapturesBefore;
    }

    // Ticks FrameCount frames of FrameSeconds and checks that every frame took exactly one capture, stamped
    // with the sim time of that frame.
    void TickFramesExpectingOneCapturePerFrame(FAutomationTestBase& Test, const FScopedSensorTestWorld& Scope, const UUBotSensorSchedulingTestCamera& Camera, int32 FrameCount, const TCHAR* Context)
    {
        int32 FramesWithoutCapture = 0;
        int32 FramesWithBurst = 0;
        int32 TimestampsOffSimTime = 0;
        for (int32 Frame = 0; Frame < FrameCount; ++Frame)
        {
            const int32 Taken = TickAndCountCaptures(Scope, Camera, FrameSeconds);
            FramesWithoutCapture += Taken < 1 ? 1 : 0;
            FramesWithBurst += Taken > 1 ? 1 : 0;
            if (Taken > 0 && !FMath::IsNearlyEqual(Camera.Captures.Last().SampleTimestampSeconds, Scope.Get()->GetTimeSeconds(), 1.e-9))
            {
                ++TimestampsOffSimTime;
            }
        }
        Test.TestEqual(FString::Printf(TEXT("%s: every frame takes a capture"), Context), FramesWithoutCapture, 0);
        Test.TestEqual(FString::Printf(TEXT("%s: never more than one capture per frame"), Context), FramesWithBurst, 0);
        Test.TestEqual(FString::Printf(TEXT("%s: captures are stamped with the sim time of their frame"), Context), TimestampsOffSimTime, 0);
    }

    // Checks the whole capture log of a camera. The log must start at the camera's first sample, so the sample
    // sequence is 1, 2, 3, ... and, for coordinator captures, the group sequence is 1, 2, 3, ... as well (the
    // coordinator keeps counting across StopCaptureGroup / StartCaptureGroup, like the legacy one).
    void VerifyCaptureLog(FAutomationTestBase& Test, const TArray<FUBotSchedulingTestCapture>& Captures, bool bExpectSynchronized, const TCHAR* Context)
    {
        int32 SynchronizationMismatches = 0;
        int32 SampleSequenceGaps = 0;
        int32 GroupSequenceGaps = 0;
        int32 GroupTimestampMismatches = 0;
        int32 NonIncreasingTimestamps = 0;
        for (int32 Index = 0; Index < Captures.Num(); ++Index)
        {
            const FUBotSchedulingTestCapture& Capture = Captures[Index];
            SynchronizationMismatches += Capture.bSynchronized == bExpectSynchronized ? 0 : 1;
            SampleSequenceGaps += Capture.SampleSequence == Index + 1 ? 0 : 1;
            if (bExpectSynchronized)
            {
                GroupSequenceGaps += Capture.GroupSequence == Index + 1 ? 0 : 1;
                // The coordinator stamps the group with the sim time of the tick that captured it.
                GroupTimestampMismatches += FMath::IsNearlyEqual(Capture.GroupTimestampSeconds, Capture.SampleTimestampSeconds, 1.e-9) ? 0 : 1;
            }
            NonIncreasingTimestamps += Index > 0 && Capture.SampleTimestampSeconds <= Captures[Index - 1].SampleTimestampSeconds ? 1 : 0;
        }
        Test.TestEqual(FString::Printf(TEXT("%s: capture kind (%s)"), Context, bExpectSynchronized ? TEXT("synchronized") : TEXT("free running")), SynchronizationMismatches, 0);
        Test.TestEqual(FString::Printf(TEXT("%s: the camera sequence rises by one per capture"), Context), SampleSequenceGaps, 0);
        Test.TestEqual(FString::Printf(TEXT("%s: the group sequence rises by one per group capture"), Context), GroupSequenceGaps, 0);
        Test.TestEqual(FString::Printf(TEXT("%s: the group timestamp is the sim time of the capture"), Context), GroupTimestampMismatches, 0);
        Test.TestEqual(FString::Printf(TEXT("%s: timestamps strictly increase"), Context), NonIncreasingTimestamps, 0);
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorSchedulingNoDuplicateSamplesTest, "UBotSensor.Scheduling.NoDuplicateSamplesPerFrame", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorSchedulingNoDuplicateSamplesTest::RunTest(const FString& Parameters)
{
    using namespace UBotSensorComponentTests;

    // A 100 Hz IMU in 20 fps frames: the legacy looping timer fired up to 5 times per frame with the
    // same world time, so every sample after the first reported InvalidDeltaTime.
    FScopedSensorTestWorld Scope;
    UWorld* World = Scope.Get();
    if (!TestNotNull(TEXT("Test world"), World))
    {
        return false;
    }
    UUBotImuSensorComponent* Imu = Scope.SpawnImu(100.0f);
    if (!TestNotNull(TEXT("IMU spawned"), Imu))
    {
        return false;
    }

    TestTrue(TEXT("BeginPlay auto-starts the sensor"), Imu->IsSensorRunning());
    TestTrue(TEXT("The sampling tick is registered"), Imu->PrimaryComponentTick.IsTickFunctionRegistered());
    TestTrue(TEXT("The sampling tick is enabled while running"), Imu->IsComponentTickEnabled());
    TestEqual(TEXT("The sampling tick runs after physics"), Imu->PrimaryComponentTick.TickGroup.GetValue(), TG_PostPhysics);
    const FUBotFrameHeader FirstHeader = Imu->GetLatestFrameHeader();
    TestEqual(TEXT("StartSensor takes one immediate sample"), FirstHeader.Sequence, static_cast<int64>(1));
    TestEqual(TEXT("The first sample is stamped with sim time"), FirstHeader.TimestampSeconds, World->GetTimeSeconds(), 1.e-9);

    // Sample 1 (StartSensor) and sample 2 (first frame) fill the IMU history; from then on every
    // sample has a 0.05 s delta and must be valid.
    constexpr int32 FrameCount = 40;
    constexpr int32 WarmUpFrames = 1;
    int64 PreviousSequence = FirstHeader.Sequence;
    double PreviousTimestamp = FirstHeader.TimestampSeconds;
    int32 FramesWithDuplicates = 0;
    int32 FramesWithoutSample = 0;
    int32 NonIncreasingTimestamps = 0;
    int32 TimestampsOffSimTime = 0;
    int32 InvalidDeltaTimeSamples = 0;
    int32 InvalidSamplesAfterWarmUp = 0;
    for (int32 Frame = 0; Frame < FrameCount; ++Frame)
    {
        Scope.Tick(FrameSeconds);

        const FUBotFrameHeader Header = Imu->GetLatestFrameHeader();
        const int64 SamplesThisFrame = Header.Sequence - PreviousSequence;
        FramesWithDuplicates += SamplesThisFrame > 1 ? 1 : 0;
        FramesWithoutSample += SamplesThisFrame < 1 ? 1 : 0;
        NonIncreasingTimestamps += SamplesThisFrame > 0 && Header.TimestampSeconds <= PreviousTimestamp ? 1 : 0;
        TimestampsOffSimTime += FMath::IsNearlyEqual(Header.TimestampSeconds, World->GetTimeSeconds(), 1.e-9) ? 0 : 1;

        const EUBotSampleStatus Status = Imu->GetLatestIMUData().Header.Status;
        InvalidDeltaTimeSamples += Status == EUBotSampleStatus::InvalidDeltaTime ? 1 : 0;
        InvalidSamplesAfterWarmUp += Frame >= WarmUpFrames && Status != EUBotSampleStatus::Valid ? 1 : 0;

        PreviousSequence = Header.Sequence;
        PreviousTimestamp = Header.TimestampSeconds;
    }

    TestEqual(TEXT("Never more than one sample per frame"), FramesWithDuplicates, 0);
    TestEqual(TEXT("A 100 Hz sensor samples in every 20 fps frame"), FramesWithoutSample, 0);
    TestEqual(TEXT("Timestamps strictly increase"), NonIncreasingTimestamps, 0);
    TestEqual(TEXT("Samples are stamped with the sim time of their frame"), TimestampsOffSimTime, 0);
    TestEqual(TEXT("No InvalidDeltaTime sample"), InvalidDeltaTimeSamples, 0);
    TestEqual(TEXT("Every sample after warm-up is valid"), InvalidSamplesAfterWarmUp, 0);
    TestEqual(TEXT("One sample per frame plus the immediate one"), Imu->GetLatestFrameHeader().Sequence, static_cast<int64>(FrameCount + 1));
    TestTrue(TEXT("Due times skipped by long frames are counted as dropped"), Imu->GetDroppedSampleCount() > 0);

    Imu->StopSensor();
    const int64 SequenceAtStop = Imu->GetLatestFrameHeader().Sequence;
    TestFalse(TEXT("StopSensor disables the sampling tick"), Imu->IsComponentTickEnabled());
    Scope.Tick(FrameSeconds);
    TestEqual(TEXT("A stopped sensor does not sample"), Imu->GetLatestFrameHeader().Sequence, SequenceAtStop);

    Imu->GetOwner()->Destroy();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorSchedulingRateChangeTest, "UBotSensor.Scheduling.RateChangeKeepsRunning", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorSchedulingRateChangeTest::RunTest(const FString& Parameters)
{
    using namespace UBotSensorComponentTests;

    FScopedSensorTestWorld Scope;
    UWorld* World = Scope.Get();
    if (!TestNotNull(TEXT("Test world"), World))
    {
        return false;
    }
    UUBotImuSensorComponent* Imu = Scope.SpawnImu(100.0f);
    if (!TestNotNull(TEXT("IMU spawned"), Imu))
    {
        return false;
    }
    for (int32 Frame = 0; Frame < 4; ++Frame)
    {
        Scope.Tick(FrameSeconds);
    }
    TestEqual(TEXT("IMU is valid before the rate change"), Imu->GetLatestIMUData().Header.Status, EUBotSampleStatus::Valid);

    // The rate changes in place: no stop/start, so no immediate sample and no history reset.
    const int64 SequenceBefore = Imu->GetLatestFrameHeader().Sequence;
    const int64 DroppedBefore = Imu->GetDroppedSampleCount();
    Imu->SetUpdateRateHz(10.0f);
    TestTrue(TEXT("The sensor keeps running"), Imu->IsSensorRunning());
    TestEqual(TEXT("UpdateRateHz is updated"), Imu->UpdateRateHz, 10.0f);
    TestEqual(TEXT("No sample is taken by the rate change"), Imu->GetLatestFrameHeader().Sequence, SequenceBefore);
    TestEqual(TEXT("The dropped counter is kept"), Imu->GetDroppedSampleCount(), DroppedBefore);

    // 20 frames of 0.05 s at 10 Hz: one sample every other frame.
    int64 PreviousSequence = SequenceBefore;
    int32 Samples = 0;
    int32 FramesWithDuplicates = 0;
    int32 InvalidSamples = 0;
    for (int32 Frame = 0; Frame < 20; ++Frame)
    {
        Scope.Tick(FrameSeconds);
        const int64 Sequence = Imu->GetLatestFrameHeader().Sequence;
        const int64 SamplesThisFrame = Sequence - PreviousSequence;
        Samples += static_cast<int32>(SamplesThisFrame);
        FramesWithDuplicates += SamplesThisFrame > 1 ? 1 : 0;
        InvalidSamples += SamplesThisFrame > 0 && Imu->GetLatestIMUData().Header.Status != EUBotSampleStatus::Valid ? 1 : 0;
        PreviousSequence = Sequence;
    }
    TestTrue(FString::Printf(TEXT("About 10 samples in one second at 10 Hz (got %d)"), Samples), Samples >= 9 && Samples <= 11);
    TestEqual(TEXT("Never more than one sample per frame"), FramesWithDuplicates, 0);
    TestEqual(TEXT("History is kept across the rate change: every sample is valid"), InvalidSamples, 0);

    Imu->SetUpdateRateHz(1000.0f);
    TestEqual(TEXT("UpdateRateHz is clamped to 240 Hz"), Imu->UpdateRateHz, 240.0f);
    Imu->SetUpdateRateHz(0.0f);
    TestEqual(TEXT("UpdateRateHz is clamped to 0.1 Hz"), Imu->UpdateRateHz, 0.1f);

    Imu->GetOwner()->Destroy();
    TestFalse(TEXT("EndPlay stops the sensor"), Imu->IsSensorRunning());
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorSchedulingCaptureGroupTickTest, "UBotSensor.Scheduling.CaptureGroupTick", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorSchedulingCaptureGroupTickTest::RunTest(const FString& Parameters)
{
    using namespace UBotSensorComponentTests;

    // A 30 Hz capture group in 20 fps frames: the legacy looping capture timer fired up to twice per frame
    // with the same world time, so a frame could capture the whole group twice. The test camera records its
    // captures and never renders; the coordinator, the group registry and CaptureSynchronized are the real code.
    FScopedSensorTestWorld Scope;
    UWorld* World = Scope.Get();
    if (!TestNotNull(TEXT("Test world"), World))
    {
        return false;
    }
    const FString GroupId = TEXT("G");
    const FScopedSensorTestWorld::FSpawnedCamera Spawned = Scope.SpawnTestCamera(GroupId, 30.0f);
    UUBotCameraCaptureCoordinatorComponent* Coordinator = Scope.SpawnCoordinator(GroupId, 30.0f);
    if (!TestNotNull(TEXT("Camera spawned"), Spawned.Camera) || !TestNotNull(TEXT("Coordinator spawned"), Coordinator))
    {
        return false;
    }
    UUBotSensorSchedulingTestCamera& Camera = *Spawned.Camera;

    TestEqual(TEXT("The coordinator sees the camera of its group"), Coordinator->GetRegisteredCameraCount(), 1);
    TestTrue(TEXT("BeginPlay auto-starts the capture group"), Coordinator->IsCaptureGroupRunning());
    TestTrue(TEXT("The coordinator tick is registered"), Coordinator->PrimaryComponentTick.IsTickFunctionRegistered());
    TestTrue(TEXT("The coordinator tick is enabled while the group runs"), Coordinator->IsComponentTickEnabled());
    TestEqual(TEXT("The coordinator tick runs after physics"), Coordinator->PrimaryComponentTick.TickGroup.GetValue(), TG_PostPhysics);
    TestFalse(TEXT("A grouped camera does not start on its own"), Camera.IsSensorRunning());
    TestFalse(TEXT("A grouped camera does not tick"), Camera.IsComponentTickEnabled());
    TestEqual(TEXT("Starting the group does not capture immediately"), Camera.Captures.Num(), 0);
    TestEqual(TEXT("Nothing is dropped before the first tick"), Coordinator->GetDroppedCaptureCount(), static_cast<int64>(0));

    // 30 Hz always has a capture due in a 0.05 s frame: exactly one group capture per frame.
    constexpr int32 FrameCount = 40;
    TickFramesExpectingOneCapturePerFrame(*this, Scope, Camera, FrameCount, TEXT("Running group"));
    TestEqual(TEXT("Running group: one capture per frame in total"), Camera.Captures.Num(), FrameCount);
    VerifyCaptureLog(*this, Camera.Captures, true, TEXT("Running group"));
    // 60 intervals of 1/30 s fit into the 2 s of 40 frames: 40 were captured, the other 20 were skipped by long
    // frames and counted instead of being captured in a burst.
    TestEqual(TEXT("Running group: captured plus dropped intervals cover every interval"), static_cast<int64>(Camera.Captures.Num()) + Coordinator->GetDroppedCaptureCount(), static_cast<int64>(60));
    TestFalse(TEXT("Running group: the camera still does not run its own schedule"), Camera.IsSensorRunning());
    TestEqual(TEXT("Running group: the camera drops no samples of its own"), Camera.GetDroppedSampleCount(), static_cast<int64>(0));

    // StopCaptureGroup disables the tick; the group captures nothing while stopped.
    Coordinator->StopCaptureGroup();
    const int32 CapturesAtStop = Camera.Captures.Num();
    TestFalse(TEXT("StopCaptureGroup stops the group"), Coordinator->IsCaptureGroupRunning());
    TestFalse(TEXT("StopCaptureGroup disables the coordinator tick"), Coordinator->IsComponentTickEnabled());
    for (int32 Frame = 0; Frame < 5; ++Frame)
    {
        Scope.Tick(FrameSeconds);
    }
    TestEqual(TEXT("A stopped group does not capture"), Camera.Captures.Num(), CapturesAtStop);

    // StartCaptureGroup re-arms the schedule: no immediate capture, the dropped counter starts again at zero,
    // and the group sequence and the camera sequence keep counting.
    TestTrue(TEXT("StartCaptureGroup restarts a stopped group"), Coordinator->StartCaptureGroup());
    TestTrue(TEXT("The restarted group is running"), Coordinator->IsCaptureGroupRunning());
    TestTrue(TEXT("The restarted coordinator tick is enabled"), Coordinator->IsComponentTickEnabled());
    TestEqual(TEXT("Restarting the group does not capture immediately"), Camera.Captures.Num(), CapturesAtStop);
    TestEqual(TEXT("Restarting the group clears the dropped counter"), Coordinator->GetDroppedCaptureCount(), static_cast<int64>(0));
    TickFramesExpectingOneCapturePerFrame(*this, Scope, Camera, 10, TEXT("Restarted group"));
    TestEqual(TEXT("Restarted group: captures resume one per frame"), Camera.Captures.Num(), CapturesAtStop + 10);
    VerifyCaptureLog(*this, Camera.Captures, true, TEXT("Restarted group"));

    // Destroying the actor of the coordinator ends play on it, which stops the group; the camera stays alive.
    const int32 CapturesBeforeDestroy = Camera.Captures.Num();
    Coordinator->GetOwner()->Destroy();
    TestFalse(TEXT("Destroying the coordinator stops the group"), Coordinator->IsCaptureGroupRunning());
    for (int32 Frame = 0; Frame < 5; ++Frame)
    {
        Scope.Tick(FrameSeconds);
    }
    TestEqual(TEXT("A destroyed coordinator does not capture"), Camera.Captures.Num(), CapturesBeforeDestroy);

    Spawned.Owner->Destroy();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorSchedulingCaptureGroupFirstCaptureTest, "UBotSensor.Scheduling.CaptureGroupFirstCapture", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorSchedulingCaptureGroupFirstCaptureTest::RunTest(const FString& Parameters)
{
    using namespace UBotSensorComponentTests;

    // A 10 Hz group in 0.04 s frames. Like the legacy looping timer (first delay == interval), the first
    // automatic capture is due one interval (0.1 s) after the group starts: not at BeginPlay, not at the first tick.
    constexpr float FineFrameSeconds = 0.04f;
    FScopedSensorTestWorld Scope;
    UWorld* World = Scope.Get();
    if (!TestNotNull(TEXT("Test world"), World))
    {
        return false;
    }
    const FString GroupId = TEXT("Slow");
    const FScopedSensorTestWorld::FSpawnedCamera Spawned = Scope.SpawnTestCamera(GroupId, 30.0f);
    UUBotCameraCaptureCoordinatorComponent* Coordinator = Scope.SpawnCoordinator(GroupId, 10.0f);
    if (!TestNotNull(TEXT("Camera spawned"), Spawned.Camera) || !TestNotNull(TEXT("Coordinator spawned"), Coordinator))
    {
        return false;
    }
    UUBotSensorSchedulingTestCamera& Camera = *Spawned.Camera;
    TestTrue(TEXT("BeginPlay auto-starts the capture group"), Coordinator->IsCaptureGroupRunning());
    TestEqual(TEXT("No capture at BeginPlay"), Camera.Captures.Num(), 0);

    Scope.Tick(FineFrameSeconds);
    Scope.Tick(FineFrameSeconds);
    TestEqual(TEXT("No capture before one interval has passed (t = 0.08 s)"), Camera.Captures.Num(), 0);

    Scope.Tick(FineFrameSeconds);
    if (TestEqual(TEXT("The first capture comes with the first tick past one interval (t = 0.12 s)"), Camera.Captures.Num(), 1))
    {
        const FUBotSchedulingTestCapture& First = Camera.Captures[0];
        TestTrue(TEXT("The first capture is synchronized"), First.bSynchronized);
        TestEqual(TEXT("The first group sequence is 1"), First.GroupSequence, static_cast<int64>(1));
        TestEqual(TEXT("The first camera sequence is 1"), First.SampleSequence, static_cast<int64>(1));
        TestEqual(TEXT("The group timestamp is the sim time of the tick"), First.GroupTimestampSeconds, World->GetTimeSeconds(), 1.e-9);
        TestEqual(TEXT("The camera sample is stamped with the sim time of the tick"), First.SampleTimestampSeconds, World->GetTimeSeconds(), 1.e-9);
    }

    // A manual capture runs the same path right away and takes the next group sequence.
    TestTrue(TEXT("CaptureGroupOnce captures the group"), Coordinator->CaptureGroupOnce());
    if (TestEqual(TEXT("CaptureGroupOnce captures right away"), Camera.Captures.Num(), 2))
    {
        const FUBotSchedulingTestCapture& Manual = Camera.Captures[1];
        TestTrue(TEXT("The manual capture is synchronized"), Manual.bSynchronized);
        TestEqual(TEXT("The manual capture takes the next group sequence"), Manual.GroupSequence, static_cast<int64>(2));
        TestEqual(TEXT("The manual capture group timestamp is the sim time"), Manual.GroupTimestampSeconds, World->GetTimeSeconds(), 1.e-9);
    }

    Coordinator->GetOwner()->Destroy();
    Spawned.Owner->Destroy();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorSchedulingFreeRunningCameraTest, "UBotSensor.Scheduling.FreeRunningCameraTick", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorSchedulingFreeRunningCameraTest::RunTest(const FString& Parameters)
{
    using namespace UBotSensorComponentTests;

    // A camera without a capture group samples from the base sensor tick, like every other sensor. The legacy
    // camera disabled ticking in its constructor, which with the tick-based base class would leave it with
    // only the immediate first sample.
    FScopedSensorTestWorld Scope;
    UWorld* World = Scope.Get();
    if (!TestNotNull(TEXT("Test world"), World))
    {
        return false;
    }
    const FScopedSensorTestWorld::FSpawnedCamera Spawned = Scope.SpawnTestCamera(FString(), 30.0f);
    if (!TestNotNull(TEXT("Camera spawned"), Spawned.Camera))
    {
        return false;
    }
    UUBotSensorSchedulingTestCamera& Camera = *Spawned.Camera;

    TestTrue(TEXT("BeginPlay auto-starts a free-running camera"), Camera.IsSensorRunning());
    TestTrue(TEXT("The camera tick is registered"), Camera.PrimaryComponentTick.IsTickFunctionRegistered());
    TestTrue(TEXT("The camera tick is enabled while running"), Camera.IsComponentTickEnabled());
    if (TestEqual(TEXT("StartSensor takes one immediate capture"), Camera.Captures.Num(), 1))
    {
        TestFalse(TEXT("A free-running capture is not synchronized"), Camera.Captures[0].bSynchronized);
    }

    constexpr int32 FrameCount = 40;
    TickFramesExpectingOneCapturePerFrame(*this, Scope, Camera, FrameCount, TEXT("Free-running camera"));
    TestEqual(TEXT("Free-running camera: one capture per frame plus the immediate one"), Camera.Captures.Num(), FrameCount + 1);
    VerifyCaptureLog(*this, Camera.Captures, false, TEXT("Free-running camera"));
    // 60 intervals of 1/30 s fit into 2 s: 40 were sampled, the other 20 were skipped by long frames and counted.
    TestEqual(TEXT("Free-running camera: sampled plus dropped intervals cover every interval"), static_cast<int64>(Camera.Captures.Num() - 1) + Camera.GetDroppedSampleCount(), static_cast<int64>(60));

    // A paused world stops sim time and does not run the sampling tick (bTickEvenWhenPaused is false); the
    // sensor stays running and continues without a burst or a repeated timestamp once the world resumes.
    const int32 CapturesBeforePause = Camera.Captures.Num();
    const double TimeBeforePause = World->GetTimeSeconds();
    if (!TestTrue(TEXT("The world can be paused"), Scope.SetPaused(true)))
    {
        return false;
    }
    for (int32 Frame = 0; Frame < 5; ++Frame)
    {
        Scope.Tick(FrameSeconds);
    }
    TestEqual(TEXT("Sim time stands still while paused"), World->GetTimeSeconds(), TimeBeforePause, 1.e-12);
    TestEqual(TEXT("A paused world takes no capture"), Camera.Captures.Num(), CapturesBeforePause);
    TestTrue(TEXT("The camera keeps running while the world is paused"), Camera.IsSensorRunning());
    TestTrue(TEXT("The world can be resumed"), Scope.SetPaused(false));
    TickFramesExpectingOneCapturePerFrame(*this, Scope, Camera, 10, TEXT("Resumed camera"));
    TestEqual(TEXT("Resumed camera: captures continue one per frame"), Camera.Captures.Num(), CapturesBeforePause + 10);
    VerifyCaptureLog(*this, Camera.Captures, false, TEXT("Resumed camera"));

    // StopSensor ends the captures; StartSensor re-arms: one immediate capture, the dropped counter starts
    // again at zero, and the camera sequence keeps counting.
    Camera.StopSensor();
    const int32 CapturesAtStop = Camera.Captures.Num();
    TestFalse(TEXT("StopSensor stops the camera"), Camera.IsSensorRunning());
    TestFalse(TEXT("StopSensor disables the camera tick"), Camera.IsComponentTickEnabled());
    for (int32 Frame = 0; Frame < 3; ++Frame)
    {
        Scope.Tick(FrameSeconds);
    }
    TestEqual(TEXT("A stopped camera takes no capture"), Camera.Captures.Num(), CapturesAtStop);

    TestTrue(TEXT("StartSensor restarts a stopped camera"), Camera.StartSensor());
    TestEqual(TEXT("StartSensor takes one immediate capture again"), Camera.Captures.Num(), CapturesAtStop + 1);
    TestEqual(TEXT("StartSensor clears the dropped counter"), Camera.GetDroppedSampleCount(), static_cast<int64>(0));
    TestTrue(TEXT("The restarted camera tick is enabled"), Camera.IsComponentTickEnabled());
    TickFramesExpectingOneCapturePerFrame(*this, Scope, Camera, 10, TEXT("Restarted camera"));
    TestEqual(TEXT("Restarted camera: captures resume one per frame"), Camera.Captures.Num(), CapturesAtStop + 11);
    VerifyCaptureLog(*this, Camera.Captures, false, TEXT("Restarted camera"));

    Spawned.Owner->Destroy();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorInterfaceImplementsTest, "UBotSensor.Interface.Implements", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorInterfaceImplementsTest::RunTest(const FString& Parameters)
{
    struct FExpectedSensor
    {
        UClass* Class;
        const TCHAR* TypeKey;
    };
    const FExpectedSensor ExpectedSensors[] =
    {
        { UUBotPoseSensorComponent::StaticClass(), TEXT("Pose") },
        { UUBotOdometrySensorComponent::StaticClass(), TEXT("Odometry") },
        { UUBotImuSensorComponent::StaticClass(), TEXT("IMU") },
        { UUBot2DLidarSensorComponent::StaticClass(), TEXT("Lidar2D") },
        { UUBot3DLidarSensorComponent::StaticClass(), TEXT("Lidar3D") },
        { UUBotRgbCameraSensorComponent::StaticClass(), TEXT("RGBCamera") },
        { UUBotDepthCameraSensorComponent::StaticClass(), TEXT("DepthCamera") },
    };

    TestTrue(TEXT("The base sensor component implements IUBotSensor"), UUBotSensorComponent::StaticClass()->ImplementsInterface(UUBotSensor::StaticClass()));
    TestEqual(TEXT("The base sensor component has no type key"), GetDefault<UUBotSensorComponent>()->GetSensorTypeName(), FName(NAME_None));

    for (const FExpectedSensor& Expected : ExpectedSensors)
    {
        const FString ClassName = Expected.Class->GetName();
        const FName ExpectedType(Expected.TypeKey);
        if (!TestTrue(FString::Printf(TEXT("%s implements IUBotSensor"), *ClassName), Expected.Class->ImplementsInterface(UUBotSensor::StaticClass())))
        {
            continue;
        }

        UUBotSensorComponent* Sensor = NewObject<UUBotSensorComponent>(GetTransientPackage(), Expected.Class);
        if (!TestNotNull(FString::Printf(TEXT("%s can be instantiated"), *ClassName), Sensor))
        {
            continue;
        }

        TestTrue(FString::Printf(TEXT("%s instance implements IUBotSensor"), *ClassName), Sensor->Implements<UUBotSensor>());
        TestEqual(FString::Printf(TEXT("%s type key (native)"), *ClassName), Sensor->GetSensorTypeName(), ExpectedType);
        TestEqual(FString::Printf(TEXT("%s type key (interface)"), *ClassName), IUBotSensor::Execute_GetSensorType(Sensor), ExpectedType);
        TestFalse(FString::Printf(TEXT("%s is not running before StartSensor"), *ClassName), IUBotSensor::Execute_IsSensorRunning(Sensor));
        TestTrue(FString::Printf(TEXT("%s is its own sensor frame component"), *ClassName), IUBotSensor::Execute_GetSensorFrameComponent(Sensor) == Sensor);
        TestEqual(FString::Printf(TEXT("%s has no header before sampling"), *ClassName), IUBotSensor::Execute_GetLatestSensorHeader(Sensor).Sequence, static_cast<int64>(0));

        Sensor->SensorId = TEXT("sensor_id");
        Sensor->FrameId = TEXT("frame_id");
        TestEqual(FString::Printf(TEXT("%s reports SensorId"), *ClassName), IUBotSensor::Execute_GetSensorId(Sensor), FString(TEXT("sensor_id")));
        TestEqual(FString::Printf(TEXT("%s reports FrameId"), *ClassName), IUBotSensor::Execute_GetSensorFrameId(Sensor), FString(TEXT("frame_id")));

        // Same fallbacks as the frame header: empty SensorId -> component name, empty FrameId -> sensor id.
        Sensor->FrameId.Reset();
        TestEqual(FString::Printf(TEXT("%s frame id falls back to the sensor id"), *ClassName), IUBotSensor::Execute_GetSensorFrameId(Sensor), FString(TEXT("sensor_id")));
        Sensor->SensorId.Reset();
        TestEqual(FString::Printf(TEXT("%s sensor id falls back to the component name"), *ClassName), IUBotSensor::Execute_GetSensorId(Sensor), Sensor->GetName());
        TestEqual(FString::Printf(TEXT("%s frame id falls back to the component name"), *ClassName), IUBotSensor::Execute_GetSensorFrameId(Sensor), Sensor->GetName());
    }
    return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
