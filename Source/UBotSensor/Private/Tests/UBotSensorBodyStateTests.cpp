#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Components/SceneComponent.h"
#include "Engine/Engine.h"
#include "Engine/EngineBaseTypes.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "Math/RandomStream.h"
#include "UBotBodyState.h"
#include "UBotImuSensorComponent.h"
#include "UBotOdometrySensorComponent.h"
#include "UBotPoseSensorComponent.h"
#include "UBotSensorBodyStateTestTypes.h"
#include "UBotSensorTypes.h"
#include "UBotTypes.h"
#include "UBotUnits.h"

// Tests of the body state mode of the Pose, Odometry and IMU sensors (UBotSensor.BodyState.*). A test source
// (UUBotSensorTestStateSource) provides the root component of a spawned actor and reports a scripted state;
// the tests move that component along an analytic trajectory, tick a transient world by hand so that sim time
// is exactly what the test chose, and call SampleSensor themselves (the sensors' own ticks are switched off
// after the start sample).
namespace UBotSensorBodyStateTests
{
    // Frame lengths of the hand-ticked world: 200 Hz for the accuracy tests, 50 Hz for the status scripts.
    constexpr float FrameSeconds = 0.005f;
    constexpr float SlowFrameSeconds = 0.02f;

    // Transient game world, initialized for play and ticked by hand; torn down when the scope ends. Same
    // setup as the scheduling tests.
    class FScopedBodyStateWorld
    {
    public:
        UE_NONCOPYABLE(FScopedBodyStateWorld);

        FScopedBodyStateWorld()
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

        ~FScopedBodyStateWorld()
        {
            if (!World)
            {
                return;
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

        double TimeSeconds() const
        {
            return World->GetTimeSeconds();
        }

        void Tick(float DeltaSeconds) const
        {
            World->Tick(LEVELTICK_All, DeltaSeconds);
            // A tick function runs at most once per GFrameCounter value, and the engine loop is not advancing
            // the counter while a test ticks the world by hand.
            ++GFrameCounter;
        }

    private:
        UWorld* World = nullptr;
        uint64 CachedFrameCounter = 0;
    };

    // An actor whose root is the body, and (optionally) the test source that provides it.
    struct FBodyRig
    {
        AActor* Owner = nullptr;
        USceneComponent* Body = nullptr;
        UUBotSensorTestStateSource* Source = nullptr;
    };

    FBodyRig MakeBodyRig(UWorld& World, bool bWithSource)
    {
        FBodyRig Rig;
        Rig.Owner = World.SpawnActor<AActor>();
        if (!Rig.Owner)
        {
            return Rig;
        }
        Rig.Body = NewObject<USceneComponent>(Rig.Owner, TEXT("Body"));
        Rig.Owner->SetRootComponent(Rig.Body);
        Rig.Body->RegisterComponent();
        if (bWithSource)
        {
            Rig.Source = NewObject<UUBotSensorTestStateSource>(Rig.Owner, TEXT("StateSource"));
            Rig.Source->ProvidedBody = Rig.Body;
        }
        return Rig;
    }

    // Adds a sensor under Parent, at the given transform relative to it. The sensor does not start on its own.
    template <typename SensorType>
    SensorType* AddSensor(const FBodyRig& Rig, USceneComponent* Parent, const TCHAR* Name, const FVector& RelativeLocationCentimeters, const FQuat& RelativeRotation)
    {
        SensorType* Sensor = NewObject<SensorType>(Rig.Owner, Name);
        Sensor->bAutoStart = false;
        Sensor->SetupAttachment(Parent);
        Sensor->RegisterComponent();
        Sensor->SetRelativeLocationAndRotation(RelativeLocationCentimeters, RelativeRotation);
        return Sensor;
    }

    // Starts the sensor (one immediate sample) and switches its tick off: from here on the test takes every
    // sample by hand, exactly at the sim time it chose.
    void StartForManualSampling(FAutomationTestBase& Test, UUBotSensorComponent& Sensor, const TCHAR* Context)
    {
        Test.TestTrue(FString::Printf(TEXT("%s: StartSensor"), Context), Sensor.StartSensor());
        Sensor.SetComponentTickEnabled(false);
    }

    FVector GravityMetersPerSecondSquared(const UWorld& World)
    {
        return UBot::Units::CentimetersToMeters(World.GetGravityZ() * FVector::UpVector);
    }

    FString StatusName(EUBotSampleStatus Status)
    {
        return StaticEnum<EUBotSampleStatus>()->GetNameStringByValue(static_cast<int64>(Status));
    }

    void ExpectNear(FAutomationTestBase& Test, const FString& What, const FVector& Actual, const FVector& Expected, double Tolerance)
    {
        Test.TestTrue(FString::Printf(TEXT("%s (got %s, expected %s, tolerance %g)"), *What, *Actual.ToString(), *Expected.ToString(), Tolerance), (Actual - Expected).Size() <= Tolerance);
    }

    // Largest component difference of two rotations, after aligning the sign of the quaternions.
    double RotationError(const FQuat& A, const FQuat& B)
    {
        const double Sign = (A.X * B.X + A.Y * B.Y + A.Z * B.Z + A.W * B.W) < 0.0 ? -1.0 : 1.0;
        return FMath::Max(
            FMath::Max(FMath::Abs(A.X - Sign * B.X), FMath::Abs(A.Y - Sign * B.Y)),
            FMath::Max(FMath::Abs(A.Z - Sign * B.Z), FMath::Abs(A.W - Sign * B.W)));
    }

    // A rigid body on an analytic trajectory (SI, UE world axes, angular vectors in the FQuat sense):
    //   position     p(t) = P0 + V0 t + A0 t^2 / 2 + C sin(w t)   (accelerating, with a slowly varying part)
    //   orientation  q(t) = rotation by theta(t) = Theta0 + Rate0 t + AngularAcceleration t^2 / 2 about the
    //                fixed, tilted world axis n
    // A point fixed to the body at offset r (body frame) is at p(t) + R(theta(t)) r; its velocity and
    // acceleration follow by differentiating Rodrigues' formula, independently of the cross products the
    // sensors use.
    struct FTrajectory
    {
        FVector P0 = FVector(1.0, -2.0, 0.5);
        FVector V0 = FVector(0.6, 0.3, -0.2);
        FVector A0 = FVector(0.8, -0.4, 0.3);
        FVector SineAmplitude = FVector(0.10, 0.05, -0.05);
        double SineRadPerSecond = UE_DOUBLE_PI;
        FVector Axis = FVector(1.0, 2.0, 3.0).GetSafeNormal();
        double Theta0 = 0.3;
        double Rate0 = 0.8;
        double AngularAcceleration = 1.0;

        FVector Position(double T) const
        {
            return P0 + V0 * T + A0 * (0.5 * T * T) + SineAmplitude * FMath::Sin(SineRadPerSecond * T);
        }

        FVector Velocity(double T) const
        {
            return V0 + A0 * T + SineAmplitude * (SineRadPerSecond * FMath::Cos(SineRadPerSecond * T));
        }

        FVector Acceleration(double T) const
        {
            return A0 - SineAmplitude * (SineRadPerSecond * SineRadPerSecond * FMath::Sin(SineRadPerSecond * T));
        }

        double Theta(double T) const
        {
            return Theta0 + Rate0 * T + 0.5 * AngularAcceleration * T * T;
        }

        double ThetaRate(double T) const
        {
            return Rate0 + AngularAcceleration * T;
        }

        FQuat Orientation(double T) const
        {
            return FQuat(Axis, Theta(T));
        }

        FVector AngularVelocity(double T) const
        {
            return Axis * ThetaRate(T);
        }

        FVector AngularAccelerationVector() const
        {
            return Axis * AngularAcceleration;
        }

        // R(theta) v for the rotation about Axis, and its first and second derivative with respect to theta.
        FVector Rotated(double ThetaValue, const FVector& V) const
        {
            return V * FMath::Cos(ThetaValue) + FVector::CrossProduct(Axis, V) * FMath::Sin(ThetaValue) + Axis * (FVector::DotProduct(Axis, V) * (1.0 - FMath::Cos(ThetaValue)));
        }

        FVector RotatedFirstDerivative(double ThetaValue, const FVector& V) const
        {
            return V * (-FMath::Sin(ThetaValue)) + FVector::CrossProduct(Axis, V) * FMath::Cos(ThetaValue) + Axis * (FVector::DotProduct(Axis, V) * FMath::Sin(ThetaValue));
        }

        FVector RotatedSecondDerivative(double ThetaValue, const FVector& V) const
        {
            return V * (-FMath::Cos(ThetaValue)) + FVector::CrossProduct(Axis, V) * (-FMath::Sin(ThetaValue)) + Axis * (FVector::DotProduct(Axis, V) * FMath::Cos(ThetaValue));
        }

        FVector PointPosition(double T, const FVector& OffsetBodyMeters) const
        {
            return Position(T) + Rotated(Theta(T), OffsetBodyMeters);
        }

        FVector PointVelocity(double T, const FVector& OffsetBodyMeters) const
        {
            return Velocity(T) + RotatedFirstDerivative(Theta(T), OffsetBodyMeters) * ThetaRate(T);
        }

        FVector PointAcceleration(double T, const FVector& OffsetBodyMeters) const
        {
            return Acceleration(T)
                + RotatedFirstDerivative(Theta(T), OffsetBodyMeters) * AngularAcceleration
                + RotatedSecondDerivative(Theta(T), OffsetBodyMeters) * FMath::Square(ThetaRate(T));
        }

        // Bound of the jerk of a body point at distance OffsetLengthMeters, for rotation rates up to MaxRate
        // (|y'''| <= |p'''| + (3 |theta'| |theta''| + |theta'|^3) |r|, theta''' = 0).
        double PointJerkBound(double MaxRate, double OffsetLengthMeters) const
        {
            return SineAmplitude.Size() * FMath::Cube(SineRadPerSecond) + (3.0 * MaxRate * FMath::Abs(AngularAcceleration) + FMath::Cube(MaxRate)) * OffsetLengthMeters;
        }

        // The state a source would report at sim time T (its timestamp).
        FUBotBodyState State(double T, bool bWithAccelerations) const
        {
            FUBotBodyState Result;
            Result.PositionMeters = Position(T);
            Result.Orientation = Orientation(T);
            Result.LinearVelocityMetersPerSecond = Velocity(T);
            Result.AngularVelocityRadiansPerSecond = AngularVelocity(T);
            Result.LinearAccelerationMetersPerSecondSquared = Acceleration(T);
            Result.AngularAccelerationRadiansPerSecondSquared = AngularAccelerationVector();
            Result.bHasLinearAcceleration = bWithAccelerations;
            Result.bHasAngularAcceleration = bWithAccelerations;
            Result.TimestampSeconds = T;
            Result.Source = FName(TEXT("UBotSensorTest"));
            return Result;
        }
    };

    // Where a sensor sits on the body: offset in the body frame and a rotated mount.
    struct FMount
    {
        FVector OffsetBodyMeters = FVector(0.3, -0.2, 0.1);
        FQuat Rotation = FRotator(15.0, -40.0, 25.0).Quaternion();

        FVector OffsetBodyCentimeters() const
        {
            return UBot::Units::MetersToCentimeters(OffsetBodyMeters);
        }
    };

    // Puts the body at its place on the trajectory at sim time T, and the source reports the state of T.
    void ApplyTrajectory(const FBodyRig& Rig, const FTrajectory& Trajectory, double T, bool bWithAccelerations = true)
    {
        Rig.Body->SetWorldLocationAndRotation(UBot::Units::MetersToCentimeters(Trajectory.Position(T)), Trajectory.Orientation(T));
        if (Rig.Source)
        {
            Rig.Source->State = Trajectory.State(T, bWithAccelerations);
        }
    }

    struct FImuExpectation
    {
        FVector SpecificForce = FVector::ZeroVector;
        FVector AngularVelocity = FVector::ZeroVector;
        FQuat Orientation = FQuat::Identity;
    };

    // What an ideal IMU on the mount reports when the state of time T is exact.
    FImuExpectation ExpectedImu(const FTrajectory& Trajectory, const FMount& Mount, double T, const FVector& Gravity)
    {
        FImuExpectation Expected;
        Expected.Orientation = Trajectory.Orientation(T) * Mount.Rotation;
        Expected.SpecificForce = Expected.Orientation.UnrotateVector(Trajectory.PointAcceleration(T, Mount.OffsetBodyMeters) - Gravity);
        Expected.AngularVelocity = Expected.Orientation.UnrotateVector(Trajectory.AngularVelocity(T));
        return Expected;
    }

    // Checks the latest sample of an IMU: status, validity flag, the mode and age getters, the header time.
    void ExpectImuSample(FAutomationTestBase& Test, const UUBotImuSensorComponent& Imu, const TCHAR* Context, EUBotSampleStatus ExpectedStatus, bool bExpectedUsingBodyState, double ExpectedAgeSeconds, double ExpectedTimestampSeconds)
    {
        const FUBotImuSensorData Data = Imu.GetLatestIMUData();
        Test.TestTrue(FString::Printf(TEXT("%s: status is %s (got %s)"), Context, *StatusName(ExpectedStatus), *StatusName(Data.Header.Status)), Data.Header.Status == ExpectedStatus);
        Test.TestTrue(FString::Printf(TEXT("%s: bValid follows the status"), Context), Data.Header.bValid == (ExpectedStatus == EUBotSampleStatus::Valid));
        Test.TestTrue(FString::Printf(TEXT("%s: IsUsingBodyState is %s"), Context, bExpectedUsingBodyState ? TEXT("true") : TEXT("false")), Imu.IsUsingBodyState() == bExpectedUsingBodyState);
        Test.TestTrue(FString::Printf(TEXT("%s: body state age is %g (got %g)"), Context, ExpectedAgeSeconds, Imu.GetBodyStateAgeSeconds()), FMath::IsNearlyEqual(Imu.GetBodyStateAgeSeconds(), ExpectedAgeSeconds, 1.e-9));
        Test.TestTrue(FString::Printf(TEXT("%s: the header carries the sample time, not the state time"), Context), FMath::IsNearlyEqual(Data.Header.TimestampSeconds, ExpectedTimestampSeconds, 1.e-12));
    }

    void ExpectOdometrySample(FAutomationTestBase& Test, const UUBotOdometrySensorComponent& Odometry, const TCHAR* Context, EUBotSampleStatus ExpectedStatus, bool bExpectedUsingBodyState, double ExpectedAgeSeconds, double ExpectedTimestampSeconds)
    {
        const FUBotOdometrySensorData Data = Odometry.GetLatestOdometryData();
        Test.TestTrue(FString::Printf(TEXT("%s: status is %s (got %s)"), Context, *StatusName(ExpectedStatus), *StatusName(Data.Header.Status)), Data.Header.Status == ExpectedStatus);
        Test.TestTrue(FString::Printf(TEXT("%s: bValid follows the status"), Context), Data.Header.bValid == (ExpectedStatus == EUBotSampleStatus::Valid));
        Test.TestTrue(FString::Printf(TEXT("%s: IsUsingBodyState is %s"), Context, bExpectedUsingBodyState ? TEXT("true") : TEXT("false")), Odometry.IsUsingBodyState() == bExpectedUsingBodyState);
        Test.TestTrue(FString::Printf(TEXT("%s: body state age is %g (got %g)"), Context, ExpectedAgeSeconds, Odometry.GetBodyStateAgeSeconds()), FMath::IsNearlyEqual(Odometry.GetBodyStateAgeSeconds(), ExpectedAgeSeconds, 1.e-9));
        Test.TestTrue(FString::Printf(TEXT("%s: the header carries the sample time, not the state time"), Context), FMath::IsNearlyEqual(Data.Header.TimestampSeconds, ExpectedTimestampSeconds, 1.e-12));
    }

    // A body at rest at the world origin with a source and an IMU 0.5 m ahead of the body origin (identity
    // mount); the scripts of ImuStatuses move only the state, never the component.
    struct FRestRig
    {
        FBodyRig Rig;
        UUBotImuSensorComponent* Imu = nullptr;
    };

    FRestRig MakeRestRig(UWorld& World, const FVector& LeverCentimeters)
    {
        FRestRig Rest;
        Rest.Rig = MakeBodyRig(World, true);
        if (Rest.Rig.Body)
        {
            Rest.Imu = AddSensor<UUBotImuSensorComponent>(Rest.Rig, Rest.Rig.Body, TEXT("RestImu"), LeverCentimeters, FQuat::Identity);
            Rest.Rig.Owner->DispatchBeginPlay();
        }
        return Rest;
    }

    // Ticks one slow frame and lets the state describe the instant LagSeconds before the new sim time.
    void AdvanceRest(const FScopedBodyStateWorld& Scope, FRestRig& Rest, double LagSeconds)
    {
        Scope.Tick(SlowFrameSeconds);
        Rest.Rig.Source->State.TimestampSeconds = Scope.TimeSeconds() - LagSeconds;
    }

    // Reference for the shared post-processing of an IMU sample (bias random walk, noise, low-pass, clamp),
    // written out independently of the component: the random draws come in the order bias walk
    // (accelerometer, gyroscope), accelerometer noise, gyroscope noise; the random walk and the filter use the
    // interval between the sample headers.
    class FImuPostProcessReference
    {
    public:
        explicit FImuPostProcessReference(const UUBotImuSensorComponent& InImu)
            : Imu(InImu)
            , Stream(InImu.RandomSeed)
            , AccelerometerBias(InImu.AccelerometerBias)
            , GyroscopeBias(InImu.GyroscopeBias)
        {
        }

        void Apply(FVector& Acceleration, FVector& AngularVelocity, double DeltaSeconds)
        {
            if (Imu.bNoiseEnabled)
            {
                const float RandomWalkScale = FMath::Sqrt(static_cast<float>(DeltaSeconds));
                AccelerometerBias += Gaussian(Imu.AccelerometerBiasRandomWalkStdDev * RandomWalkScale);
                GyroscopeBias += Gaussian(Imu.GyroscopeBiasRandomWalkStdDev * RandomWalkScale);
                Acceleration += AccelerometerBias + Gaussian(Imu.AccelerometerNoiseStdDev);
                AngularVelocity += GyroscopeBias + Gaussian(Imu.GyroscopeNoiseStdDev);
            }
            if (Imu.bLowPassFilterEnabled)
            {
                const double Rc = 1.0 / (2.0 * UE_PI * FMath::Max(0.1f, Imu.LowPassCutoffHz));
                const double Alpha = DeltaSeconds / (Rc + DeltaSeconds);
                Acceleration = FMath::Lerp(PreviousAcceleration, Acceleration, Alpha);
                AngularVelocity = FMath::Lerp(PreviousAngularVelocity, AngularVelocity, Alpha);
            }
            if (Imu.bClampOutput)
            {
                Acceleration = Acceleration.GetClampedToMaxSize(Imu.MaxLinearAcceleration);
                AngularVelocity = AngularVelocity.GetClampedToMaxSize(Imu.MaxAngularVelocity);
            }
            PreviousAcceleration = Acceleration;
            PreviousAngularVelocity = AngularVelocity;
        }

    private:
        FVector Gaussian(float StandardDeviation)
        {
            if (StandardDeviation <= 0.0f)
            {
                return FVector::ZeroVector;
            }
            return FVector(
                Stream.GetFraction() + Stream.GetFraction() + Stream.GetFraction() - 1.5f,
                Stream.GetFraction() + Stream.GetFraction() + Stream.GetFraction() - 1.5f,
                Stream.GetFraction() + Stream.GetFraction() + Stream.GetFraction() - 1.5f) * (StandardDeviation * 1.41421356f);
        }

        const UUBotImuSensorComponent& Imu;
        FRandomStream Stream;
        FVector AccelerometerBias;
        FVector GyroscopeBias;
        FVector PreviousAcceleration = FVector::ZeroVector;
        FVector PreviousAngularVelocity = FVector::ZeroVector;
    };

    // Noise, filter and clamp settings of the noise test; the values are large enough to show up in every
    // sample and the clamps are low enough to be hit during the run.
    void ConfigureImu(UUBotImuSensorComponent& Imu, bool bUseBodyState, bool bNoise, bool bFilters)
    {
        Imu.bUseBodyState = bUseBodyState;
        Imu.bNoiseEnabled = bNoise;
        Imu.RandomSeed = 4242;
        Imu.AccelerometerNoiseStdDev = 0.05f;
        Imu.GyroscopeNoiseStdDev = 0.01f;
        Imu.AccelerometerBias = FVector(0.02, -0.01, 0.03);
        Imu.GyroscopeBias = FVector(0.001, 0.002, -0.001);
        Imu.AccelerometerBiasRandomWalkStdDev = 0.01f;
        Imu.GyroscopeBiasRandomWalkStdDev = 0.002f;
        Imu.bLowPassFilterEnabled = bFilters;
        Imu.LowPassCutoffHz = 20.0f;
        Imu.bClampOutput = bFilters;
        Imu.MaxLinearAcceleration = 11.0f;
        Imu.MaxAngularVelocity = 1.2f;
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorBodyStateImuMatchesDifferencesTest, "UBotSensor.BodyState.ImuMatchesDifferences", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorBodyStateImuMatchesDifferencesTest::RunTest(const FString& Parameters)
{
    using namespace UBotSensorBodyStateTests;

    FScopedBodyStateWorld Scope;
    UWorld* World = Scope.Get();
    if (!TestNotNull(TEXT("Test world"), World))
    {
        return false;
    }

    const FTrajectory Trajectory{};
    const FMount Mount{};
    const FVector Gravity = GravityMetersPerSecondSquared(*World);

    // One body with a source whose state carries the accelerations, with an IMU on the rotated mount that
    // reads the state and one that differentiates poses; and a second body whose state carries none, so its
    // IMU derives the accelerations from the state history.
    FBodyRig Rig = MakeBodyRig(*World, true);
    FBodyRig DerivedRig = MakeBodyRig(*World, true);
    if (!TestNotNull(TEXT("Body"), Rig.Body) || !TestNotNull(TEXT("Second body"), DerivedRig.Body))
    {
        return false;
    }
    UUBotImuSensorComponent* StateImu = AddSensor<UUBotImuSensorComponent>(Rig, Rig.Body, TEXT("StateImu"), Mount.OffsetBodyCentimeters(), Mount.Rotation);
    UUBotImuSensorComponent* DifferenceImu = AddSensor<UUBotImuSensorComponent>(Rig, Rig.Body, TEXT("DifferenceImu"), Mount.OffsetBodyCentimeters(), Mount.Rotation);
    UUBotImuSensorComponent* DerivedImu = AddSensor<UUBotImuSensorComponent>(DerivedRig, DerivedRig.Body, TEXT("DerivedImu"), Mount.OffsetBodyCentimeters(), Mount.Rotation);
    DifferenceImu->bUseBodyState = false;
    Rig.Owner->DispatchBeginPlay();
    DerivedRig.Owner->DispatchBeginPlay();

    TestTrue(TEXT("The test source provides the body"), UBot::BodyState::FindSource(Rig.Body) == Rig.Source);
    FUBotBodyState Probe;
    const USceneComponent* ProbeBody = nullptr;
    TestTrue(TEXT("The IMU reaches the state of the body through its attach parent"), UBot::BodyState::GetNearestBodyState(StateImu, Probe, &ProbeBody) && ProbeBody == Rig.Body);

    const double StartTime = Scope.TimeSeconds();
    ApplyTrajectory(Rig, Trajectory, StartTime);
    ApplyTrajectory(DerivedRig, Trajectory, StartTime, false);
    StartForManualSampling(*this, *StateImu, TEXT("State IMU"));
    StartForManualSampling(*this, *DifferenceImu, TEXT("Difference IMU"));
    StartForManualSampling(*this, *DerivedImu, TEXT("Derived IMU"));
    ExpectImuSample(*this, *StateImu, TEXT("State IMU, start sample"), EUBotSampleStatus::WarmingUp, true, 0.0, StartTime);
    ExpectImuSample(*this, *DifferenceImu, TEXT("Difference IMU, start sample"), EUBotSampleStatus::WarmingUp, false, 0.0, StartTime);
    ExpectImuSample(*this, *DerivedImu, TEXT("Derived IMU, start sample"), EUBotSampleStatus::WarmingUp, true, 0.0, StartTime);

    // Tolerances.
    //  - State mode with the accelerations in the state, or the derived angular acceleration (exact here: the
    //    angular velocity is linear in time): the sensor evaluates the same closed form at the same instant as
    //    the test, so only round-off remains (about 1e-12); 1e-6 is far above that and still far below
    //    any missing or wrong term (the smallest lever arm term here is above 0.2 m/s^2).
    //  - Difference mode: the acceleration is the second difference of the sensor point over three samples,
    //    y''(t - dt) + O(dt^2), rotated with the orientation of t, so it differs from the true value at t by
    //    dt |y'''|; |y'''| is bounded by PointJerkBound for the rotation rates of this run. The gyro is the
    //    rotation over one interval, the exact rate at t - dt/2, so it differs by AngularAcceleration dt / 2.
    //    Both bounds get 25 % for the O(dt^2) terms.
    //  - Derived accelerations: the body acceleration is the velocity change over one state step, the true
    //    value at t - dt/2: dt/2 |p'''| = dt/2 |C| w^3 for the translation (the angular part is exact); 1e-5
    //    covers the O(dt^2) terms.
    constexpr int32 StepCount = 200;
    const double Dt = static_cast<double>(FrameSeconds);
    const double MaxRate = Trajectory.ThetaRate(StepCount * Dt);
    const double OffsetLength = Mount.OffsetBodyMeters.Size();
    const double StateTolerance = 1.e-6;
    const double DifferenceForceTolerance = 1.25 * Dt * Trajectory.PointJerkBound(MaxRate, OffsetLength);
    const double DifferenceGyroTolerance = 1.25 * 0.5 * FMath::Abs(Trajectory.AngularAcceleration) * Dt;
    const double DerivedForceTolerance = 1.25 * 0.5 * Dt * Trajectory.SineAmplitude.Size() * FMath::Cube(Trajectory.SineRadPerSecond) + 1.e-5;

    int32 StateStatusErrors = 0;
    int32 DifferenceStatusErrors = 0;
    int32 DerivedStatusErrors = 0;
    int32 ModeErrors = 0;
    int32 TimestampErrors = 0;
    double MaxStateForceError = 0.0;
    double MaxStateGyroError = 0.0;
    double MaxStateOrientationError = 0.0;
    double MaxStateAge = 0.0;
    double MaxDifferenceForceError = 0.0;
    double MaxDifferenceGyroError = 0.0;
    double MaxDerivedForceError = 0.0;
    double MaxDerivedGyroError = 0.0;
    for (int32 Step = 1; Step <= StepCount; ++Step)
    {
        Scope.Tick(FrameSeconds);
        const double T = Scope.TimeSeconds();
        ApplyTrajectory(Rig, Trajectory, T);
        ApplyTrajectory(DerivedRig, Trajectory, T, false);
        StateImu->SampleSensor();
        DifferenceImu->SampleSensor();
        DerivedImu->SampleSensor();

        const FImuExpectation Expected = ExpectedImu(Trajectory, Mount, T, Gravity);
        const FUBotImuSensorData StateData = StateImu->GetLatestIMUData();
        const FUBotImuSensorData DifferenceData = DifferenceImu->GetLatestIMUData();
        const FUBotImuSensorData DerivedData = DerivedImu->GetLatestIMUData();

        // The start sample was the only warm-up sample in state mode; difference mode needs one more.
        StateStatusErrors += StateData.Header.Status == EUBotSampleStatus::Valid ? 0 : 1;
        DerivedStatusErrors += DerivedData.Header.Status == EUBotSampleStatus::Valid ? 0 : 1;
        DifferenceStatusErrors += DifferenceData.Header.Status == (Step >= 2 ? EUBotSampleStatus::Valid : EUBotSampleStatus::WarmingUp) ? 0 : 1;
        ModeErrors += (StateImu->IsUsingBodyState() && DerivedImu->IsUsingBodyState() && !DifferenceImu->IsUsingBodyState()) ? 0 : 1;
        TimestampErrors += (FMath::IsNearlyEqual(StateData.Header.TimestampSeconds, T, 1.e-12) && FMath::IsNearlyEqual(DifferenceData.Header.TimestampSeconds, T, 1.e-12) && FMath::IsNearlyEqual(DerivedData.Header.TimestampSeconds, T, 1.e-12)) ? 0 : 1;
        MaxStateAge = FMath::Max(MaxStateAge, FMath::Abs(StateImu->GetBodyStateAgeSeconds()));

        MaxStateForceError = FMath::Max(MaxStateForceError, (StateData.LinearAccelerationMetersPerSecondSquared - Expected.SpecificForce).Size());
        MaxStateGyroError = FMath::Max(MaxStateGyroError, (StateData.AngularVelocityRadPerSecond - Expected.AngularVelocity).Size());
        MaxStateOrientationError = FMath::Max(MaxStateOrientationError, RotationError(StateData.OrientationQuaternion, Expected.Orientation));
        MaxDerivedForceError = FMath::Max(MaxDerivedForceError, (DerivedData.LinearAccelerationMetersPerSecondSquared - Expected.SpecificForce).Size());
        MaxDerivedGyroError = FMath::Max(MaxDerivedGyroError, (DerivedData.AngularVelocityRadPerSecond - Expected.AngularVelocity).Size());
        if (Step >= 2)
        {
            MaxDifferenceForceError = FMath::Max(MaxDifferenceForceError, (DifferenceData.LinearAccelerationMetersPerSecondSquared - Expected.SpecificForce).Size());
            MaxDifferenceGyroError = FMath::Max(MaxDifferenceGyroError, (DifferenceData.AngularVelocityRadPerSecond - Expected.AngularVelocity).Size());
        }
    }

    TestEqual(TEXT("State mode: every sample after the start sample is valid"), StateStatusErrors, 0);
    TestEqual(TEXT("Derived accelerations: every sample after the start sample is valid"), DerivedStatusErrors, 0);
    TestEqual(TEXT("Difference mode: two warm-up samples, then valid"), DifferenceStatusErrors, 0);
    TestEqual(TEXT("IsUsingBodyState follows the mode"), ModeErrors, 0);
    TestEqual(TEXT("Every header carries the sim time of its sample"), TimestampErrors, 0);
    TestTrue(FString::Printf(TEXT("The state of the sample time is exactly as old as the sample (max age %g)"), MaxStateAge), MaxStateAge <= 1.e-12);
    TestTrue(FString::Printf(TEXT("State mode matches the analytic specific force (max error %g, tolerance %g)"), MaxStateForceError, StateTolerance), MaxStateForceError <= StateTolerance);
    TestTrue(FString::Printf(TEXT("State mode matches the analytic gyro (max error %g, tolerance %g)"), MaxStateGyroError, StateTolerance), MaxStateGyroError <= StateTolerance);
    TestTrue(FString::Printf(TEXT("State mode reports the orientation of the sensor frame (max error %g)"), MaxStateOrientationError), MaxStateOrientationError <= 1.e-9);
    TestTrue(FString::Printf(TEXT("Difference mode matches the state within the finite-difference error (specific force: max error %g, tolerance %g)"), MaxDifferenceForceError, DifferenceForceTolerance), MaxDifferenceForceError <= DifferenceForceTolerance);
    TestTrue(FString::Printf(TEXT("Difference mode matches the state within the finite-difference error (gyro: max error %g, tolerance %g)"), MaxDifferenceGyroError, DifferenceGyroTolerance), MaxDifferenceGyroError <= DifferenceGyroTolerance);
    TestTrue(FString::Printf(TEXT("Derived accelerations match the analytic specific force within one state step (max error %g, tolerance %g)"), MaxDerivedForceError, DerivedForceTolerance), MaxDerivedForceError <= DerivedForceTolerance);
    TestTrue(FString::Printf(TEXT("A derived angular acceleration is exact for a constant one: gyro (max error %g)"), MaxDerivedGyroError), MaxDerivedGyroError <= StateTolerance);

    // The lever arm terms (alpha x r, the centripetal term) are what the comparison is about: they must be
    // large against the tolerance, or a missing term could hide inside it.
    const double EndTime = Scope.TimeSeconds();
    const double LeverArmAcceleration = (Trajectory.PointAcceleration(EndTime, Mount.OffsetBodyMeters) - Trajectory.Acceleration(EndTime)).Size();
    TestTrue(FString::Printf(TEXT("The lever arm acceleration (%g m/s^2) is far above the difference tolerance (%g)"), LeverArmAcceleration, DifferenceForceTolerance), LeverArmAcceleration > 4.0 * DifferenceForceTolerance);

    Rig.Owner->Destroy();
    DerivedRig.Owner->Destroy();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorBodyStateImuStatusesTest, "UBotSensor.BodyState.ImuStatuses", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorBodyStateImuStatusesTest::RunTest(const FString& Parameters)
{
    using namespace UBotSensorBodyStateTests;

    FScopedBodyStateWorld Scope;
    UWorld* World = Scope.Get();
    if (!TestNotNull(TEXT("Test world"), World))
    {
        return false;
    }

    const FVector Gravity = GravityMetersPerSecondSquared(*World);
    const FVector Lever(0.5, 0.0, 0.0);
    const FVector LeverCentimeters = UBot::Units::MetersToCentimeters(Lever);
    const double Step = static_cast<double>(SlowFrameSeconds);

    // 1. Without a source, and with a source but bUseBodyState off: difference mode, the same outputs. The
    //    bodies move on the analytic trajectory; both IMUs warm up for two samples (the start sample and the
    //    next one), the third is valid.
    {
        const FTrajectory Trajectory{};
        const FMount Mount{};
        FBodyRig Plain = MakeBodyRig(*World, false);
        FBodyRig Disabled = MakeBodyRig(*World, true);
        if (!TestNotNull(TEXT("Plain body"), Plain.Body) || !TestNotNull(TEXT("Disabled body"), Disabled.Body))
        {
            return false;
        }
        UUBotImuSensorComponent* PlainImu = AddSensor<UUBotImuSensorComponent>(Plain, Plain.Body, TEXT("PlainImu"), Mount.OffsetBodyCentimeters(), Mount.Rotation);
        UUBotImuSensorComponent* DisabledImu = AddSensor<UUBotImuSensorComponent>(Disabled, Disabled.Body, TEXT("DisabledImu"), Mount.OffsetBodyCentimeters(), Mount.Rotation);
        DisabledImu->bUseBodyState = false;
        Plain.Owner->DispatchBeginPlay();
        Disabled.Owner->DispatchBeginPlay();

        ApplyTrajectory(Plain, Trajectory, Scope.TimeSeconds());
        ApplyTrajectory(Disabled, Trajectory, Scope.TimeSeconds());
        StartForManualSampling(*this, *PlainImu, TEXT("No source"));
        StartForManualSampling(*this, *DisabledImu, TEXT("Source off"));
        ExpectImuSample(*this, *PlainImu, TEXT("No source, sample 1"), EUBotSampleStatus::WarmingUp, false, 0.0, Scope.TimeSeconds());
        ExpectImuSample(*this, *DisabledImu, TEXT("Source off, sample 1"), EUBotSampleStatus::WarmingUp, false, 0.0, Scope.TimeSeconds());

        double MaxOutputDifference = 0.0;
        for (int32 Index = 2; Index <= 7; ++Index)
        {
            Scope.Tick(SlowFrameSeconds);
            ApplyTrajectory(Plain, Trajectory, Scope.TimeSeconds());
            ApplyTrajectory(Disabled, Trajectory, Scope.TimeSeconds());
            PlainImu->SampleSensor();
            DisabledImu->SampleSensor();
            const EUBotSampleStatus ExpectedStatus = Index <= 2 ? EUBotSampleStatus::WarmingUp : EUBotSampleStatus::Valid;
            const FString Context = FString::Printf(TEXT("Difference mode, sample %d"), Index);
            ExpectImuSample(*this, *PlainImu, *(Context + TEXT(" (no source)")), ExpectedStatus, false, 0.0, Scope.TimeSeconds());
            ExpectImuSample(*this, *DisabledImu, *(Context + TEXT(" (source off)")), ExpectedStatus, false, 0.0, Scope.TimeSeconds());
            const FUBotImuSensorData PlainData = PlainImu->GetLatestIMUData();
            const FUBotImuSensorData DisabledData = DisabledImu->GetLatestIMUData();
            MaxOutputDifference = FMath::Max(MaxOutputDifference, (PlainData.LinearAccelerationMetersPerSecondSquared - DisabledData.LinearAccelerationMetersPerSecondSquared).Size());
            MaxOutputDifference = FMath::Max(MaxOutputDifference, (PlainData.AngularVelocityRadPerSecond - DisabledData.AngularVelocityRadPerSecond).Size());
            MaxOutputDifference = FMath::Max(MaxOutputDifference, RotationError(PlainData.OrientationQuaternion, DisabledData.OrientationQuaternion));
        }
        TestTrue(FString::Printf(TEXT("A source with bUseBodyState off gives the same outputs as no source (max difference %g)"), MaxOutputDifference), MaxOutputDifference <= 1.e-12);
        Plain.Owner->Destroy();
        Disabled.Owner->Destroy();
    }

    // 2. A state that carries the accelerations: warm-up on the start sample only, then valid; states that
    //    are stale or from the future end in InvalidDeltaTime and the next fresh state is valid again.
    {
        FRestRig Rest = MakeRestRig(*World, LeverCentimeters);
        if (!TestNotNull(TEXT("Rest body"), Rest.Rig.Body))
        {
            return false;
        }
        const FVector BodyAcceleration(1.0, 0.5, -0.5);
        FUBotBodyState& State = Rest.Rig.Source->State;
        State.TimestampSeconds = Scope.TimeSeconds();
        State.LinearAccelerationMetersPerSecondSquared = BodyAcceleration;
        State.bHasLinearAcceleration = true;
        State.bHasAngularAcceleration = true;

        StartForManualSampling(*this, *Rest.Imu, TEXT("Accelerations given"));
        ExpectImuSample(*this, *Rest.Imu, TEXT("Accelerations given, start sample"), EUBotSampleStatus::WarmingUp, true, 0.0, Scope.TimeSeconds());

        struct FScriptedSample
        {
            const TCHAR* Name;
            double LagSeconds;
            EUBotSampleStatus Status;
        };
        const FScriptedSample Script[] =
        {
            { TEXT("fresh state"), 0.0, EUBotSampleStatus::Valid },
            { TEXT("state one physics step old"), 0.004, EUBotSampleStatus::Valid },
            { TEXT("state older than the maximum sample delta"), 0.6, EUBotSampleStatus::InvalidDeltaTime },
            { TEXT("fresh state after the stale one"), 0.0, EUBotSampleStatus::Valid },
            { TEXT("state from the future"), -0.01, EUBotSampleStatus::InvalidDeltaTime },
            { TEXT("fresh state after the future one"), 0.0, EUBotSampleStatus::Valid },
            { TEXT("state a hair into the future, inside the tolerance"), -5.0e-7, EUBotSampleStatus::Valid },
        };
        for (const FScriptedSample& Sample : Script)
        {
            AdvanceRest(Scope, Rest, Sample.LagSeconds);
            Rest.Imu->SampleSensor();
            ExpectImuSample(*this, *Rest.Imu, Sample.Name, Sample.Status, true, Sample.LagSeconds, Scope.TimeSeconds());
            if (Sample.Status == EUBotSampleStatus::Valid)
            {
                // Identity orientation and no rotation: specific force = a_B - g, gyro = 0.
                const FUBotImuSensorData Data = Rest.Imu->GetLatestIMUData();
                ExpectNear(*this, FString::Printf(TEXT("%s: specific force"), Sample.Name), Data.LinearAccelerationMetersPerSecondSquared, BodyAcceleration - Gravity, 1.e-9);
                ExpectNear(*this, FString::Printf(TEXT("%s: gyro"), Sample.Name), Data.AngularVelocityRadPerSecond, FVector::ZeroVector, 1.e-12);
            }
        }
        Rest.Rig.Owner->Destroy();
    }

    // 3. A provided body that has no state yet: warm-up, and the history is reset every time.
    {
        FRestRig Rest = MakeRestRig(*World, LeverCentimeters);
        if (!TestNotNull(TEXT("Rest body"), Rest.Rig.Body))
        {
            return false;
        }
        FUBotBodyState& State = Rest.Rig.Source->State;
        State.TimestampSeconds = Scope.TimeSeconds();
        State.bHasLinearAcceleration = true;
        State.bHasAngularAcceleration = true;
        Rest.Rig.Source->bHasState = false;

        StartForManualSampling(*this, *Rest.Imu, TEXT("No state yet"));
        ExpectImuSample(*this, *Rest.Imu, TEXT("No state, start sample"), EUBotSampleStatus::WarmingUp, false, 0.0, Scope.TimeSeconds());

        struct FScriptedSample
        {
            const TCHAR* Name;
            bool bHasState;
            EUBotSampleStatus Status;
        };
        const FScriptedSample Script[] =
        {
            { TEXT("still no state"), false, EUBotSampleStatus::WarmingUp },
            { TEXT("the state appears: first sample after the reset"), true, EUBotSampleStatus::WarmingUp },
            { TEXT("second sample with a state"), true, EUBotSampleStatus::Valid },
            { TEXT("the state disappears"), false, EUBotSampleStatus::WarmingUp },
            { TEXT("the state is back: first sample again"), true, EUBotSampleStatus::WarmingUp },
            { TEXT("and valid again"), true, EUBotSampleStatus::Valid },
        };
        for (const FScriptedSample& Sample : Script)
        {
            Rest.Rig.Source->bHasState = Sample.bHasState;
            AdvanceRest(Scope, Rest, 0.0);
            Rest.Imu->SampleSensor();
            ExpectImuSample(*this, *Rest.Imu, Sample.Name, Sample.Status, Sample.bHasState, 0.0, Scope.TimeSeconds());
        }
        Rest.Rig.Owner->Destroy();
    }

    // 4. A state without accelerations: they are derived from two state times (velocity change over the
    //    time change), held while the state time stays, and the IMU warms up until they exist. The state
    //    has v = (2 t, 0, 0) and w = (0, 0, t / 2): a_B = (2, 0, 0), alpha = (0, 0, 1/2). The IMU is 0.5 m
    //    ahead on X, so specific force = (a_Bx - w^2 r, alpha r, 0) - g and the gyro is (0, 0, w).
    {
        FRestRig Rest = MakeRestRig(*World, LeverCentimeters);
        if (!TestNotNull(TEXT("Rest body"), Rest.Rig.Body))
        {
            return false;
        }
        FUBotBodyState& State = Rest.Rig.Source->State;
        const auto SetStateTime = [&State](double StateTime)
        {
            State.TimestampSeconds = StateTime;
            State.LinearVelocityMetersPerSecond = FVector(2.0 * StateTime, 0.0, 0.0);
            State.AngularVelocityRadiansPerSecond = FVector(0.0, 0.0, 0.5 * StateTime);
        };
        const auto ExpectedForce = [&Gravity, &Lever](double StateTime, double BodyAccelerationX)
        {
            const double Rate = 0.5 * StateTime;
            return FVector(BodyAccelerationX - Rate * Rate * Lever.X, 0.5 * Lever.X, 0.0) - Gravity;
        };

        SetStateTime(Scope.TimeSeconds());
        StartForManualSampling(*this, *Rest.Imu, TEXT("Derived"));
        ExpectImuSample(*this, *Rest.Imu, TEXT("Derived, start sample"), EUBotSampleStatus::WarmingUp, true, 0.0, Scope.TimeSeconds());

        // The state time does not advance: nothing to derive from yet, still warming up (the history is kept).
        Scope.Tick(SlowFrameSeconds);
        Rest.Imu->SampleSensor();
        ExpectImuSample(*this, *Rest.Imu, TEXT("Derived, state time unchanged"), EUBotSampleStatus::WarmingUp, true, Step, Scope.TimeSeconds());

        // The state time advances by two frames: derivable.
        Scope.Tick(SlowFrameSeconds);
        SetStateTime(Scope.TimeSeconds());
        Rest.Imu->SampleSensor();
        ExpectImuSample(*this, *Rest.Imu, TEXT("Derived, state time advanced"), EUBotSampleStatus::Valid, true, 0.0, Scope.TimeSeconds());
        const FUBotImuSensorData Derived = Rest.Imu->GetLatestIMUData();
        ExpectNear(*this, TEXT("Derived: specific force"), Derived.LinearAccelerationMetersPerSecondSquared, ExpectedForce(State.TimestampSeconds, 2.0), 1.e-9);
        ExpectNear(*this, TEXT("Derived: gyro"), Derived.AngularVelocityRadPerSecond, FVector(0.0, 0.0, 0.5 * State.TimestampSeconds), 1.e-12);

        // The state time stays: the derived values are held, and the state's own velocity terms are the same.
        Scope.Tick(SlowFrameSeconds);
        Rest.Imu->SampleSensor();
        ExpectImuSample(*this, *Rest.Imu, TEXT("Derived, state time held"), EUBotSampleStatus::Valid, true, Step, Scope.TimeSeconds());
        const FUBotImuSensorData Held = Rest.Imu->GetLatestIMUData();
        ExpectNear(*this, TEXT("Derived: held specific force"), Held.LinearAccelerationMetersPerSecondSquared, Derived.LinearAccelerationMetersPerSecondSquared, 1.e-12);
        ExpectNear(*this, TEXT("Derived: held gyro"), Held.AngularVelocityRadPerSecond, Derived.AngularVelocityRadPerSecond, 1.e-12);

        // The next state time: derived again from the last two state times.
        Scope.Tick(SlowFrameSeconds);
        SetStateTime(Scope.TimeSeconds());
        Rest.Imu->SampleSensor();
        ExpectImuSample(*this, *Rest.Imu, TEXT("Derived, next state time"), EUBotSampleStatus::Valid, true, 0.0, Scope.TimeSeconds());
        ExpectNear(*this, TEXT("Derived: next specific force"), Rest.Imu->GetLatestIMUData().LinearAccelerationMetersPerSecondSquared, ExpectedForce(State.TimestampSeconds, 2.0), 1.e-9);

        // A state that gives the linear acceleration itself wins over the derived one; the angular
        // acceleration is still derived.
        Scope.Tick(SlowFrameSeconds);
        SetStateTime(Scope.TimeSeconds());
        State.LinearAccelerationMetersPerSecondSquared = FVector(5.0, 0.0, 0.0);
        State.bHasLinearAcceleration = true;
        Rest.Imu->SampleSensor();
        ExpectImuSample(*this, *Rest.Imu, TEXT("Derived, linear acceleration given"), EUBotSampleStatus::Valid, true, 0.0, Scope.TimeSeconds());
        ExpectNear(*this, TEXT("Linear acceleration given: specific force"), Rest.Imu->GetLatestIMUData().LinearAccelerationMetersPerSecondSquared, ExpectedForce(State.TimestampSeconds, 5.0), 1.e-9);
        Rest.Rig.Owner->Destroy();
    }

    // 5. The mode changes between two samples: the history is reset (warm-up again), in both directions.
    {
        FRestRig Rest = MakeRestRig(*World, LeverCentimeters);
        if (!TestNotNull(TEXT("Rest body"), Rest.Rig.Body))
        {
            return false;
        }
        FUBotBodyState& State = Rest.Rig.Source->State;
        State.TimestampSeconds = Scope.TimeSeconds();
        State.LinearAccelerationMetersPerSecondSquared = FVector(1.0, 0.0, 0.0);
        State.bHasLinearAcceleration = true;
        State.bHasAngularAcceleration = true;

        StartForManualSampling(*this, *Rest.Imu, TEXT("Mode switch"));
        ExpectImuSample(*this, *Rest.Imu, TEXT("State mode, start sample"), EUBotSampleStatus::WarmingUp, true, 0.0, Scope.TimeSeconds());

        struct FScriptedSample
        {
            const TCHAR* Name;
            bool bUseBodyState;
            EUBotSampleStatus Status;
        };
        const FScriptedSample Script[] =
        {
            { TEXT("state mode"), true, EUBotSampleStatus::Valid },
            { TEXT("state mode again"), true, EUBotSampleStatus::Valid },
            { TEXT("switched to difference mode: first sample"), false, EUBotSampleStatus::WarmingUp },
            { TEXT("difference mode: second warm-up sample"), false, EUBotSampleStatus::WarmingUp },
            { TEXT("difference mode: valid"), false, EUBotSampleStatus::Valid },
            { TEXT("switched back to state mode: first sample"), true, EUBotSampleStatus::WarmingUp },
            { TEXT("state mode: valid"), true, EUBotSampleStatus::Valid },
        };
        for (const FScriptedSample& Sample : Script)
        {
            Rest.Imu->bUseBodyState = Sample.bUseBodyState;
            AdvanceRest(Scope, Rest, 0.0);
            Rest.Imu->SampleSensor();
            ExpectImuSample(*this, *Rest.Imu, Sample.Name, Sample.Status, Sample.bUseBodyState, 0.0, Scope.TimeSeconds());
            if (Sample.Status == EUBotSampleStatus::Valid)
            {
                // A body at rest: the difference mode reads no acceleration, so the specific force is -g.
                const FVector ExpectedForce = Sample.bUseBodyState ? FVector(1.0, 0.0, 0.0) - Gravity : -Gravity;
                ExpectNear(*this, FString::Printf(TEXT("%s: specific force"), Sample.Name), Rest.Imu->GetLatestIMUData().LinearAccelerationMetersPerSecondSquared, ExpectedForce, 1.e-9);
            }
        }
        Rest.Rig.Owner->Destroy();
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorBodyStateImuNoiseTest, "UBotSensor.BodyState.ImuNoise", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorBodyStateImuNoiseTest::RunTest(const FString& Parameters)
{
    using namespace UBotSensorBodyStateTests;

    FScopedBodyStateWorld Scope;
    UWorld* World = Scope.Get();
    if (!TestNotNull(TEXT("Test world"), World))
    {
        return false;
    }

    const FTrajectory Trajectory{};
    const FMount Mount{};
    const FVector Gravity = GravityMetersPerSecondSquared(*World);

    // Five IMUs on one body and one mount, all with the same noise parameters:
    //  - two in state mode with noise, filter and clamp, same seed: identical outputs;
    //  - one in state mode with noise off (the parameters are set but must not matter): the noiseless reference;
    //  - one in difference mode with noise, filter and clamp, and one in difference mode with noise off, whose
    //    outputs are the true values the noisy one starts from.
    FBodyRig Rig = MakeBodyRig(*World, true);
    if (!TestNotNull(TEXT("Body"), Rig.Body))
    {
        return false;
    }
    const auto AddImu = [&](const TCHAR* Name, bool bUseBodyState, bool bNoise, bool bFilters)
    {
        UUBotImuSensorComponent* Imu = AddSensor<UUBotImuSensorComponent>(Rig, Rig.Body, Name, Mount.OffsetBodyCentimeters(), Mount.Rotation);
        ConfigureImu(*Imu, bUseBodyState, bNoise, bFilters);
        return Imu;
    };
    UUBotImuSensorComponent* NoisyA = AddImu(TEXT("NoisyA"), true, true, true);
    UUBotImuSensorComponent* NoisyB = AddImu(TEXT("NoisyB"), true, true, true);
    UUBotImuSensorComponent* Clean = AddImu(TEXT("Clean"), true, false, false);
    UUBotImuSensorComponent* NoisyDifference = AddImu(TEXT("NoisyDifference"), false, true, true);
    UUBotImuSensorComponent* CleanDifference = AddImu(TEXT("CleanDifference"), false, false, false);
    Rig.Owner->DispatchBeginPlay();

    const double StartTime = Scope.TimeSeconds();
    ApplyTrajectory(Rig, Trajectory, StartTime);
    StartForManualSampling(*this, *NoisyA, TEXT("Noisy A"));
    StartForManualSampling(*this, *NoisyB, TEXT("Noisy B"));
    StartForManualSampling(*this, *Clean, TEXT("Clean"));
    StartForManualSampling(*this, *NoisyDifference, TEXT("Noisy difference"));
    StartForManualSampling(*this, *CleanDifference, TEXT("Clean difference"));

    FImuPostProcessReference ReferenceState(*NoisyA);
    FImuPostProcessReference ReferenceDifference(*NoisyDifference);

    // The reference applies the documented pipeline to the true values: the analytic ones for state mode, the
    // output of the noiseless difference IMU for difference mode. Everything but round-off (about 1e-12) is
    // identical, and any change in the order of the random draws, the interval of the random walk or the
    // filter would move the outputs by the size of the noise (0.01 to 0.05); 1e-8 separates the two.
    // The state-mode truth comes from the closed form, whose difference from the sensor's own evaluation is
    // round-off as well.
    constexpr int32 StepCount = 100;
    int32 StatusErrors = 0;
    double PreviousTime = StartTime;
    double MaxPairDifference = 0.0;
    double MaxStateReferenceError = 0.0;
    double MaxDifferenceReferenceError = 0.0;
    double MaxCleanError = 0.0;
    double MaxNoiseEffect = 0.0;
    double MaxDifferenceNoiseEffect = 0.0;
    for (int32 Step = 1; Step <= StepCount; ++Step)
    {
        Scope.Tick(FrameSeconds);
        const double T = Scope.TimeSeconds();
        const double DeltaSeconds = T - PreviousTime;
        PreviousTime = T;
        ApplyTrajectory(Rig, Trajectory, T);
        NoisyA->SampleSensor();
        NoisyB->SampleSensor();
        Clean->SampleSensor();
        NoisyDifference->SampleSensor();
        CleanDifference->SampleSensor();

        const FUBotImuSensorData A = NoisyA->GetLatestIMUData();
        const FUBotImuSensorData B = NoisyB->GetLatestIMUData();
        const FUBotImuSensorData C = Clean->GetLatestIMUData();
        const FUBotImuSensorData NoisyDiff = NoisyDifference->GetLatestIMUData();
        const FUBotImuSensorData CleanDiff = CleanDifference->GetLatestIMUData();

        const bool bStateValid = A.Header.Status == EUBotSampleStatus::Valid && B.Header.Status == EUBotSampleStatus::Valid && C.Header.Status == EUBotSampleStatus::Valid;
        const bool bDifferenceValid = NoisyDiff.Header.Status == EUBotSampleStatus::Valid && CleanDiff.Header.Status == EUBotSampleStatus::Valid;
        StatusErrors += bStateValid ? 0 : 1;
        StatusErrors += bDifferenceValid == (Step >= 2) ? 0 : 1;

        if (bStateValid)
        {
            const FImuExpectation Expected = ExpectedImu(Trajectory, Mount, T, Gravity);
            MaxPairDifference = FMath::Max(MaxPairDifference, (A.LinearAccelerationMetersPerSecondSquared - B.LinearAccelerationMetersPerSecondSquared).Size());
            MaxPairDifference = FMath::Max(MaxPairDifference, (A.AngularVelocityRadPerSecond - B.AngularVelocityRadPerSecond).Size());
            MaxCleanError = FMath::Max(MaxCleanError, (C.LinearAccelerationMetersPerSecondSquared - Expected.SpecificForce).Size());
            MaxCleanError = FMath::Max(MaxCleanError, (C.AngularVelocityRadPerSecond - Expected.AngularVelocity).Size());
            MaxNoiseEffect = FMath::Max(MaxNoiseEffect, (A.LinearAccelerationMetersPerSecondSquared - C.LinearAccelerationMetersPerSecondSquared).Size());

            FVector ReferenceForce = Expected.SpecificForce;
            FVector ReferenceGyro = Expected.AngularVelocity;
            ReferenceState.Apply(ReferenceForce, ReferenceGyro, DeltaSeconds);
            MaxStateReferenceError = FMath::Max(MaxStateReferenceError, (A.LinearAccelerationMetersPerSecondSquared - ReferenceForce).Size());
            MaxStateReferenceError = FMath::Max(MaxStateReferenceError, (A.AngularVelocityRadPerSecond - ReferenceGyro).Size());
        }
        if (bDifferenceValid)
        {
            FVector ReferenceForce = CleanDiff.LinearAccelerationMetersPerSecondSquared;
            FVector ReferenceGyro = CleanDiff.AngularVelocityRadPerSecond;
            ReferenceDifference.Apply(ReferenceForce, ReferenceGyro, DeltaSeconds);
            MaxDifferenceReferenceError = FMath::Max(MaxDifferenceReferenceError, (NoisyDiff.LinearAccelerationMetersPerSecondSquared - ReferenceForce).Size());
            MaxDifferenceReferenceError = FMath::Max(MaxDifferenceReferenceError, (NoisyDiff.AngularVelocityRadPerSecond - ReferenceGyro).Size());
            MaxDifferenceNoiseEffect = FMath::Max(MaxDifferenceNoiseEffect, (NoisyDiff.LinearAccelerationMetersPerSecondSquared - CleanDiff.LinearAccelerationMetersPerSecondSquared).Size());
        }
    }

    TestEqual(TEXT("Every sample has the expected status"), StatusErrors, 0);
    TestTrue(FString::Printf(TEXT("Same seed and same state sequence: identical outputs (max difference %g)"), MaxPairDifference), MaxPairDifference <= 1.e-12);
    TestTrue(FString::Printf(TEXT("Noise off: the outputs are the noiseless reference, whatever the noise parameters (max error %g)"), MaxCleanError), MaxCleanError <= 1.e-6);
    TestTrue(FString::Printf(TEXT("Noise on actually disturbs the outputs (largest effect %g m/s^2)"), MaxNoiseEffect), MaxNoiseEffect > 1.e-3);
    TestTrue(FString::Printf(TEXT("State mode: noise, filter and clamp follow the documented pipeline (max error %g)"), MaxStateReferenceError), MaxStateReferenceError <= 1.e-8);
    TestTrue(FString::Printf(TEXT("Difference mode: noise, filter and clamp follow the documented pipeline (max error %g)"), MaxDifferenceReferenceError), MaxDifferenceReferenceError <= 1.e-8);
    TestTrue(FString::Printf(TEXT("Difference mode: noise actually disturbs the outputs (largest effect %g m/s^2)"), MaxDifferenceNoiseEffect), MaxDifferenceNoiseEffect > 1.e-3);

    // The variances reported with a sample come from the noise parameters in every mode and with noise off.
    const FUBotImuSensorData CleanData = Clean->GetLatestIMUData();
    TestTrue(TEXT("The accelerometer variance is the noise variance"), FMath::IsNearlyEqual(CleanData.LinearAccelerationVariance.X, 0.05 * 0.05, 1.e-9));
    TestTrue(TEXT("The gyroscope variance is the noise variance"), FMath::IsNearlyEqual(CleanData.AngularVelocityVariance.X, 0.01 * 0.01, 1.e-9));

    Rig.Owner->Destroy();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorBodyStateOdometryTest, "UBotSensor.BodyState.Odometry", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorBodyStateOdometryTest::RunTest(const FString& Parameters)
{
    using namespace UBotSensorBodyStateTests;

    FScopedBodyStateWorld Scope;
    UWorld* World = Scope.Get();
    if (!TestNotNull(TEXT("Test world"), World))
    {
        return false;
    }

    const FTrajectory Trajectory{};
    const FMount Mount{};
    const double Step = static_cast<double>(SlowFrameSeconds);

    // The tracked component is a child of the provided body, off its origin on a rotated mount. Two odometry
    // sensors track it: one reads the state, one differentiates poses.
    FBodyRig Rig = MakeBodyRig(*World, true);
    if (!TestNotNull(TEXT("Body"), Rig.Body))
    {
        return false;
    }
    USceneComponent* Tracked = NewObject<USceneComponent>(Rig.Owner, TEXT("Tracked"));
    Tracked->SetupAttachment(Rig.Body);
    Tracked->RegisterComponent();
    Tracked->SetRelativeLocationAndRotation(Mount.OffsetBodyCentimeters(), Mount.Rotation);
    UUBotOdometrySensorComponent* StateOdometry = AddSensor<UUBotOdometrySensorComponent>(Rig, Rig.Body, TEXT("StateOdometry"), FVector::ZeroVector, FQuat::Identity);
    UUBotOdometrySensorComponent* DifferenceOdometry = AddSensor<UUBotOdometrySensorComponent>(Rig, Rig.Body, TEXT("DifferenceOdometry"), FVector::ZeroVector, FQuat::Identity);
    StateOdometry->CustomTrackedComponent.OverrideComponent = Tracked;
    DifferenceOdometry->CustomTrackedComponent.OverrideComponent = Tracked;
    DifferenceOdometry->bUseBodyState = false;
    Rig.Owner->DispatchBeginPlay();
    TestTrue(TEXT("The sensor tracks the offset component"), StateOdometry->GetTrackedComponent() == Tracked);

    const double StartTime = Scope.TimeSeconds();
    ApplyTrajectory(Rig, Trajectory, StartTime);
    FTransform OriginPose = Tracked->GetComponentTransform();
    StartForManualSampling(*this, *StateOdometry, TEXT("State odometry"));
    StartForManualSampling(*this, *DifferenceOdometry, TEXT("Difference odometry"));
    ExpectOdometrySample(*this, *StateOdometry, TEXT("State odometry, start sample"), EUBotSampleStatus::WarmingUp, true, 0.0, StartTime);
    ExpectOdometrySample(*this, *DifferenceOdometry, TEXT("Difference odometry, start sample"), EUBotSampleStatus::WarmingUp, false, 0.0, StartTime);

    // Position of the tracked component relative to the pose the odometry started from, in meters.
    const auto ExpectedPosition = [&Tracked, &OriginPose]()
    {
        return UBot::Units::CentimetersToMeters(Tracked->GetComponentTransform().GetRelativeTransform(OriginPose).GetLocation());
    };

    // 1. Moving: valid from the second sample (the first is the warm-up that sets the previous sample).
    //    State velocities are P.Rot^-1 * (v + w x r) and P.Rot^-1 * w; the pose is the difference mode's.
    //    Difference mode differs from them by the finite-difference error of one interval: the velocity
    //    is the exact rate at t - dt/2, so by dt/2 |y''| (|y''| <= |A0| + |C| w^2 + (|alpha| + rate^2) |r|),
    //    the angular velocity by |alpha| dt/2; 25 % for the O(dt^2) terms and 2e-3 m/s for the rest.
    constexpr int32 MovingSteps = 25;
    const double MaxRate = Trajectory.ThetaRate(MovingSteps * Step);
    const double AccelerationBound = Trajectory.A0.Size() + Trajectory.SineAmplitude.Size() * FMath::Square(Trajectory.SineRadPerSecond)
        + (FMath::Abs(Trajectory.AngularAcceleration) + FMath::Square(MaxRate)) * Mount.OffsetBodyMeters.Size();
    const double DifferenceLinearTolerance = 1.25 * 0.5 * Step * AccelerationBound + 2.e-3;
    const double DifferenceAngularTolerance = 1.25 * 0.5 * FMath::Abs(Trajectory.AngularAcceleration) * Step + 1.e-6;
    int32 StatusErrors = 0;
    int32 VelocityFlagErrors = 0;
    double MaxStateLinearError = 0.0;
    double MaxStateAngularError = 0.0;
    double MaxPoseDifference = 0.0;
    double MaxDifferenceLinearError = 0.0;
    double MaxDifferenceAngularError = 0.0;
    for (int32 Index = 1; Index <= MovingSteps; ++Index)
    {
        Scope.Tick(SlowFrameSeconds);
        const double T = Scope.TimeSeconds();
        ApplyTrajectory(Rig, Trajectory, T);
        StateOdometry->SampleSensor();
        DifferenceOdometry->SampleSensor();

        const FUBotOdometrySensorData State = StateOdometry->GetLatestOdometryData();
        const FUBotOdometrySensorData Difference = DifferenceOdometry->GetLatestOdometryData();
        StatusErrors += (State.Header.Status == EUBotSampleStatus::Valid && Difference.Header.Status == EUBotSampleStatus::Valid) ? 0 : 1;
        VelocityFlagErrors += (State.bHasVelocity && Difference.bHasVelocity) ? 0 : 1;

        const FQuat TrackedRotation = Trajectory.Orientation(T) * Mount.Rotation;
        const FVector ExpectedLinear = TrackedRotation.UnrotateVector(Trajectory.PointVelocity(T, Mount.OffsetBodyMeters));
        const FVector ExpectedAngular = TrackedRotation.UnrotateVector(Trajectory.AngularVelocity(T));
        MaxStateLinearError = FMath::Max(MaxStateLinearError, (State.LinearVelocityLocalMetersPerSecond - ExpectedLinear).Size());
        MaxStateAngularError = FMath::Max(MaxStateAngularError, (State.AngularVelocityLocalRadPerSecond - ExpectedAngular).Size());
        MaxPoseDifference = FMath::Max(MaxPoseDifference, (State.PositionMeters - Difference.PositionMeters).Size());
        MaxPoseDifference = FMath::Max(MaxPoseDifference, RotationError(State.OrientationQuaternion, Difference.OrientationQuaternion));
        MaxDifferenceLinearError = FMath::Max(MaxDifferenceLinearError, (Difference.LinearVelocityLocalMetersPerSecond - ExpectedLinear).Size());
        MaxDifferenceAngularError = FMath::Max(MaxDifferenceAngularError, (Difference.AngularVelocityLocalRadPerSecond - ExpectedAngular).Size());
    }
    TestEqual(TEXT("Both modes are valid from the second sample on"), StatusErrors, 0);
    TestEqual(TEXT("Both modes report velocity"), VelocityFlagErrors, 0);
    TestTrue(FString::Printf(TEXT("State mode: linear velocity is P.Rot^-1 (v + w x r) (max error %g)"), MaxStateLinearError), MaxStateLinearError <= 1.e-6);
    TestTrue(FString::Printf(TEXT("State mode: angular velocity is P.Rot^-1 w (max error %g)"), MaxStateAngularError), MaxStateAngularError <= 1.e-6);
    TestTrue(FString::Printf(TEXT("The pose is identical to difference mode (max difference %g)"), MaxPoseDifference), MaxPoseDifference <= 1.e-12);
    TestTrue(FString::Printf(TEXT("Difference mode agrees within the finite-difference error: linear (max error %g, tolerance %g)"), MaxDifferenceLinearError, DifferenceLinearTolerance), MaxDifferenceLinearError <= DifferenceLinearTolerance);
    TestTrue(FString::Printf(TEXT("Difference mode agrees within the finite-difference error: angular (max error %g, tolerance %g)"), MaxDifferenceAngularError, DifferenceAngularTolerance), MaxDifferenceAngularError <= DifferenceAngularTolerance);
    const double DistanceFromOrigin = StateOdometry->GetLatestOdometryData().PositionMeters.Size();
    TestTrue(FString::Printf(TEXT("The body has moved away from the origin (%g m)"), DistanceFromOrigin), DistanceFromOrigin > 0.2);
    DifferenceOdometry->StopSensor();

    // 2. A provided body without a state: warm-up (the previous sample starts over, the origin stays). The
    //    first sample with a state again is a mode change and warms up once more.
    Rig.Source->bHasState = false;
    Scope.Tick(SlowFrameSeconds);
    ApplyTrajectory(Rig, Trajectory, Scope.TimeSeconds());
    StateOdometry->SampleSensor();
    ExpectOdometrySample(*this, *StateOdometry, TEXT("No state"), EUBotSampleStatus::WarmingUp, false, 0.0, Scope.TimeSeconds());
    ExpectNear(*this, TEXT("No state: the pose is still relative to the same origin"), StateOdometry->GetLatestOdometryData().PositionMeters, ExpectedPosition(), 1.e-9);

    Rig.Source->bHasState = true;
    Scope.Tick(SlowFrameSeconds);
    ApplyTrajectory(Rig, Trajectory, Scope.TimeSeconds());
    StateOdometry->SampleSensor();
    ExpectOdometrySample(*this, *StateOdometry, TEXT("The state is back: first sample"), EUBotSampleStatus::WarmingUp, true, 0.0, Scope.TimeSeconds());
    Scope.Tick(SlowFrameSeconds);
    ApplyTrajectory(Rig, Trajectory, Scope.TimeSeconds());
    StateOdometry->SampleSensor();
    ExpectOdometrySample(*this, *StateOdometry, TEXT("The state is back: second sample"), EUBotSampleStatus::Valid, true, 0.0, Scope.TimeSeconds());
    ExpectNear(*this, TEXT("After the warm-up the pose is still relative to the same origin"), StateOdometry->GetLatestOdometryData().PositionMeters, ExpectedPosition(), 1.e-9);

    // 3. The age check: a state older than MaximumValidDeltaSeconds, or from the future, ends in
    //    InvalidDeltaTime and starts the odometry over at the current pose (new origin, as an invalid
    //    interval does). The next fresh state is valid, and the pose is relative to that new origin.
    struct FAgeCase
    {
        const TCHAR* Name;
        double LagSeconds;
    };
    const FAgeCase AgeCases[] =
    {
        { TEXT("Stale state"), 0.6 },
        { TEXT("State from the future"), -0.01 },
    };
    for (const FAgeCase& Case : AgeCases)
    {
        Scope.Tick(SlowFrameSeconds);
        ApplyTrajectory(Rig, Trajectory, Scope.TimeSeconds());
        Rig.Source->State.TimestampSeconds = Scope.TimeSeconds() - Case.LagSeconds;
        OriginPose = Tracked->GetComponentTransform();
        StateOdometry->SampleSensor();
        ExpectOdometrySample(*this, *StateOdometry, Case.Name, EUBotSampleStatus::InvalidDeltaTime, true, Case.LagSeconds, Scope.TimeSeconds());
        TestTrue(FString::Printf(TEXT("%s: the invalid sample carries no pose"), Case.Name), StateOdometry->GetLatestOdometryData().PositionMeters.IsNearlyZero(1.e-12) && !StateOdometry->GetLatestOdometryData().bHasVelocity);

        Scope.Tick(SlowFrameSeconds);
        ApplyTrajectory(Rig, Trajectory, Scope.TimeSeconds());
        StateOdometry->SampleSensor();
        ExpectOdometrySample(*this, *StateOdometry, *FString::Printf(TEXT("Fresh state after the %s"), Case.Name), EUBotSampleStatus::Valid, true, 0.0, Scope.TimeSeconds());
        const FVector Position = StateOdometry->GetLatestOdometryData().PositionMeters;
        ExpectNear(*this, FString::Printf(TEXT("%s: the pose is relative to the new origin"), Case.Name), Position, ExpectedPosition(), 1.e-9);
        TestTrue(FString::Printf(TEXT("%s: the new origin is close (%g m)"), Case.Name, Position.Size()), Position.Size() < 0.1);
    }

    // 4. The mode changes between two samples: the previous sample starts over without moving the origin.
    struct FModeCase
    {
        const TCHAR* Name;
        bool bUseBodyState;
        EUBotSampleStatus Status;
    };
    const FModeCase ModeCases[] =
    {
        { TEXT("Switched to difference mode"), false, EUBotSampleStatus::WarmingUp },
        { TEXT("Difference mode"), false, EUBotSampleStatus::Valid },
        { TEXT("Switched back to state mode"), true, EUBotSampleStatus::WarmingUp },
        { TEXT("State mode"), true, EUBotSampleStatus::Valid },
    };
    for (const FModeCase& Case : ModeCases)
    {
        StateOdometry->bUseBodyState = Case.bUseBodyState;
        Scope.Tick(SlowFrameSeconds);
        ApplyTrajectory(Rig, Trajectory, Scope.TimeSeconds());
        StateOdometry->SampleSensor();
        ExpectOdometrySample(*this, *StateOdometry, Case.Name, Case.Status, Case.bUseBodyState, 0.0, Scope.TimeSeconds());
        ExpectNear(*this, FString::Printf(TEXT("%s: the origin did not move"), Case.Name), StateOdometry->GetLatestOdometryData().PositionMeters, ExpectedPosition(), 1.e-9);
    }

    // ResetOdometry starts everything over, also the state-mode history: the next sample is a warm-up.
    StateOdometry->ResetOdometry();
    Scope.Tick(SlowFrameSeconds);
    ApplyTrajectory(Rig, Trajectory, Scope.TimeSeconds());
    StateOdometry->SampleSensor();
    ExpectOdometrySample(*this, *StateOdometry, TEXT("After ResetOdometry"), EUBotSampleStatus::WarmingUp, true, 0.0, Scope.TimeSeconds());

    Rig.Owner->Destroy();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotSensorBodyStatePoseTest, "UBotSensor.BodyState.Pose", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotSensorBodyStatePoseTest::RunTest(const FString& Parameters)
{
    using namespace UBotSensorBodyStateTests;

    FScopedBodyStateWorld Scope;
    UWorld* World = Scope.Get();
    if (!TestNotNull(TEXT("Test world"), World))
    {
        return false;
    }

    const FTrajectory Trajectory{};
    const FMount Mount{};

    // Three pose sensors on the same mount of a provided body: the World frame (reads the state), the Parent
    // frame (the body) and a Custom Component frame (the root of another actor, fixed in the world). Only the
    // World frame may use the state.
    FBodyRig Rig = MakeBodyRig(*World, true);
    AActor* ReferenceActor = World->SpawnActor<AActor>();
    if (!TestNotNull(TEXT("Body"), Rig.Body) || !TestNotNull(TEXT("Reference actor"), ReferenceActor))
    {
        return false;
    }
    USceneComponent* ReferenceRoot = NewObject<USceneComponent>(ReferenceActor, TEXT("ReferenceRoot"));
    ReferenceActor->SetRootComponent(ReferenceRoot);
    ReferenceRoot->RegisterComponent();
    ReferenceRoot->SetWorldLocationAndRotation(FVector(100.0, 50.0, 0.0), FRotator(0.0, 30.0, 0.0).Quaternion());

    UUBotPoseSensorComponent* PoseWorld = AddSensor<UUBotPoseSensorComponent>(Rig, Rig.Body, TEXT("PoseWorld"), Mount.OffsetBodyCentimeters(), Mount.Rotation);
    UUBotPoseSensorComponent* PoseParent = AddSensor<UUBotPoseSensorComponent>(Rig, Rig.Body, TEXT("PoseParent"), Mount.OffsetBodyCentimeters(), Mount.Rotation);
    UUBotPoseSensorComponent* PoseCustom = AddSensor<UUBotPoseSensorComponent>(Rig, Rig.Body, TEXT("PoseCustom"), Mount.OffsetBodyCentimeters(), Mount.Rotation);
    PoseParent->ReferenceFrame = EUBotPoseReferenceFrame::Parent;
    PoseCustom->ReferenceFrame = EUBotPoseReferenceFrame::CustomComponent;
    PoseCustom->CustomReferenceComponent.OverrideComponent = ReferenceRoot;
    Rig.Owner->DispatchBeginPlay();
    ReferenceActor->DispatchBeginPlay();

    // Velocities and the state-mode flags of a pose sample, against the trajectory.
    const auto ExpectWorldSample = [this, &Trajectory, &Mount](const UUBotPoseSensorComponent& Pose, const TCHAR* Context, double T, double ExpectedAgeSeconds)
    {
        const FUBotPoseSensorData Data = Pose.GetLatestPoseData();
        TestTrue(FString::Printf(TEXT("%s: valid sample"), Context), Data.Header.bValid && Data.Header.Status == EUBotSampleStatus::Valid);
        TestTrue(FString::Printf(TEXT("%s: velocity from the first sample on"), Context), Data.bHasVelocity);
        TestTrue(FString::Printf(TEXT("%s: IsUsingBodyState"), Context), Pose.IsUsingBodyState());
        TestTrue(FString::Printf(TEXT("%s: body state age is %g (got %g)"), Context, ExpectedAgeSeconds, Pose.GetBodyStateAgeSeconds()), FMath::IsNearlyEqual(Pose.GetBodyStateAgeSeconds(), ExpectedAgeSeconds, 1.e-9));
        TestTrue(FString::Printf(TEXT("%s: the header carries the sample time"), Context), FMath::IsNearlyEqual(Data.Header.TimestampSeconds, T, 1.e-12));
        ExpectNear(*this, FString::Printf(TEXT("%s: position (from the component)"), Context), Data.PositionMeters, Trajectory.PointPosition(T, Mount.OffsetBodyMeters), 1.e-9);
        TestTrue(FString::Printf(TEXT("%s: orientation (from the component)"), Context), RotationError(Data.OrientationQuaternion, Trajectory.Orientation(T) * Mount.Rotation) <= 1.e-9);
        ExpectNear(*this, FString::Printf(TEXT("%s: linear velocity v + w x r"), Context), Data.LinearVelocityMetersPerSecond, Trajectory.PointVelocity(T, Mount.OffsetBodyMeters), 1.e-6);
        ExpectNear(*this, FString::Printf(TEXT("%s: angular velocity w"), Context), Data.AngularVelocityRadPerSecond, Trajectory.AngularVelocity(T), 1.e-6);
    };

    const double StartTime = Scope.TimeSeconds();
    ApplyTrajectory(Rig, Trajectory, StartTime);
    StartForManualSampling(*this, *PoseWorld, TEXT("World frame"));
    StartForManualSampling(*this, *PoseParent, TEXT("Parent frame"));
    StartForManualSampling(*this, *PoseCustom, TEXT("Custom frame"));

    // 1. Lever arm velocity from the very first sample in the World frame; the Parent and Custom Component
    //    frames differentiate the relative pose, so their first sample has no velocity and IsUsingBodyState
    //    stays false even though a source is present.
    ExpectWorldSample(*PoseWorld, TEXT("World frame, start sample"), StartTime, 0.0);
    TestFalse(TEXT("Parent frame: no velocity on the first sample"), PoseParent->GetLatestPoseData().bHasVelocity);
    TestFalse(TEXT("Custom frame: no velocity on the first sample"), PoseCustom->GetLatestPoseData().bHasVelocity);
    TestFalse(TEXT("Parent frame does not use the state"), PoseParent->IsUsingBodyState());
    TestFalse(TEXT("Custom frame does not use the state"), PoseCustom->IsUsingBodyState());

    FUBotPoseSensorData PreviousCustom = PoseCustom->GetLatestPoseData();
    for (int32 Index = 1; Index <= 3; ++Index)
    {
        Scope.Tick(SlowFrameSeconds);
        const double T = Scope.TimeSeconds();
        ApplyTrajectory(Rig, Trajectory, T);
        PoseWorld->SampleSensor();
        PoseParent->SampleSensor();
        PoseCustom->SampleSensor();
        const FString Context = FString::Printf(TEXT("World frame, sample %d"), Index + 1);
        ExpectWorldSample(*PoseWorld, *Context, T, 0.0);

        // The sensor is fixed in the body frame, so the relative pose does not change: about zero velocity (the
        // angular one is the acos round-off of two equal quaternions over dt, below 1e-5 rad/s), whereas the
        // state would give about one meter per second.
        const FUBotPoseSensorData Parent = PoseParent->GetLatestPoseData();
        TestTrue(FString::Printf(TEXT("Parent frame, sample %d: velocity by differences"), Index + 1), Parent.bHasVelocity && Parent.Header.Status == EUBotSampleStatus::Valid);
        TestTrue(FString::Printf(TEXT("Parent frame, sample %d: the relative pose is at rest (v %g m/s, w %g rad/s)"), Index + 1, Parent.LinearVelocityMetersPerSecond.Size(), Parent.AngularVelocityRadPerSecond.Size()), Parent.LinearVelocityMetersPerSecond.Size() < 1.e-6 && Parent.AngularVelocityRadPerSecond.Size() < 1.e-4);
        TestFalse(FString::Printf(TEXT("Parent frame, sample %d: IsUsingBodyState"), Index + 1), PoseParent->IsUsingBodyState());
        TestTrue(FString::Printf(TEXT("Parent frame, sample %d: no body state age"), Index + 1), PoseParent->GetBodyStateAgeSeconds() == 0.0);

        const FUBotPoseSensorData Custom = PoseCustom->GetLatestPoseData();
        const double Interval = Custom.Header.TimestampSeconds - PreviousCustom.Header.TimestampSeconds;
        TestTrue(FString::Printf(TEXT("Custom frame, sample %d: velocity by differences"), Index + 1), Custom.bHasVelocity && Custom.Header.Status == EUBotSampleStatus::Valid);
        ExpectNear(*this, FString::Printf(TEXT("Custom frame, sample %d: velocity is the difference of consecutive poses"), Index + 1), Custom.LinearVelocityMetersPerSecond, (Custom.PositionMeters - PreviousCustom.PositionMeters) / Interval, 1.e-9);
        TestTrue(FString::Printf(TEXT("Custom frame, sample %d: the body moves in the reference frame"), Index + 1), Custom.LinearVelocityMetersPerSecond.Size() > 0.1);
        TestFalse(FString::Printf(TEXT("Custom frame, sample %d: IsUsingBodyState"), Index + 1), PoseCustom->IsUsingBodyState());
        PreviousCustom = Custom;
    }

    // 2. The Pose sensor does not check the age of the state: a state five seconds old still gives the velocity.
    Scope.Tick(SlowFrameSeconds);
    ApplyTrajectory(Rig, Trajectory, Scope.TimeSeconds());
    Rig.Source->State.TimestampSeconds = Scope.TimeSeconds() - 5.0;
    PoseWorld->SampleSensor();
    ExpectWorldSample(*PoseWorld, TEXT("World frame, old state"), Scope.TimeSeconds(), 5.0);

    // 3. A provided body without a state: a valid pose without velocity (no fallback to differences).
    Scope.Tick(SlowFrameSeconds);
    ApplyTrajectory(Rig, Trajectory, Scope.TimeSeconds());
    Rig.Source->bHasState = false;
    PoseWorld->SampleSensor();
    {
        const FUBotPoseSensorData Data = PoseWorld->GetLatestPoseData();
        TestTrue(TEXT("No state: the pose sample is valid"), Data.Header.bValid && Data.Header.Status == EUBotSampleStatus::Valid);
        TestFalse(TEXT("No state: no velocity"), Data.bHasVelocity);
        TestTrue(TEXT("No state: the velocities are zero"), Data.LinearVelocityMetersPerSecond.IsZero() && Data.AngularVelocityRadPerSecond.IsZero());
        TestFalse(TEXT("No state: IsUsingBodyState"), PoseWorld->IsUsingBodyState());
        TestTrue(TEXT("No state: no body state age"), PoseWorld->GetBodyStateAgeSeconds() == 0.0);
        ExpectNear(*this, TEXT("No state: the pose is still reported"), Data.PositionMeters, Trajectory.PointPosition(Scope.TimeSeconds(), Mount.OffsetBodyMeters), 1.e-9);
    }
    Rig.Source->bHasState = true;

    // 4. bUseBodyState off with a source present: difference mode. The previous pose is recorded in every
    //    mode, so the velocity is there on the first sample.
    PoseWorld->bUseBodyState = false;
    FUBotPoseSensorData PreviousWorld = PoseWorld->GetLatestPoseData();
    Scope.Tick(SlowFrameSeconds);
    ApplyTrajectory(Rig, Trajectory, Scope.TimeSeconds());
    PoseWorld->SampleSensor();
    {
        const FUBotPoseSensorData Data = PoseWorld->GetLatestPoseData();
        const double Interval = Data.Header.TimestampSeconds - PreviousWorld.Header.TimestampSeconds;
        TestTrue(TEXT("bUseBodyState off: velocity by differences"), Data.bHasVelocity);
        TestFalse(TEXT("bUseBodyState off: IsUsingBodyState"), PoseWorld->IsUsingBodyState());
        ExpectNear(*this, TEXT("bUseBodyState off: velocity is the difference of consecutive poses"), Data.LinearVelocityMetersPerSecond, (Data.PositionMeters - PreviousWorld.PositionMeters) / Interval, 1.e-9);
    }

    // 5. bIncludeVelocity off: no velocity in either mode, but the mode itself is still reported.
    PoseWorld->bUseBodyState = true;
    PoseWorld->bIncludeVelocity = false;
    Scope.Tick(SlowFrameSeconds);
    ApplyTrajectory(Rig, Trajectory, Scope.TimeSeconds());
    PoseWorld->SampleSensor();
    {
        const FUBotPoseSensorData Data = PoseWorld->GetLatestPoseData();
        TestFalse(TEXT("bIncludeVelocity off: no velocity"), Data.bHasVelocity);
        TestTrue(TEXT("bIncludeVelocity off: the velocities are zero"), Data.LinearVelocityMetersPerSecond.IsZero() && Data.AngularVelocityRadPerSecond.IsZero());
        TestTrue(TEXT("bIncludeVelocity off: the sensor is in state mode"), PoseWorld->IsUsingBodyState());
    }

    Rig.Owner->Destroy();
    ReferenceActor->Destroy();
    return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
