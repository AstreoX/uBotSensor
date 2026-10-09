#include "UBotSensorMedium.h"

#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "UBotPerceptionMedium.h"
#include "UObject/UnrealType.h"

namespace UBot::SensorMedium::Private
{
    const FName SmokeVolumeTagName(TEXT("AgentSensor.SmokeVolume"));
    const FName IsSmokeFieldActiveFunctionName(TEXT("IsSmokeFieldActive"));
    const FName GetSmokeCellSnapshotsFunctionName(TEXT("GetSmokeCellSnapshots"));
    const FName CellVisualRadiusMetersPropertyName(TEXT("CellVisualRadiusMeters"));
    const FName CellSizeMetersPropertyName(TEXT("CellSizeMeters"));
    const FName WorldPositionPropertyName(TEXT("WorldPosition"));
    const FName DensityPropertyName(TEXT("Density"));
    const FName EdgeFactorPropertyName(TEXT("EdgeFactor"));

    void InitializeParams(const UFunction& Function, void* Params)
    {
        for (TFieldIterator<FProperty> PropertyIt(&Function); PropertyIt; ++PropertyIt)
        {
            FProperty* Property = *PropertyIt;
            if (Property->HasAnyPropertyFlags(CPF_Parm))
            {
                Property->InitializeValue_InContainer(Params);
            }
        }
    }

    void DestroyParams(const UFunction& Function, void* Params)
    {
        for (TFieldIterator<FProperty> PropertyIt(&Function); PropertyIt; ++PropertyIt)
        {
            FProperty* Property = *PropertyIt;
            if (Property->HasAnyPropertyFlags(CPF_Parm))
            {
                Property->DestroyValue_InContainer(Params);
            }
        }
    }

    bool InvokeBoolNoParam(AActor& Actor, FName FunctionName, bool bDefaultValue)
    {
        UFunction* Function = Actor.FindFunction(FunctionName);
        if (!Function)
        {
            return bDefaultValue;
        }

        TArray<uint8> Params;
        Params.SetNumZeroed(Function->ParmsSize);
        InitializeParams(*Function, Params.GetData());
        Actor.ProcessEvent(Function, Params.GetData());

        bool bValue = bDefaultValue;
        for (TFieldIterator<FProperty> PropertyIt(Function); PropertyIt; ++PropertyIt)
        {
            if (FBoolProperty* BoolProperty = CastField<FBoolProperty>(*PropertyIt))
            {
                if (BoolProperty->HasAnyPropertyFlags(CPF_ReturnParm))
                {
                    bValue = BoolProperty->GetPropertyValue_InContainer(Params.GetData());
                    break;
                }
            }
        }

        DestroyParams(*Function, Params.GetData());
        return bValue;
    }

    bool ReadFloatProperty(const void* Container, const UStruct& Struct, FName PropertyName, float& OutValue)
    {
        const FProperty* Property = Struct.FindPropertyByName(PropertyName);
        if (const FFloatProperty* FloatProperty = CastField<FFloatProperty>(Property))
        {
            OutValue = FloatProperty->GetPropertyValue_InContainer(Container);
            return true;
        }
        if (const FDoubleProperty* DoubleProperty = CastField<FDoubleProperty>(Property))
        {
            OutValue = static_cast<float>(DoubleProperty->GetPropertyValue_InContainer(Container));
            return true;
        }
        if (const FNumericProperty* NumericProperty = CastField<FNumericProperty>(Property))
        {
            if (NumericProperty->IsFloatingPoint())
            {
                OutValue = static_cast<float>(NumericProperty->GetFloatingPointPropertyValue(Property->ContainerPtrToValuePtr<void>(Container)));
                return true;
            }
        }
        return false;
    }

    bool ReadVectorProperty(const void* Container, const UStruct& Struct, FName PropertyName, FVector& OutValue)
    {
        const FStructProperty* StructProperty = FindFProperty<FStructProperty>(&Struct, PropertyName);
        if (!StructProperty || StructProperty->Struct != TBaseStructure<FVector>::Get())
        {
            return false;
        }
        OutValue = *StructProperty->ContainerPtrToValuePtr<FVector>(Container);
        return true;
    }

    float RadiusScale(const FQuerySettings& Settings)
    {
        return FMath::Max(0.01f, Settings.CellRadiusScale);
    }

    float ReadSmokeCellRadiusCentimeters(const AActor& Actor, const FQuerySettings& Settings)
    {
        float RadiusMeters = 0.0f;
        if (!ReadFloatProperty(&Actor, *Actor.GetClass(), CellVisualRadiusMetersPropertyName, RadiusMeters)
            || RadiusMeters <= 0.0f)
        {
            if (!ReadFloatProperty(&Actor, *Actor.GetClass(), CellSizeMetersPropertyName, RadiusMeters)
                || RadiusMeters <= 0.0f)
            {
                return FMath::Max(1.0f, Settings.DefaultCellRadiusCentimeters * RadiusScale(Settings));
            }
        }
        return FMath::Max(1.0f, RadiusMeters * 100.0f * RadiusScale(Settings));
    }

    // Legacy reflection path: copies the cells returned by GetSmokeCellSnapshots(), keeping only the
    // cells at or above the density threshold. Returns false when the actor has no usable function.
    bool CopySmokeCells(AActor& Actor, const FQuerySettings& Settings, TArray<FUBotMediumCell>& OutCells)
    {
        UFunction* Function = Actor.FindFunction(GetSmokeCellSnapshotsFunctionName);
        if (!Function)
        {
            return false;
        }

        TArray<uint8> Params;
        Params.SetNumZeroed(Function->ParmsSize);
        InitializeParams(*Function, Params.GetData());
        Actor.ProcessEvent(Function, Params.GetData());

        const FArrayProperty* ReturnArrayProperty = nullptr;
        for (TFieldIterator<FProperty> PropertyIt(Function); PropertyIt; ++PropertyIt)
        {
            if (const FArrayProperty* ArrayProperty = CastField<FArrayProperty>(*PropertyIt))
            {
                if (ArrayProperty->HasAnyPropertyFlags(CPF_ReturnParm))
                {
                    ReturnArrayProperty = ArrayProperty;
                    break;
                }
            }
        }

        if (!ReturnArrayProperty)
        {
            DestroyParams(*Function, Params.GetData());
            return false;
        }

        const FStructProperty* InnerStructProperty = CastField<FStructProperty>(ReturnArrayProperty->Inner);
        if (!InnerStructProperty)
        {
            DestroyParams(*Function, Params.GetData());
            return false;
        }

        const float CellRadiusCentimeters = ReadSmokeCellRadiusCentimeters(Actor, Settings);
        const void* ArrayAddress = ReturnArrayProperty->ContainerPtrToValuePtr<void>(Params.GetData());
        FScriptArrayHelper ArrayHelper(ReturnArrayProperty, ArrayAddress);
        OutCells.Reserve(OutCells.Num() + ArrayHelper.Num());
        for (int32 Index = 0; Index < ArrayHelper.Num(); ++Index)
        {
            const void* CellData = ArrayHelper.GetRawPtr(Index);
            FUBotMediumCell Cell;
            Cell.RadiusCentimeters = CellRadiusCentimeters;
            if (!ReadVectorProperty(CellData, *InnerStructProperty->Struct, WorldPositionPropertyName, Cell.WorldPositionCentimeters)
                || !ReadFloatProperty(CellData, *InnerStructProperty->Struct, DensityPropertyName, Cell.Density))
            {
                continue;
            }
            float EdgeFactor = 1.0f;
            if (ReadFloatProperty(CellData, *InnerStructProperty->Struct, EdgeFactorPropertyName, EdgeFactor))
            {
                Cell.EdgeFactor = EdgeFactor;
            }
            if (UBot::Medium::EffectiveDensity(Cell) >= Settings.DensityThreshold)
            {
                OutCells.Add(Cell);
            }
        }

        DestroyParams(*Function, Params.GetData());
        return true;
    }

    // Core interface path: applies the same sensor-side settings the legacy path applies while copying
    // (radius scale, density threshold) and refreshes the bounds. May leave the snapshot without cells.
    void PrepareInterfaceSnapshot(FUBotMediumSnapshot& Snapshot, const FQuerySettings& Settings)
    {
        const float Scale = RadiusScale(Settings);
        for (FUBotMediumCell& Cell : Snapshot.Cells)
        {
            Cell.RadiusCentimeters = FMath::Max(1.0f, Cell.RadiusCentimeters * Scale);
        }
        Snapshot.Cells.RemoveAll([&Settings](const FUBotMediumCell& Cell)
        {
            return UBot::Medium::EffectiveDensity(Cell) < Settings.DensityThreshold;
        });
        UBot::Medium::FinalizeSnapshot(Snapshot);
    }

    // True when the snapshot was produced by an IUBotPerceptionMedium implementer (the source itself or
    // the actor owning it). Legacy-path and hand-built snapshots are not.
    bool CameFromMediumInterface(const FUBotMediumSnapshot& Snapshot)
    {
        const UObject* Source = Snapshot.Source.Get();
        if (!Source)
        {
            return false;
        }
        if (Source->Implements<UUBotPerceptionMedium>())
        {
            return true;
        }
        const AActor* SourceActor = Source->GetTypedOuter<AActor>();
        return SourceActor && SourceActor->Implements<UUBotPerceptionMedium>();
    }

    // The nearest medium along a pixel ray decides the blend colour. When every snapshot blends towards
    // the same tint (always the case without Core interface media) the original array is intersected in
    // one call, exactly like the legacy code. Otherwise every snapshot is intersected on its own and the
    // nearest entry wins, which selects the same cell as one combined call (the first snapshot wins a tie).
    struct FMediaByTint
    {
        bool bSingleTint = true;
        FLinearColor SingleTint = FLinearColor::White;
        TArray<TArray<FUBotMediumSnapshot>> PerSnapshot;
        TArray<FLinearColor> PerSnapshotTint;
    };

    void GroupByTint(const TArray<FUBotMediumSnapshot>& Snapshots, const FLinearColor& SettingsTint, FMediaByTint& OutMedia)
    {
        TArray<FLinearColor> Tints;
        Tints.Reserve(Snapshots.Num());
        for (const FUBotMediumSnapshot& Snapshot : Snapshots)
        {
            Tints.Add(CameFromMediumInterface(Snapshot) ? Snapshot.Tint : SettingsTint);
        }

        OutMedia.SingleTint = Tints.IsEmpty() ? SettingsTint : Tints[0];
        OutMedia.bSingleTint = true;
        for (const FLinearColor& Tint : Tints)
        {
            if (Tint != OutMedia.SingleTint)
            {
                OutMedia.bSingleTint = false;
                break;
            }
        }

        if (!OutMedia.bSingleTint)
        {
            OutMedia.PerSnapshot.Reserve(Snapshots.Num());
            for (const FUBotMediumSnapshot& Snapshot : Snapshots)
            {
                OutMedia.PerSnapshot.AddDefaulted_GetRef().Add(Snapshot);
            }
            OutMedia.PerSnapshotTint = MoveTemp(Tints);
        }
    }

    bool FindNearestMedium(
        const FVector& SegmentStart,
        const FVector& SegmentEnd,
        const TArray<FUBotMediumSnapshot>& Snapshots,
        const FMediaByTint& Media,
        float DensityThreshold,
        float& OutDensity,
        FLinearColor& OutTint)
    {
        float DistanceCentimeters = 0.0f;
        if (Media.bSingleTint)
        {
            OutTint = Media.SingleTint;
            return UBot::Medium::FindFirstIntersection(SegmentStart, SegmentEnd, Snapshots, DensityThreshold, DistanceCentimeters, OutDensity);
        }

        float BestDistance = TNumericLimits<float>::Max();
        bool bFound = false;
        for (int32 Index = 0; Index < Media.PerSnapshot.Num(); ++Index)
        {
            float Density = 0.0f;
            if (UBot::Medium::FindFirstIntersection(SegmentStart, SegmentEnd, Media.PerSnapshot[Index], DensityThreshold, DistanceCentimeters, Density)
                && DistanceCentimeters < BestDistance)
            {
                BestDistance = DistanceCentimeters;
                OutDensity = Density;
                OutTint = Media.PerSnapshotTint[Index];
                bFound = true;
            }
        }
        return bFound;
    }
}

namespace UBot::SensorMedium
{
    FName LegacySmokeVolumeTag()
    {
        return Private::SmokeVolumeTagName;
    }

    FQuerySettings::FQuerySettings()
        : LegacySmokeVolumeTag(UBot::SensorMedium::LegacySmokeVolumeTag())
    {
    }

    FCameraSettings::FCameraSettings()
        : Tint(0.48f, 0.50f, 0.52f, 1.0f)
    {
    }

    void CollectMedia(UWorld& World, const FQuerySettings& Settings, TArray<FUBotMediumSnapshot>& OutSnapshots)
    {
        OutSnapshots.Reset();

        // Core interface path. CollectSnapshots resets its output and finalizes the snapshots.
        UBot::Medium::CollectSnapshots(World, OutSnapshots);
        for (FUBotMediumSnapshot& Snapshot : OutSnapshots)
        {
            Private::PrepareInterfaceSnapshot(Snapshot, Settings);
        }
        OutSnapshots.RemoveAll([](const FUBotMediumSnapshot& Snapshot)
        {
            return Snapshot.Cells.IsEmpty();
        });

        // Legacy tagged-actor path.
        if (Settings.LegacySmokeVolumeTag.IsNone())
        {
            return;
        }

        for (TActorIterator<AActor> ActorIt(&World); ActorIt; ++ActorIt)
        {
            AActor* Actor = *ActorIt;
            if (!Actor
                || Actor->IsActorBeingDestroyed()
                || !Actor->ActorHasTag(Settings.LegacySmokeVolumeTag)
                // Already collected through the Core interface; never count a medium twice.
                || Actor->Implements<UUBotPerceptionMedium>()
                || !Private::InvokeBoolNoParam(*Actor, Private::IsSmokeFieldActiveFunctionName, true))
            {
                continue;
            }

            FUBotMediumSnapshot Snapshot;
            Snapshot.Source = Actor;
            if (!Private::CopySmokeCells(*Actor, Settings, Snapshot.Cells) || Snapshot.Cells.IsEmpty())
            {
                continue;
            }

            UBot::Medium::FinalizeSnapshot(Snapshot);
            OutSnapshots.Add(MoveTemp(Snapshot));
        }
    }

    FVector CameraPixelDirectionLocal(int32 PixelX, int32 PixelY, float Fx, float Fy, float Cx, float Cy)
    {
        if (Fx <= UE_SMALL_NUMBER || Fy <= UE_SMALL_NUMBER)
        {
            return FVector::ForwardVector;
        }
        return FVector(
            1.0,
            (static_cast<float>(PixelX) - Cx) / Fx,
            -(static_cast<float>(PixelY) - Cy) / Fy).GetSafeNormal();
    }

    void ApplyToBGRAPixels(
        const FTransform& SensorTransform,
        int32 Width,
        int32 Height,
        float Fx,
        float Fy,
        float Cx,
        float Cy,
        float MaximumDistanceMeters,
        const TArray<FUBotMediumSnapshot>& Snapshots,
        const FCameraSettings& Settings,
        TArray<uint8>& PixelBytes)
    {
        if (Snapshots.IsEmpty() || Width <= 0 || Height <= 0 || PixelBytes.Num() != Width * Height * 4 || MaximumDistanceMeters <= 0.0f)
        {
            return;
        }

        Private::FMediaByTint Media;
        Private::GroupByTint(Snapshots, Settings.Tint, Media);

        const FVector Origin = SensorTransform.GetLocation();
        const float MaximumDistanceCentimeters = MaximumDistanceMeters * 100.0f;
        for (int32 Y = 0; Y < Height; ++Y)
        {
            for (int32 X = 0; X < Width; ++X)
            {
                const FVector LocalDirection = CameraPixelDirectionLocal(X, Y, Fx, Fy, Cx, Cy);
                const FVector WorldDirection = SensorTransform.TransformVectorNoScale(LocalDirection).GetSafeNormal();
                float MediumDensity = 0.0f;
                FLinearColor MediumTint = Settings.Tint;
                if (!Private::FindNearestMedium(
                        Origin,
                        Origin + WorldDirection * MaximumDistanceCentimeters,
                        Snapshots,
                        Media,
                        Settings.DensityThreshold,
                        MediumDensity,
                        MediumTint))
                {
                    continue;
                }

                const float Alpha = FMath::Clamp(MediumDensity * FMath::Max(0.0f, Settings.OpacityScale), 0.0f, Settings.MaximumOpacity);
                const int32 Offset = (Y * Width + X) * 4;
                const FLinearColor SourceColor(
                    static_cast<float>(PixelBytes[Offset + 2]) / 255.0f,
                    static_cast<float>(PixelBytes[Offset + 1]) / 255.0f,
                    static_cast<float>(PixelBytes[Offset]) / 255.0f,
                    static_cast<float>(PixelBytes[Offset + 3]) / 255.0f);
                const FLinearColor Blended = FMath::Lerp(SourceColor, MediumTint, Alpha).GetClamped();
                PixelBytes[Offset] = static_cast<uint8>(FMath::RoundToInt(Blended.B * 255.0f));
                PixelBytes[Offset + 1] = static_cast<uint8>(FMath::RoundToInt(Blended.G * 255.0f));
                PixelBytes[Offset + 2] = static_cast<uint8>(FMath::RoundToInt(Blended.R * 255.0f));
                PixelBytes[Offset + 3] = static_cast<uint8>(FMath::RoundToInt(SourceColor.A * 255.0f));
            }
        }
    }

    void ApplyToDepthValues(
        const FTransform& SensorTransform,
        int32 Width,
        int32 Height,
        float Fx,
        float Fy,
        float Cx,
        float Cy,
        float MinimumDepthMeters,
        float MaximumDepthMeters,
        const TArray<FUBotMediumSnapshot>& Snapshots,
        const FQuerySettings& Settings,
        TArray<float>& DepthValuesMeters)
    {
        if (Snapshots.IsEmpty() || Width <= 0 || Height <= 0 || DepthValuesMeters.Num() != Width * Height || MaximumDepthMeters <= MinimumDepthMeters)
        {
            return;
        }

        const FVector Origin = SensorTransform.GetLocation();
        for (int32 Y = 0; Y < Height; ++Y)
        {
            for (int32 X = 0; X < Width; ++X)
            {
                const int32 Index = Y * Width + X;
                const float ExistingDepthMeters = DepthValuesMeters[Index];
                const float SegmentMaxMeters = ExistingDepthMeters > 0.0f
                    ? FMath::Min(ExistingDepthMeters, MaximumDepthMeters)
                    : MaximumDepthMeters;
                if (SegmentMaxMeters <= MinimumDepthMeters)
                {
                    continue;
                }

                const FVector LocalDirection = CameraPixelDirectionLocal(X, Y, Fx, Fy, Cx, Cy);
                const FVector WorldDirection = SensorTransform.TransformVectorNoScale(LocalDirection).GetSafeNormal();
                float MediumDistanceCentimeters = 0.0f;
                float MediumDensity = 0.0f;
                if (!UBot::Medium::FindFirstIntersection(
                        Origin,
                        Origin + WorldDirection * SegmentMaxMeters * 100.0f,
                        Snapshots,
                        Settings.DensityThreshold,
                        MediumDistanceCentimeters,
                        MediumDensity))
                {
                    continue;
                }

                const float MediumDistanceMeters = MediumDistanceCentimeters * 0.01f;
                DepthValuesMeters[Index] = MediumDistanceMeters >= MinimumDepthMeters && MediumDistanceMeters <= MaximumDepthMeters
                    ? MediumDistanceMeters
                    : 0.0f;
            }
        }
    }
}
