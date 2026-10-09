#pragma once

#include "CoreMinimal.h"
#include "UBotPerceptionMedium.h"

class UWorld;

/**
 * Perception media (smoke, fog, ...) as seen by the uBot sensors.
 *
 * This is a thin layer over Core's UBot::Medium. Media are discovered on two paths and both end up
 * as FUBotMediumSnapshot:
 *  - Core interface path: every actor implementing IUBotPerceptionMedium (UBot::Medium::CollectSnapshots).
 *  - Legacy path: actors tagged LegacySmokeVolumeTag() that expose, through reflection only,
 *    IsSmokeFieldActive() (optional, defaults to true), GetSmokeCellSnapshots() returning an array of
 *    structs with WorldPosition (FVector, cm), Density and optional EdgeFactor, and the float properties
 *    CellVisualRadiusMeters / CellSizeMeters. This keeps the AgentCombatCore smoke actors working.
 *
 * Intersections go through UBot::Medium::FindFirstIntersection with FQuerySettings::DensityThreshold.
 */
namespace UBot::SensorMedium
{
    /** TEXT("AgentSensor.SmokeVolume"), kept for the AgentCombatCore smoke actors. */
    UBOTSENSOR_API FName LegacySmokeVolumeTag();

    struct UBOTSENSOR_API FQuerySettings
    {
        FQuerySettings();

        /** Actor tag of the legacy reflection path. None disables that path (the Core interface path stays on). */
        FName LegacySmokeVolumeTag;

        /** Cells whose effective density (Density * EdgeFactor) is below this value are ignored. */
        float DensityThreshold = 0.05f;

        /** Legacy path only: radius used when the actor exposes neither CellVisualRadiusMeters nor CellSizeMeters. */
        float DefaultCellRadiusCentimeters = 150.0f;

        /** Scales every cell radius (both paths), clamped to at least 0.01. */
        float CellRadiusScale = 1.0f;
    };

    struct UBOTSENSOR_API FCameraSettings : public FQuerySettings
    {
        FCameraSettings();

        /**
         * Colour the camera image is blended towards. Snapshots that came from the Core interface use
         * their own FUBotMediumSnapshot::Tint instead; legacy and hand-built snapshots use this one.
         */
        FLinearColor Tint;

        float OpacityScale = 0.75f;
        float MaximumOpacity = 0.85f;
    };

    /**
     * Collects the media of World: Core interface snapshots first, then legacy tagged actors (actors that
     * implement IUBotPerceptionMedium are never collected twice). On both paths cell radii are scaled by
     * CellRadiusScale (at least 1 cm), cells below DensityThreshold are dropped, snapshots left without
     * cells are skipped and the bounds are finalized. OutSnapshots is reset first.
     */
    UBOTSENSOR_API void CollectMedia(UWorld& World, const FQuerySettings& Settings, TArray<FUBotMediumSnapshot>& OutSnapshots);

    /** Unit ray of a pixel in the camera frame (X forward, Y right, Z up) for pinhole intrinsics. */
    UBOTSENSOR_API FVector CameraPixelDirectionLocal(int32 PixelX, int32 PixelY, float Fx, float Fy, float Cx, float Cy);

    /**
     * Blends every BGRA8 pixel whose ray enters a medium within MaximumDistanceMeters towards the tint,
     * with alpha = Clamp(Density * OpacityScale, 0, MaximumOpacity). The source alpha is kept.
     * Numerically identical to the legacy ApplySmokeToBGRAPixels for equivalent input.
     */
    UBOTSENSOR_API void ApplyToBGRAPixels(
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
        TArray<uint8>& PixelBytes);

    /**
     * Replaces every depth value (meters, 0 = invalid) whose ray enters a medium before the measured
     * surface by the medium entry distance, or by 0 when that distance is outside
     * [MinimumDepthMeters, MaximumDepthMeters].
     * Numerically identical to the legacy ApplySmokeToDepthValues for equivalent input.
     */
    UBOTSENSOR_API void ApplyToDepthValues(
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
        TArray<float>& DepthValuesMeters);
}
