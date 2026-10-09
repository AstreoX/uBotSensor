#include "UBotPointCloudFrame.h"

bool UUBotPointCloudFrame::GetPointByIndex(int32 Index, FVector& PointMeters, int32& RingIndex, float& TimeOffsetSeconds, float& Intensity, bool& bHasIntensity) const
{
    PointMeters = FVector::ZeroVector;
    RingIndex = 0;
    TimeOffsetSeconds = 0.0f;
    Intensity = 0.0f;
    bHasIntensity = false;
    if (!PointsMeters.IsValidIndex(Index) || !RingIndices.IsValidIndex(Index) || !TimeOffsetsSeconds.IsValidIndex(Index))
    {
        return false;
    }
    PointMeters = PointsMeters[Index];
    RingIndex = RingIndices[Index];
    TimeOffsetSeconds = TimeOffsetsSeconds[Index];
    bHasIntensity = Intensities.IsValidIndex(Index);
    if (bHasIntensity)
    {
        Intensity = Intensities[Index];
    }
    return true;
}
