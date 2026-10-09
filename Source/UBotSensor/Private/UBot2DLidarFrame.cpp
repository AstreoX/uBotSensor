#include "UBot2DLidarFrame.h"

bool UUBot2DLidarFrame::GetRangeByIndex(int32 Index, float& RangeMeters, bool& bHit) const
{
    if (!RangesMeters.IsValidIndex(Index) || !HitFlags.IsValidIndex(Index))
    {
        RangeMeters = 0.0f;
        bHit = false;
        return false;
    }
    RangeMeters = RangesMeters[Index];
    bHit = HitFlags[Index] != 0;
    return true;
}

TArray<bool> UUBot2DLidarFrame::CopyAllHitFlags() const
{
    TArray<bool> Result;
    Result.Reserve(HitFlags.Num());
    for (uint8 Flag : HitFlags)
    {
        Result.Add(Flag != 0);
    }
    return Result;
}
