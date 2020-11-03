#include "temporal_filter.h"

using namespace lmt::filter;

cv_bridge::CvImagePtr TemporalFilter::filterImpl(const cv_bridge::CvImagePtr& inputImg)
{
    if (skipFrameCount_ < 0)
    {
        oldImg_ = inputImg;
        skipFrameCount_ = skipFrames_;
    }
    --skipFrameCount_;

    return oldImg_;
}

TemporalFilter::TemporalFilter(uint8_t skipFrames) noexcept : skipFrames_(skipFrames) {}
