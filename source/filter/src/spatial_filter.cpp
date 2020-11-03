#include "spatial_filter.h"

using namespace lmt::filter;

SpatialFilter::SpatialFilter(uint16_t outWidth, uint16_t outHeight) noexcept
    : outWidth_(outWidth), outHeight_(outHeight)
{
}

cv_bridge::CvImagePtr SpatialFilter::filterImpl(const cv_bridge::CvImagePtr& inputImg)
{
    if (inputImg->image.cols == outWidth_ && inputImg->image.rows == outHeight_)
    {
        return inputImg;
    }

    cv_bridge::CvImagePtr image = inputImg;
    cv::resize(inputImg->image, image->image, cv::Size(outWidth_, outHeight_));

    return image;
}
