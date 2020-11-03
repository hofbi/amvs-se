#include "base_image_filter.h"

using namespace lmt::filter;

BaseImageFilter::~BaseImageFilter() = default;

cv_bridge::CvImagePtr BaseImageFilter::filter(const cv_bridge::CvImagePtr& inputImg)
{
    return filterImpl(inputImg);
}
