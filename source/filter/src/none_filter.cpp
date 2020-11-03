#include "none_filter.h"

using namespace lmt::filter;

cv_bridge::CvImagePtr NoneFilter::filterImpl(const cv_bridge::CvImagePtr& inputImg)
{
    return inputImg;
}
