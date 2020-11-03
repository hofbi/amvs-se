#include "blur_filter.h"

using namespace lmt::filter;

BlurFilter::BlurFilter(int ksize) : ksize_(ksize, ksize) {}

cv_bridge::CvImagePtr BlurFilter::filterImpl(const cv_bridge::CvImagePtr& inputImg)
{
    cv_bridge::CvImagePtr image = inputImg;
    cv::blur(inputImg->image, image->image, ksize_, cv::Point(-1, -1));

    return image;
}
