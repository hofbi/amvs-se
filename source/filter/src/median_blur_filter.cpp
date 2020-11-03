#include "median_blur_filter.h"

using namespace lmt::filter;

cv_bridge::CvImagePtr MedianBlurFilter::filterImpl(const cv_bridge::CvImagePtr& inputImg)
{
    cv_bridge::CvImagePtr image = inputImg;
    cv::medianBlur(inputImg->image, image->image, ksize_);

    return image;
}

MedianBlurFilter::MedianBlurFilter(int ksize) : ksize_(ksize) {}
