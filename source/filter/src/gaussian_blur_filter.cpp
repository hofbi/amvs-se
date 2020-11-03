#include "gaussian_blur_filter.h"

using namespace lmt::filter;

GaussianBlurFilter::GaussianBlurFilter(int ksize, double sigma) : ksize_(ksize, ksize), sigma_(sigma) {}

cv_bridge::CvImagePtr GaussianBlurFilter::filterImpl(const cv_bridge::CvImagePtr& inputImg)
{
    cv_bridge::CvImagePtr image = inputImg;
    cv::GaussianBlur(inputImg->image, image->image, ksize_, sigma_);

    return image;
}
