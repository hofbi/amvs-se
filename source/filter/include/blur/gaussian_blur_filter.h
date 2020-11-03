#ifndef SINGLE_ENCODER_GAUSSIAN_BLUR_FILTER_H
#define SINGLE_ENCODER_GAUSSIAN_BLUR_FILTER_H

#include "base_image_filter.h"

namespace lmt
{
namespace filter
{
class GaussianBlurFilter final : public BaseImageFilter
{
  public:
    explicit GaussianBlurFilter(int ksize, double sigma);

  private:
    cv_bridge::CvImagePtr filterImpl(const cv_bridge::CvImagePtr& inputImg) override;

    cv::Size ksize_{};
    double sigma_{};
};
}  // namespace filter
}  // namespace lmt

#endif  // SINGLE_ENCODER_GAUSSIAN_BLUR_FILTER_H
