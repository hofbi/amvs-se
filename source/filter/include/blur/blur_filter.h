#ifndef SINGLE_ENCODER_BLUR_FILTER_H
#define SINGLE_ENCODER_BLUR_FILTER_H

#include "base_image_filter.h"

namespace lmt
{
namespace filter
{
class BlurFilter final : public BaseImageFilter
{
  public:
    explicit BlurFilter(int ksize);

  private:
    cv_bridge::CvImagePtr filterImpl(const cv_bridge::CvImagePtr& inputImg) override;

    cv::Size ksize_{};
};
}  // namespace filter
}  // namespace lmt
#endif  // SINGLE_ENCODER_BLUR_FILTER_H
