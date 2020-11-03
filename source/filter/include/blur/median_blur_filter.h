#ifndef SINGLE_ENCODER_MEDIAN_BLUR_FILTER_H
#define SINGLE_ENCODER_MEDIAN_BLUR_FILTER_H

#include "base_image_filter.h"

namespace lmt
{
namespace filter
{
class MedianBlurFilter final : public BaseImageFilter
{
  public:
    explicit MedianBlurFilter(int ksize);

  private:
    cv_bridge::CvImagePtr filterImpl(const cv_bridge::CvImagePtr& inputImg) override;

    int ksize_{1};
};
}  // namespace filter
}  // namespace lmt

#endif  // SINGLE_ENCODER_MEDIAN_BLUR_FILTER_H
