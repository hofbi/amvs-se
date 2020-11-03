#ifndef SINGLE_ENCODER_TEMPORAL_FILTER_H
#define SINGLE_ENCODER_TEMPORAL_FILTER_H

#include "base_image_filter.h"

namespace lmt
{
namespace filter
{
class TemporalFilter final : public BaseImageFilter
{
  public:
    explicit TemporalFilter(uint8_t skipFrames) noexcept;

  private:
    cv_bridge::CvImagePtr filterImpl(const cv_bridge::CvImagePtr& inputImg) override;

    uint8_t skipFrames_{0};
    int8_t skipFrameCount_{-1};
    cv_bridge::CvImagePtr oldImg_{nullptr};
};
}  // namespace filter
}  // namespace lmt

#endif  // SINGLE_ENCODER_TEMPORAL_FILTER_H
