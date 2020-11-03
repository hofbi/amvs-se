#ifndef SINGLE_ENCODER_CHANNEL_FILTER_H
#define SINGLE_ENCODER_CHANNEL_FILTER_H

#include "base_image_filter.h"

namespace lmt
{
namespace filter
{
class ChannelFilter final : public BaseImageFilter
{
  public:
    explicit ChannelFilter(bool dropChroma) noexcept;

  private:
    cv_bridge::CvImagePtr filterImpl(const cv_bridge::CvImagePtr& inputImg) override;

    bool dropChroma_{false};
};

cv::ColorConversionCodes getCvColorConversionCodeFromSensorMsg(const cv_bridge::CvImageConstPtr& img) noexcept;
void dropChroma(cv::Mat& yuvImage);
}  // namespace filter
}  // namespace lmt

#endif  // SINGLE_ENCODER_CHANNEL_FILTER_H
