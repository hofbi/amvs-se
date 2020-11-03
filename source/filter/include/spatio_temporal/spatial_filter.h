#ifndef SINGLE_ENCODER_SPATIAL_FILTER_H
#define SINGLE_ENCODER_SPATIAL_FILTER_H

#include "base_image_filter.h"

namespace lmt
{
namespace filter
{
class SpatialFilter final : public BaseImageFilter
{
  public:
    SpatialFilter(uint16_t outWidth, uint16_t outHeight) noexcept;

  private:
    cv_bridge::CvImagePtr filterImpl(const cv_bridge::CvImagePtr& inputImg) override;

    uint16_t outWidth_;
    uint16_t outHeight_;
};
}  // namespace filter
}  // namespace lmt

#endif  // SINGLE_ENCODER_SPATIAL_FILTER_H
