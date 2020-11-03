#ifndef SINGLE_ENCODER_NONE_FILTER_H
#define SINGLE_ENCODER_NONE_FILTER_H

#include "base_image_filter.h"

namespace lmt
{
namespace filter
{
class NoneFilter final : public BaseImageFilter
{
  private:
    cv_bridge::CvImagePtr filterImpl(const cv_bridge::CvImagePtr& inputImg) override;
};
}  // namespace filter
}  // namespace lmt

#endif  // SINGLE_ENCODER_NONE_FILTER_H
