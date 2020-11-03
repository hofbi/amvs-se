#ifndef SINGLE_ENCODER_BASE_IMAGE_FILTER_H
#define SINGLE_ENCODER_BASE_IMAGE_FILTER_H

#include <cv_bridge/cv_bridge.h>

namespace lmt
{
namespace filter
{
class BaseImageFilter
{
  public:
    virtual ~BaseImageFilter();

    cv_bridge::CvImagePtr filter(const cv_bridge::CvImagePtr& inputImg);

  private:
    virtual cv_bridge::CvImagePtr filterImpl(const cv_bridge::CvImagePtr& inputImg) = 0;
};
}  // namespace filter
}  // namespace lmt

#endif  // SINGLE_ENCODER_BASE_IMAGE_FILTER_H
