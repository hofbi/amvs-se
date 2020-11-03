#include "base_image_filter_factory.h"

#include "blur_filter.h"
#include "filter_parameter.h"
#include "gaussian_blur_filter.h"
#include "median_blur_filter.h"
#include "none_filter.h"

namespace lmt
{
namespace filter
{
std::unique_ptr<BaseImageFilter> createFilterFromParameter(const FilterParameter& filterParameter)
{
    if (filterParameter.type == FilterTypeBlur)
    {
        return std::make_unique<BlurFilter>(filterParameter.ksize);
    }
    if (filterParameter.type == FilterTypeGaussian)
    {
        return std::make_unique<GaussianBlurFilter>(filterParameter.ksize, filterParameter.sigma);
    }
    if (filterParameter.type == FilterTypeMedian)
    {
        return std::make_unique<MedianBlurFilter>(filterParameter.ksize);
    }

    return std::make_unique<NoneFilter>();
}
}  // namespace filter
}  // namespace lmt
