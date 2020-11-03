#ifndef SINGLE_ENCODER_BASE_IMAGE_FILTER_FACTORY_H
#define SINGLE_ENCODER_BASE_IMAGE_FILTER_FACTORY_H

#include "base_image_filter.h"

namespace lmt
{
namespace filter
{
struct FilterParameter;

std::unique_ptr<BaseImageFilter> createFilterFromParameter(const FilterParameter& filterParameter);
}  // namespace filter
}  // namespace lmt

#endif  // SINGLE_ENCODER_BASE_IMAGE_FILTER_FACTORY_H
