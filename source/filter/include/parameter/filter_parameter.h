#ifndef SINGLE_ENCODER_FILTER_PARAMETER_H
#define SINGLE_ENCODER_FILTER_PARAMETER_H

#include <string>

namespace lmt
{
namespace filter
{
constexpr auto FilterTypeNone = "none";
constexpr auto FilterTypeGaussian = "gaussian";
constexpr auto FilterTypeMedian = "median";
constexpr auto FilterTypeBlur = "blur";

struct FilterParameter
{
    std::string type{FilterTypeNone};
    int ksize{0};
    double sigma{0.0};

    std::string serialize() const;
};
}  // namespace filter
}  // namespace lmt

#endif  // SINGLE_ENCODER_FILTER_PARAMETER_H
