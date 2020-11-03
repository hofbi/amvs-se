#ifndef SINGLE_ENCODER_MDVQM_PARAM_H
#define SINGLE_ENCODER_MDVQM_PARAM_H

#include <vector>

namespace lmt
{
namespace quality
{
struct MDVQMParam
{
    std::vector<double> a{-0.632, 0.6591, 4.1797, -0.0352, 0.1133, 5.6086};
    double as{0.0222};
    double at{0.0518};
    double bs{0.0035};
    double bt{0.7889};
};
}  // namespace quality
}  // namespace lmt

#endif  // SINGLE_ENCODER_MDVQM_PARAM_H
