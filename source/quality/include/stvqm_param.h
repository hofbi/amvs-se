#ifndef SINGLE_ENCODER_STVQM_PARAM_H
#define SINGLE_ENCODER_STVQM_PARAM_H

namespace lmt
{
namespace quality
{
struct STVQMParam
{
    double ws{0.0356};
    double wt{0.236};
    double mu{36.9};
    double s{2.59};
    double a{0.028};
    double b{0.764};
};
}  // namespace quality
}  // namespace lmt

#endif  // SINGLE_ENCODER_STVQM_PARAM_H
