#include "filter_parameter.h"

#include <sstream>

using namespace lmt::filter;

std::string FilterParameter::serialize() const
{
    std::ostringstream stream;
    stream << type << "-" << ksize;

    if (type == FilterTypeGaussian)
    {
        stream << "-" << sigma;
    }

    return stream.str();
}
