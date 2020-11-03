#include "filter_parameter.h"

#include <algorithm>

#include <gtest/gtest.h>

using namespace lmt::filter;

TEST(FilterParameterTest, constructor_default_correct)
{
    FilterParameter unit;

    EXPECT_EQ(0, unit.ksize);
    EXPECT_EQ(0.0, unit.sigma);
    EXPECT_EQ("none", unit.type);
}

TEST(FilterParameterTest, serialize_defaultObject_separatedByOneDash)
{
    FilterParameter unit;

    const auto actualResult = unit.serialize();

    EXPECT_EQ(1, std::count(actualResult.begin(), actualResult.end(), '-'));
}

TEST(FilterParameterTest, serialize_gaussianType_separatedByTwoDash)
{
    FilterParameter unit{FilterTypeGaussian};

    const auto actualResult = unit.serialize();

    EXPECT_EQ(2, std::count(actualResult.begin(), actualResult.end(), '-'));
}
