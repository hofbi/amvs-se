#include "stvqm.h"

#include <gtest/gtest.h>

using namespace lmt::quality;

class STVQMTest : public ::testing::Test
{
  protected:
    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    STVQMParam param_;
};

TEST_F(STVQMTest, getTemporalCorrectionFactor_rateEqRateMax_almostOne)
{
    const auto actualTCF = getTemporalCorrectionFactor(20, 20, param_.a, param_.b, 1);

    EXPECT_DOUBLE_EQ(1, actualTCF);
}

TEST_F(STVQMTest, getTemporalCorrectionFactor_TAAlmostZero_almostOne)
{
    const auto actualTCF = getTemporalCorrectionFactor(20, 20, param_.a, param_.b, 1e-15);

    EXPECT_DOUBLE_EQ(1, actualTCF);
}

TEST_F(STVQMTest, getTemporalCorrectionFactor_rateNotEqRateMax_lessOne)
{
    const auto actualTCF = getTemporalCorrectionFactor(10, 20, param_.a, param_.b, 1);

    EXPECT_LE(actualTCF, 1);
}
