#include "mdvqm.h"

#include <gtest/gtest.h>

using namespace lmt::quality;

class MDVQMTest : public ::testing::Test
{
  protected:
    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    MDVQMParam param_;
};

TEST_F(MDVQMTest, getSpatialCorrectionFactor_spatialScaleFactorOne_almostOne)
{
    const auto actualSCF = getSpatialCorrectionFactor(1, param_.bs, 1);

    EXPECT_DOUBLE_EQ(1, actualSCF);
}

TEST_F(MDVQMTest, getSpatialCorrectionFactor_spatialSAZero_almostOne)
{
    const auto actualSCF = getSpatialCorrectionFactor(0.5, param_.bs, 0);

    EXPECT_DOUBLE_EQ(1, actualSCF);
}

TEST_F(MDVQMTest, getSpatialCorrectionFactor_spatialScaleFactorBetweenZeroAndOne_lessOne)
{
    const auto actualSCF = getSpatialCorrectionFactor(0.5, param_.bs, 1);

    EXPECT_LT(actualSCF, 1);
}

TEST_F(MDVQMTest, getTemporalCorrectionFactor_rateEqRateMax_almostOne)
{
    const auto actualTCF = getTemporalCorrectionFactor(20, 20, param_.bt, 1);

    EXPECT_DOUBLE_EQ(1, actualTCF);
}

TEST_F(MDVQMTest, getTemporalCorrectionFactor_TAAlmostZero_almostOne)
{
    const auto actualTCF = getTemporalCorrectionFactor(20, 20, param_.bt, 1e-15);

    EXPECT_DOUBLE_EQ(1, actualTCF);
}

TEST_F(MDVQMTest, getTemporalCorrectionFactor_rateNotEqRateMax_lessOne)
{
    const auto actualTCF = getTemporalCorrectionFactor(10, 20, param_.bt, 1);

    EXPECT_LE(actualTCF, 1);
}
