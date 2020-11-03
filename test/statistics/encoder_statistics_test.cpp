#include "encoder_statistics.h"

#include <gtest/gtest.h>

#include "frame_stats.h"

using namespace lmt::statistics;

class EncoderStatisticsTest : public ::testing::Test
{
  protected:
    EncoderStatisticsTest() = default;

    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    EncoderStatistics defaultUint_{0, "topic", 0, 10, 5, 1000, false, {}};
};

TEST_F(EncoderStatisticsTest, constructor_defaultUnit_correct)
{
    EXPECT_EQ(0, defaultUint_.getEncoderId());
    EXPECT_EQ("topic", defaultUint_.getTopicName());
    EXPECT_EQ(0, defaultUint_.getSkipFrames());
    EXPECT_EQ(10, defaultUint_.getWidth());
    EXPECT_EQ(5, defaultUint_.getHeight());
    EXPECT_EQ(1000, defaultUint_.getBitrate());
    EXPECT_FALSE(defaultUint_.isDropChroma());
}

TEST_F(EncoderStatisticsTest, getEncodingStats_defaultUnit_allEmpty)
{
    EXPECT_TRUE(defaultUint_.getEncodedFrameSizes().empty());
    EXPECT_TRUE(defaultUint_.getFrameCounts().empty());
    EXPECT_TRUE(defaultUint_.getEncodingTimes().empty());
    EXPECT_TRUE(defaultUint_.getTimestamps().empty());
}

TEST_F(EncoderStatisticsTest, getEncodingStats_defaultUnitAndAddOne_sizeIsOne)
{
    defaultUint_.addFrameStats({0, 0, 0, 0});

    const auto expectedSize = 1;

    EXPECT_EQ(expectedSize, defaultUint_.getEncodedFrameSizes().size());
    EXPECT_EQ(expectedSize, defaultUint_.getFrameCounts().size());
    EXPECT_EQ(expectedSize, defaultUint_.getEncodingTimes().size());
    EXPECT_EQ(expectedSize, defaultUint_.getTimestamps().size());
}

TEST_F(EncoderStatisticsTest, getEncodingStats_defaultUnitAndAddThree_sizeIsThree)
{
    defaultUint_.addFrameStats({0, 0, 0, 0});
    defaultUint_.addFrameStats({0, 0, 0, 0});
    defaultUint_.addFrameStats({0, 0, 0, 0});

    const auto expectedSize = 3;

    EXPECT_EQ(expectedSize, defaultUint_.getEncodedFrameSizes().size());
    EXPECT_EQ(expectedSize, defaultUint_.getFrameCounts().size());
    EXPECT_EQ(expectedSize, defaultUint_.getEncodingTimes().size());
    EXPECT_EQ(expectedSize, defaultUint_.getTimestamps().size());
}

TEST_F(EncoderStatisticsTest, createSuperFrameStatistics_emptyInputStatistic_emptyStatistic)
{
    const auto superFrameStat = EncoderStatistics::createSuperFrameStatistics({});

    EXPECT_EQ(0, superFrameStat.getEncoderId());
    EXPECT_EQ("combined", superFrameStat.getTopicName());
    EXPECT_EQ(0, superFrameStat.getWidth());
    EXPECT_EQ(0, superFrameStat.getHeight());
    EXPECT_FALSE(defaultUint_.isDropChroma());
}

TEST_F(EncoderStatisticsTest, createSuperFrameStatistics_oneInputStatistic_inputStatistic)
{
    const auto superFrameStat = EncoderStatistics::createSuperFrameStatistics({defaultUint_});

    const auto expectedId = defaultUint_.getEncoderId() + 1;
    const auto expectedWidth = defaultUint_.getWidth();
    const auto expectedHeight = defaultUint_.getHeight();
    const auto expectedBitrate = defaultUint_.getBitrate();

    EXPECT_EQ(expectedId, superFrameStat.getEncoderId());
    EXPECT_EQ("combined", superFrameStat.getTopicName());
    EXPECT_EQ(expectedWidth, superFrameStat.getWidth());
    EXPECT_EQ(expectedHeight, superFrameStat.getHeight());
    EXPECT_EQ(expectedBitrate, superFrameStat.getBitrate());
    EXPECT_FALSE(defaultUint_.isDropChroma());
}

TEST_F(EncoderStatisticsTest, createSuperFrameStatistics_twoEqualInputStatistics_correctSize)
{
    const auto superFrameStat = EncoderStatistics::createSuperFrameStatistics({defaultUint_, defaultUint_});

    const auto expectedId = defaultUint_.getEncoderId() + 2;
    const auto expectedWidth = 2 * defaultUint_.getWidth();
    const auto expectedHeight = defaultUint_.getHeight();
    const auto expectedBitrate = 2 * defaultUint_.getBitrate();

    EXPECT_EQ(expectedId, superFrameStat.getEncoderId());
    EXPECT_EQ("combined", superFrameStat.getTopicName());
    EXPECT_EQ(expectedWidth, superFrameStat.getWidth());
    EXPECT_EQ(expectedHeight, superFrameStat.getHeight());
    EXPECT_EQ(expectedBitrate, superFrameStat.getBitrate());
    EXPECT_FALSE(defaultUint_.isDropChroma());
}

TEST_F(EncoderStatisticsTest, createSuperFrameStatistics_twoUnequalInputStatistics_correctSize)
{
    EncoderStatistics secondTestStatistic{1, "", 1, 15, 3, 1000, true, {}};
    const auto superFrameStat = EncoderStatistics::createSuperFrameStatistics({defaultUint_, secondTestStatistic});

    const auto expectedId = defaultUint_.getEncoderId() + 2;
    const auto expectedWidth = defaultUint_.getWidth() + secondTestStatistic.getWidth();
    const auto expectedHeight = defaultUint_.getHeight();
    const auto expectedBitrate = defaultUint_.getBitrate() + secondTestStatistic.getBitrate();

    EXPECT_EQ(expectedId, superFrameStat.getEncoderId());
    EXPECT_EQ("combined", superFrameStat.getTopicName());
    EXPECT_EQ(expectedWidth, superFrameStat.getWidth());
    EXPECT_EQ(expectedHeight, superFrameStat.getHeight());
    EXPECT_EQ(expectedBitrate, superFrameStat.getBitrate());
    EXPECT_FALSE(defaultUint_.isDropChroma());
}
