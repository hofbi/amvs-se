#include "statistics_collection.h"

#include <gtest/gtest.h>

#include "encoder_statistics.h"
#include "frame_stats.h"

using namespace lmt::statistics;

class StatisticsCollectionTest : public ::testing::Test
{
  protected:
    StatisticsCollectionTest() = default;

    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    EncoderStatistics statisticsOne_{0, "one", 0, 10, 5, 1000, false, {}};
    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    EncoderStatistics statisticsTwo_{0, "two", 0, 10, 5, 1000, false, {}};
};

TEST_F(StatisticsCollectionTest, hasData_defaultConstructedWithEmptyRange_true)
{
    StatisticsCollection unit({});

    EXPECT_TRUE(unit.hasData());
    EXPECT_TRUE(unit.getStats().empty());
}

TEST_F(StatisticsCollectionTest, hasData_defaultConstructedWithTwoStatistics_false)
{
    StatisticsCollection unit({statisticsOne_, statisticsTwo_});

    EXPECT_FALSE(unit.hasData());
    EXPECT_EQ(2, unit.getStats().size());
}

TEST_F(StatisticsCollectionTest, hasData_statsAddedToAll_true)
{
    StatisticsCollection unit({statisticsOne_, statisticsTwo_});

    unit.addFrameStatsById(0, {});
    unit.addFrameStatsById(1, {});

    EXPECT_TRUE(unit.hasData());
}

TEST_F(StatisticsCollectionTest, hasData_statsAddedToOne_false)
{
    StatisticsCollection unit({statisticsOne_, statisticsTwo_});

    unit.addFrameStatsById(0, {});

    EXPECT_FALSE(unit.hasData());
}

TEST_F(StatisticsCollectionTest, addEncoderStatistic_empty_oneStat)
{
    StatisticsCollection unit({});

    unit.addEncoderStatistic(statisticsOne_);

    EXPECT_EQ(1, unit.getStats().size());
}

TEST_F(StatisticsCollectionTest, addFrameStatsById_empty_oneStat)
{
    StatisticsCollection unit({statisticsOne_});
    EXPECT_TRUE(unit.getStats().front().getTimestamps().empty());

    unit.addFrameStatsById(0, {});

    EXPECT_EQ(1, unit.getStats().front().getTimestamps().size());
}
