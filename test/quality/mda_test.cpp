#include "mda.h"

#include <gtest/gtest.h>

using namespace lmt::quality;

class MDATest : public ::testing::Test
{
  protected:
    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    const uint16_t width_{15};
    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    const uint16_t height_{10};
    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    MDA unit_{width_, height_};
};

TEST_F(MDATest, constructor_initSize_correctAndNoSkipFrames)
{
    EXPECT_EQ(0, unit_.getSkipFrames());
    EXPECT_EQ(width_, unit_.getWidth());
    EXPECT_EQ(height_, unit_.getHeight());
    EXPECT_FALSE(unit_.isNewCodingMode());
}

TEST_F(MDATest, updateFrame_singleUpdate_noParameterUpdateRequired)
{
    const cv::Mat image(width_, height_, CV_8UC3);

    unit_.updateFrame(image);

    EXPECT_FALSE(unit_.isNewCodingMode());
}

TEST_F(MDATest, updateFrame_aboveMinimumUpdate_parameterUpdateRequired)
{
    const cv::Mat image(width_, height_, CV_8UC3);

    for (int i = 0; i < 21; ++i)
    {
        unit_.updateFrame(image);
    }

    EXPECT_TRUE(unit_.isNewCodingMode());
}
