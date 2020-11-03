#include "vqm_state.h"

#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>

using namespace lmt::quality;

constexpr auto akiyo0Path = "quality/resources/akiyo-0.png";
constexpr auto akiyo1Path = "quality/resources/akiyo-1.png";
constexpr auto mobile0Path = "quality/resources/mobile-0.png";
constexpr auto mobile1Path = "quality/resources/mobile-1.png";

class VQMStateTest : public ::testing::Test
{
  protected:
    VQMStateTest() { cv::randn(imageWithActivity_, cv::Scalar::all(100), cv::Scalar::all(100)); }

    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    const cv::Mat imageNoTA_{cv::Mat(10, 10, CV_8UC3)};
    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    cv::Mat imageWithActivity_{cv::Mat(10, 10, CV_8UC3)};
    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    const cv::Mat akiyo0_{cv::imread(akiyo0Path)};
    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    const cv::Mat akiyo1_{cv::imread(akiyo1Path)};
    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    const cv::Mat mobile0_{cv::imread(mobile0Path)};
    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    const cv::Mat mobile1_{cv::imread(mobile1Path)};
};

TEST_F(VQMStateTest, getCurrentFrameRateFromSkipFrames_noSkipFrames_equalToMax)
{
    const auto maxFR{20};
    const auto skipFrames{0};

    const auto actualFR = getCurrentFrameRateFromSkipFrames(maxFR, skipFrames);

    EXPECT_EQ(maxFR, actualFR);
}

TEST_F(VQMStateTest, getCurrentFrameRateFromSkipFrames_oneSkipFrames_halfOfMax)
{
    const auto maxFR{20};
    const auto skipFrames{1};

    const auto actualFR = getCurrentFrameRateFromSkipFrames(maxFR, skipFrames);

    EXPECT_EQ(maxFR / 2, actualFR);
}

TEST_F(VQMStateTest, getCurrentFrameRateFromSkipFrames_twoSkipFrames_thirdOfMax)
{
    const auto maxFR{20.0};
    const auto skipFrames{2};

    const auto actualFR = getCurrentFrameRateFromSkipFrames(maxFR, skipFrames);

    EXPECT_DOUBLE_EQ(maxFR / 3, actualFR);
}

TEST_F(VQMStateTest, getYChannel_3ChannelIn_1ChannelOut)
{
    const auto actual = getYChannel(imageWithActivity_);

    EXPECT_EQ(1, actual.channels());
    EXPECT_EQ(CV_8UC1, actual.type());
}

TEST_F(VQMStateTest, getYChannel_1ChannelIn_unChanged)
{
    cv::Mat singleChannel(10, 10, CV_8UC1);

    const auto actual = getYChannel(singleChannel);

    EXPECT_EQ(1, actual.channels());
    EXPECT_EQ(CV_8UC1, actual.type());
}

TEST_F(VQMStateTest, getTemporalActivity_emptyOldImage_zeroTA)
{
    const auto actualTA = getTemporalActivity(imageNoTA_, {});

    EXPECT_DOUBLE_EQ(0, actualTA);
}

TEST_F(VQMStateTest, getTemporalActivity_noTemporalDiff_zeroTA)
{
    const auto actualTA = getTemporalActivity(imageNoTA_, imageNoTA_);

    EXPECT_EQ(0, actualTA);
}

TEST_F(VQMStateTest, getTemporalActivity_noTemporalDiffAkiyo_zeroTA)
{
    const auto actualTA = getTemporalActivity(akiyo0_, akiyo0_);

    EXPECT_EQ(0, actualTA);
}

TEST_F(VQMStateTest, getTemporalActivity_temporalDiff_nonZeroTA)
{
    const auto actualTA = getTemporalActivity(imageWithActivity_, imageNoTA_);

    EXPECT_NE(0, actualTA);
}

TEST_F(VQMStateTest, DISABLED_getTemporalActivity_realImagesAkiyo_correctTA)
{
    const auto actualTA = getTemporalActivity(akiyo1_, akiyo0_);

    EXPECT_EQ(0.926952, actualTA);
}

TEST_F(VQMStateTest, DISABLED_getTemporalActivity_realImagesMobile_correctTA)
{
    const auto actualTA = getTemporalActivity(mobile1_, mobile0_);

    EXPECT_EQ(24.726, actualTA);
}

TEST_F(VQMStateTest, getMeanTemporalActivity_temporalActivityDiffButTooLongAgo_zeroTA)
{
    VQMState unit(0);

    unit.update(imageWithActivity_);
    for (int i = 0; i < VQMState::activityHistorySize + 1; ++i)  // Update once more because of diff to previous frame
    {
        if (i == 0)
        {
            EXPECT_EQ(0, unit.getTA());
            EXPECT_EQ(0, unit.getMeanTA());
        }
        else
        {
            EXPECT_NE(0, unit.getMeanTA());
        }
        unit.update(imageNoTA_);
    }

    EXPECT_EQ(0, unit.getTA());
    EXPECT_EQ(0, unit.getMeanTA());
}

TEST_F(VQMStateTest, getSpatialActivity_monoChromeImages_zeroSA)
{
    EXPECT_EQ(0, getSpatialActivity(cv::Mat(10, 10, CV_8UC3, cv::Scalar::all(0))));
    EXPECT_EQ(0, getSpatialActivity(cv::Mat(10, 10, CV_8UC3, cv::Scalar::all(100))));
    EXPECT_EQ(0, getSpatialActivity(cv::Mat(10, 10, CV_8UC3, cv::Scalar::all(255))));
}

TEST_F(VQMStateTest, getSpatialActivity_randomImage_nonZeroSA)
{
    const auto actualSA = getSpatialActivity(imageWithActivity_);

    EXPECT_NE(0, actualSA);
}

TEST_F(VQMStateTest, DISABLED_getSpatialActivity_realImagesAkiyo_correctSA)
{
    EXPECT_EQ(64.6736, getSpatialActivity(akiyo0_));
    EXPECT_EQ(64.6685, getSpatialActivity(akiyo1_));
}

TEST_F(VQMStateTest, DISABLED_getSpatialActivity_realImagesMobile_correctSA)
{
    EXPECT_EQ(166.638, getSpatialActivity(mobile0_));
    EXPECT_EQ(166.878, getSpatialActivity(mobile1_));
}

TEST_F(VQMStateTest, getMeanSpatialActivity_spatialActivityDiffButTooLongAgo_zeroSA)
{
    VQMState unit(0);
    cv::Mat monochromeImage(10, 10, CV_8UC3, cv::Scalar::all(0));

    unit.update(imageWithActivity_);
    for (int i = 0; i < VQMState::activityHistorySize; ++i)
    {
        EXPECT_NE(0, unit.getMeanSA());
        unit.update(monochromeImage);
    }

    EXPECT_EQ(0, unit.getSA());
    EXPECT_EQ(0, unit.getMeanSA());
}

TEST_F(VQMStateTest, constructor_noSkipFrames_equalToMax)
{
    const VQMState unit(0);

    EXPECT_EQ(unit.getFrameRateMax(), unit.getFrameRate());
}
