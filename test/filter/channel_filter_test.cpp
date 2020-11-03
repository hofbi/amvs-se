#include "channel_filter.h"

#include <boost/make_shared.hpp>

#include "test_utils.h"

using namespace lmt::filter;
using namespace lmt::util;

void checkChannelFilter(const cv_bridge::CvImagePtr& expected, const cv_bridge::CvImagePtr& actual)
{

    EXPECT_EQ("bgr8", actual->encoding);
    EXPECT_EQ(3, actual->image.channels());

    compareCvImagePtr(expected, actual);
}

class ChannelFilterTest : public ::testing::Test
{
  protected:
    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    cv_bridge::CvImagePtr imageBGR8_;
    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    cv_bridge::CvImagePtr imageBGRA_;

    ChannelFilterTest()
        : imageBGR8_(boost::make_shared<cv_bridge::CvImage>(std_msgs::Header(),
                                                            "bgr8",
                                                            cv::Mat(30, 30, CV_8UC3, cv::Scalar(50, 100, 150)))),
          imageBGRA_(boost::make_shared<cv_bridge::CvImage>(std_msgs::Header(),
                                                            "bgra8",
                                                            cv::Mat(30, 30, CV_8UC4, cv::Scalar(50, 100, 150, 200))))
    {
    }
};

TEST_F(ChannelFilterTest, filter_noChromaDropAndBGR_memoryAreaUnchanged)
{
    ChannelFilter unit(false);
    cv::Mat bgrOrig = imageBGR8_->image.clone();

    const auto filtered = unit.filter(imageBGR8_);

    compareCvMatPixelWise(bgrOrig, filtered->image, true);
    checkChannelFilter(imageBGR8_, filtered);
}

TEST_F(ChannelFilterTest, filter_chromaDropAndBGR_memoryAreaUnchanged)
{
    ChannelFilter unit(true);
    cv::Mat bgrOrig = imageBGR8_->image.clone();

    const auto filtered = unit.filter(imageBGR8_);

    compareCvMatPixelWise(bgrOrig, filtered->image, false);
    checkChannelFilter(imageBGR8_, filtered);
}

TEST_F(ChannelFilterTest, filter_noChromaDropAndBGRA_memoryAreaUnchanged)
{
    ChannelFilter unit(false);
    cv::Mat bgrOrig = imageBGR8_->image.clone();

    const auto filtered = unit.filter(imageBGRA_);

    compareCvMatPixelWise(bgrOrig, filtered->image, true);
    checkChannelFilter(imageBGRA_, filtered);
}

TEST_F(ChannelFilterTest, filter_chromaDropAndBGRA_memoryAreaUnchanged)
{
    ChannelFilter unit(true);
    cv::Mat bgrOrig = imageBGR8_->image.clone();

    const auto filtered = unit.filter(imageBGRA_);

    compareCvMatPixelWise(bgrOrig, filtered->image, false);
    checkChannelFilter(imageBGRA_, filtered);
}

TEST_F(ChannelFilterTest, getCvColorConversionCodeFromSensorMsg_bgr8_2bgr)
{
    const auto actual = getCvColorConversionCodeFromSensorMsg(imageBGR8_);

    EXPECT_EQ(cv::COLOR_BGRA2BGR, actual);
}

TEST_F(ChannelFilterTest, getCvColorConversionCodeFromSensorMsg_bgra_2bgr)
{
    const auto actual = getCvColorConversionCodeFromSensorMsg(imageBGRA_);

    EXPECT_EQ(cv::COLOR_BGRA2BGR, actual);
}

TEST_F(ChannelFilterTest, dropChroma_yuvNonZeroInput_uvIsZero)
{
    cv::Mat bgr(30, 30, CV_8UC3, cv::Scalar(50, 100, 150));
    cv::Mat yuv(bgr.rows + bgr.rows / 2, bgr.cols, CV_8UC1);

    cv::cvtColor(bgr, yuv, cv::COLOR_BGR2YUV_I420);
    const auto chromaRect = cv::Rect(0, bgr.rows, bgr.cols, bgr.rows / 2);

    EXPECT_NE(0, cv::countNonZero(yuv(chromaRect)));

    dropChroma(yuv);

    EXPECT_EQ(0, cv::countNonZero(yuv(chromaRect)));

    cv::cvtColor(yuv, imageBGR8_->image, cv::COLOR_YUV2BGR_I420);
    compareCvMatPixelWise(bgr, imageBGR8_->image, false);
    compareCvMatPixelSize(bgr, imageBGR8_->image);
    compareCvMatSize(bgr, imageBGR8_->image);
}
