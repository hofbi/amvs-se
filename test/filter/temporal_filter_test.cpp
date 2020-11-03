#include "temporal_filter.h"

#include <boost/make_shared.hpp>

#include "test_utils.h"

using namespace lmt::filter;
using namespace lmt::util;

class TemporalFilterTest : public ::testing::Test
{
  protected:
    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    std::vector<cv_bridge::CvImagePtr> images_;
    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    cv_bridge::CvImagePtr black_{nullptr};
    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    cv_bridge::CvImagePtr white_{nullptr};

    TemporalFilterTest()
        : black_(boost::make_shared<cv_bridge::CvImage>(std_msgs::Header(),
                                                        "bgr8",
                                                        cv::Mat(25, 25, CV_8UC3, cv::Scalar::all(0)))),
          white_(boost::make_shared<cv_bridge::CvImage>(std_msgs::Header(),
                                                        "bgr8",
                                                        cv::Mat(25, 25, CV_8UC3, cv::Scalar::all(255))))
    {
        for (size_t i = 0; i < 10; ++i)
        {
            std_msgs::Header header;
            header.seq = i;
            images_.push_back(boost::make_shared<cv_bridge::CvImage>(header, "", cv::Mat()));
        }
    }
};

TEST_F(TemporalFilterTest, filter_noSkipFrames_noImageSkipped)
{
    TemporalFilter unit(0);

    auto filtered = unit.filter(black_);
    compareCvImagePtr(black_, filtered);

    filtered = unit.filter(white_);
    compareCvImagePtr(white_, filtered);
}

TEST_F(TemporalFilterTest, filter_oneSkipFrames_oneImageSkipped)
{
    TemporalFilter unit(1);

    auto filtered = unit.filter(black_);
    compareCvImagePtr(black_, filtered);

    filtered = unit.filter(white_);
    compareCvImagePtr(black_, filtered);

    filtered = unit.filter(white_);
    compareCvImagePtr(white_, filtered);
}

TEST_F(TemporalFilterTest, filter_sequence_noSkipFrames_noImageSkipped)
{
    TemporalFilter unit(0);

    for (auto image = images_.begin(); image != images_.end(); ++image)
    {
        auto filtered = unit.filter(*image);

        EXPECT_EQ(*image, filtered);
        EXPECT_EQ((*image)->header.seq, filtered->header.seq);

        ++image;
        filtered = unit.filter(*image);
        EXPECT_EQ(*image, filtered);
        EXPECT_EQ((*image)->header.seq, filtered->header.seq);
    }
}

TEST_F(TemporalFilterTest, filter_sequence_oneSkipFrames_oneImageSkipped)
{
    TemporalFilter unit(1);

    for (auto image = images_.begin(); image != images_.end(); ++image)
    {
        auto filtered = unit.filter(*image);
        const auto seq = (*image)->header.seq;

        EXPECT_EQ(*image, filtered);
        EXPECT_EQ(seq, filtered->header.seq);

        ++image;
        filtered = unit.filter(*image);
        EXPECT_NE(*image, filtered);
        EXPECT_NE((*image)->header.seq, filtered->header.seq);
        EXPECT_EQ(seq, filtered->header.seq);
    }
}
