#include "spatial_filter.h"

#include <boost/make_shared.hpp>

#include "test_utils.h"

using namespace lmt::filter;
using namespace lmt::util;

class SpatialFilterTest : public ::testing::Test
{
  protected:
    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    int imgWidth_{32};
    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    int imgHeight_{18};
    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    cv_bridge::CvImagePtr image_;

    SpatialFilterTest()
        : image_(
              boost::make_shared<cv_bridge::CvImage>(std_msgs::Header(), "", cv::Mat(imgHeight_, imgWidth_, CV_8UC3)))
    {
    }
};

TEST_F(SpatialFilterTest, filter_equalInputAndOutputSize_noResizedFrame)
{
    SpatialFilter unit(imgWidth_, imgHeight_);

    auto filtered = unit.filter(image_);

    compareCvImagePtr(image_, filtered);
}

TEST_F(SpatialFilterTest, filter_differentInputAndOutputSize_resizedFrame)
{
    const auto outWidth = imgWidth_ / 2;
    const auto outHeight = imgHeight_ / 2;
    const auto* const oldEnd = image_->image.dataend;

    SpatialFilter unit(outWidth, outHeight);

    auto filtered = unit.filter(image_);

    EXPECT_EQ(image_, filtered);
    EXPECT_EQ(outWidth, filtered->image.cols);
    EXPECT_EQ(outHeight, filtered->image.rows);

    EXPECT_EQ(image_->image.data, filtered->image.data);
    EXPECT_EQ(image_->image.dataend, filtered->image.dataend);
    EXPECT_NE(oldEnd, filtered->image.dataend);
}
