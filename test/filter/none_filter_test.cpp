#include "none_filter.h"

#include <boost/make_shared.hpp>

#include "test_utils.h"

using namespace lmt::filter;
using namespace lmt::util;

TEST(NoneFilterTest, filter_noFiltering_originalImage)
{
    NoneFilter unit;
    auto image = boost::make_shared<cv_bridge::CvImage>(std_msgs::Header(), "bgr8", cv::Mat(25, 25, CV_8UC3));

    auto filtered = unit.filter(image);

    compareCvImagePtr(image, filtered);
}
