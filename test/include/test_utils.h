#ifndef SINGLE_ENCODER_TEST_UTILS_H
#define SINGLE_ENCODER_TEST_UTILS_H

#include <cv_bridge/cv_bridge.h>
#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>

namespace lmt
{
namespace util
{
inline void compareCvMatPixelWise(const cv::Mat& expectedResult, const cv::Mat& actualResult, bool expectEqual = true)
{
    expectedResult.forEach<cv::Vec3b>([&](const cv::Vec3b& expectedPixel, const int position[]) -> void {
        const auto& actualPixel = actualResult.at<cv::Vec3b>(position);
        for (int i = 0; i < 3; ++i)
        {
            if (expectEqual)
            {
                EXPECT_EQ(expectedPixel[i], actualPixel[i]);
            }
            else
            {
                EXPECT_NE(expectedPixel[i], actualPixel[i]);
            }
        }
    });
}

inline void compareCvMatPixelSize(const cv::Mat& expectedResult, const cv::Mat& actualResult)
{
    const auto expectedSize = expectedResult.elemSize() * expectedResult.total();
    const auto actualSize = actualResult.elemSize() * actualResult.total();

    EXPECT_EQ(expectedSize, actualSize);
}

inline void compareCvMatSize(const cv::Mat& expectedResult, const cv::Mat& actualResult)
{
    EXPECT_EQ(expectedResult.cols, actualResult.cols);
    EXPECT_EQ(expectedResult.rows, actualResult.rows);
}

inline void compareCvImagePtr(const cv_bridge::CvImagePtr& expected, const cv_bridge::CvImagePtr& actual)
{
    EXPECT_EQ(expected, actual);

    compareCvMatPixelSize(expected->image, actual->image);
    compareCvMatSize(expected->image, actual->image);
    compareCvMatPixelWise(expected->image, actual->image);
}
}  // namespace util
}  // namespace lmt

#endif  // SINGLE_ENCODER_TEST_UTILS_H
