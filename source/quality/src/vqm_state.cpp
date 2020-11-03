#include "vqm_state.h"

#include <numeric>

#include <opencv2/imgproc.hpp>

namespace lmt
{
namespace quality
{
VQMState::VQMState(uint8_t skipFrames) noexcept
    : frameRate_(getCurrentFrameRateFromSkipFrames(frameRateMax_, skipFrames)),
      spatialActivityHistory_(activityHistorySize, 0.0),
      temporalActivityHistory_(activityHistorySize, 0.0)
{
}

double VQMState::getFrameRateMax() const noexcept
{
    return frameRateMax_;
}

double VQMState::getFrameRate() const noexcept
{
    return frameRate_;
}

void VQMState::update(const cv::Mat& image)
{
    const auto yChannel = getYChannel(image);
    spatialActivityHistory_.push_back(getSpatialActivity(yChannel));
    temporalActivityHistory_.push_back(getTemporalActivity(yChannel, oldImage_));
    oldImage_ = yChannel;
}

double VQMState::getMeanTA() const noexcept
{
    return std::accumulate(temporalActivityHistory_.begin(), temporalActivityHistory_.end(), 0.0) /
           temporalActivityHistory_.size();
}

double VQMState::getMeanSA() const noexcept
{
    return std::accumulate(spatialActivityHistory_.begin(), spatialActivityHistory_.end(), 0.0) /
           spatialActivityHistory_.size();
}

double VQMState::getTA() const noexcept
{
    return temporalActivityHistory_.back();
}

double VQMState::getSA() const noexcept
{
    return spatialActivityHistory_.back();
}

double getCurrentFrameRateFromSkipFrames(double maxFrameRate, uint8_t skipFrames) noexcept
{
    return maxFrameRate / (skipFrames + 1);
}

double getSpatialActivity(const cv::Mat& image)
{
    cv::Mat gradX;
    cv::Mat gradY;
    cv::Mat grad;

    cv::Sobel(getYChannel(image), gradX, CV_32F, 1, 0);
    cv::Sobel(getYChannel(image), gradY, CV_32F, 0, 1);
    cv::add(gradX.mul(gradX), gradY.mul(gradY), grad);
    cv::sqrt(grad, grad);

    return getStdDev(grad);
}

double getTemporalActivity(const cv::Mat& image, const cv::Mat& oldImage)
{
    if (oldImage.empty())
    {
        return 0;
    }

    cv::Mat diff;
    cv::absdiff(getYChannel(image), getYChannel(oldImage), diff);
    return getStdDev(diff);
}

double getStdDev(const cv::Mat& image)
{
    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(image, mean, stddev);

    return stddev[0];
}

cv::Mat getYChannel(const cv::Mat& image)
{
    if (image.channels() == 1)
    {
        return image;
    }

    cv::Mat yChannel;
    cv::cvtColor(image, yChannel, cv::COLOR_BGR2GRAY);
    return yChannel;
}

}  // namespace quality
}  // namespace lmt
