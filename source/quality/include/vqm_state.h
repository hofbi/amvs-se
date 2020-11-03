#ifndef SINGLE_ENCODER_VQM_STATE_H
#define SINGLE_ENCODER_VQM_STATE_H

#include <boost/circular_buffer.hpp>
#include <opencv2/core/mat.hpp>

namespace lmt
{
namespace quality
{
class VQMState
{
  public:
    explicit VQMState(uint8_t skipFrames) noexcept;

    double getFrameRateMax() const noexcept;
    double getFrameRate() const noexcept;
    double getMeanSA() const noexcept;
    double getMeanTA() const noexcept;
    double getSA() const noexcept;
    double getTA() const noexcept;

    void update(const cv::Mat& image);

  public:
    static constexpr uint8_t activityHistorySize{5};

  private:
    double frameRateMax_{20.0};
    double frameRate_{20.0};
    boost::circular_buffer<double> spatialActivityHistory_;
    boost::circular_buffer<double> temporalActivityHistory_;
    cv::Mat oldImage_;
};

double getCurrentFrameRateFromSkipFrames(double maxFrameRate, uint8_t skipFrames) noexcept;

double getSpatialActivity(const cv::Mat& image);
double getTemporalActivity(const cv::Mat& image, const cv::Mat& oldImage);
double getStdDev(const cv::Mat& image);
cv::Mat getYChannel(const cv::Mat& image);
}  // namespace quality
}  // namespace lmt

#endif  // SINGLE_ENCODER_VQM_STATE_H
