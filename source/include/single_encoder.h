#ifndef SINGLE_ENCODER_SINGLE_ENCODER_H
#define SINGLE_ENCODER_SINGLE_ENCODER_H

#include <ros/subscriber.h>

#include "statistics_collection.h"
#include "super_frame_demux.h"
#include "super_frame_mux.h"

namespace lmt
{
namespace encoder
{
class VideoEncoder;
}  // namespace encoder
namespace statistics
{
struct FrameStats;
}
class SingleEncoder
{
  public:
    SingleEncoder(ros::NodeHandle& pnh, std::vector<statistics::EncoderStatistics> statistics);

    void superFrameCallback(const std::vector<statistics::FrameStats>& stats,
                            const cv::Mat& superFrame,
                            const std::vector<cv::Mat>& individualRawImages);

    void writeStatistics();

  private:
    static std::unique_ptr<encoder::VideoEncoder> createEncoder(ros::NodeHandle& pnh,
                                                                const statistics::EncoderStatistics& statistics);

    std::vector<ros::Subscriber> subscribers_;
    statistics::StatisticsCollection statisticsCollection_;
    mux::SuperFrameMux superFrameMux_;
    mux::SuperFrameDemux superFrameDemux_;
};
}  // namespace lmt

#endif  // SINGLE_ENCODER_SINGLE_ENCODER_H
