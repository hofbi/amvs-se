#ifndef SINGLE_ENCODER_SUPER_FRAME_MUX_H
#define SINGLE_ENCODER_SUPER_FRAME_MUX_H

#include <deque>
#include <memory>

#include <cv_bridge/cv_bridge.h>

#include "frame_stats.h"

namespace lmt
{
namespace encoder
{
class VideoEncoder;
}
namespace decoder
{
class VideoDecoder;
}
namespace quality
{
class VQMState;
}
namespace mux
{
class SuperFrameMux
{
  public:
    using SuperFrameCallback = std::function<void(const std::vector<statistics::FrameStats>& stats,
                                                  const cv::Mat& superFrame,
                                                  const std::vector<cv::Mat>& individualRawImages)>;

    using FrameBuffer = std::deque<cv::Mat>;
    using SuperFrameBuffer = std::vector<FrameBuffer>;

    using FrameStatsBufferList = std::vector<std::deque<statistics::FrameStats>>;

    SuperFrameMux(SuperFrameCallback superFrameCallback, std::unique_ptr<quality::VQMState> vqmState);

    void init(uint8_t numberOfIndividualFrames,
              std::unique_ptr<encoder::VideoEncoder> encoder,
              std::unique_ptr<decoder::VideoDecoder> decoder) noexcept;

    void addImageSegment(int index, const cv_bridge::CvImageConstPtr& img, const statistics::FrameStats& stats);

  private:
    SuperFrameCallback superFrameCallback_;
    std::unique_ptr<quality::VQMState> vqmState_{nullptr};
    SuperFrameBuffer superFrameBuffer_;
    FrameStatsBufferList frameStatsBufferList_;
    std::shared_ptr<encoder::VideoEncoder> encoder_{nullptr};
    std::shared_ptr<decoder::VideoDecoder> decoder_{nullptr};
};

cv::Mat moveFramesIntoSuperFrame(const std::vector<cv::Mat>& individualRawImages);
void addSegmentToSuperFrame(cv::Mat& superFrame, const cv::Mat& segment);
}  // namespace mux
}  // namespace lmt

#endif  // SINGLE_ENCODER_SUPER_FRAME_MUX_H
