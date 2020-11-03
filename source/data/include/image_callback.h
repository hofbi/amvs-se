#ifndef SINGLE_ENCODER_IMAGE_CALLBACK_H
#define SINGLE_ENCODER_IMAGE_CALLBACK_H

#include <functional>
#include <memory>

#include <cv_bridge/cv_bridge.h>

namespace lmt
{
namespace encoder
{
class VideoEncoder;
}  // namespace encoder
namespace decoder
{
class VideoDecoder;
}
namespace filter
{
class BaseImageFilter;
}
namespace quality
{
class MDVQM;
class STVQM;
class VQMState;
}  // namespace quality
namespace statistics
{
struct FrameStats;
}
namespace data
{
class ImageCallback
{
  public:
    using ProcessingCallback =
        std::function<void(int index, const cv_bridge::CvImageConstPtr& img, const statistics::FrameStats& stats)>;

    ImageCallback(uint8_t index,
                  ProcessingCallback processingCallback,
                  std::unique_ptr<encoder::VideoEncoder> encoder,
                  std::unique_ptr<decoder::VideoDecoder> decoder,
                  std::vector<std::shared_ptr<filter::BaseImageFilter>> filters,
                  std::unique_ptr<quality::VQMState> vqmState,
                  std::unique_ptr<quality::MDVQM> mdvqm,
                  std::unique_ptr<quality::STVQM> stvqm) noexcept;

    void operator()(const sensor_msgs::ImageConstPtr& msg);

  private:
    uint8_t index_{0};
    ProcessingCallback processingCallback_;
    std::shared_ptr<encoder::VideoEncoder> encoder_{nullptr};
    std::shared_ptr<decoder::VideoDecoder> decoder_{nullptr};
    std::vector<std::shared_ptr<filter::BaseImageFilter>> filters_;
    std::shared_ptr<quality::VQMState> vqmState_{nullptr};
    std::shared_ptr<quality::MDVQM> mdvqm_{nullptr};
    std::shared_ptr<quality::STVQM> stvqm_{nullptr};
};
}  // namespace data
}  // namespace lmt

#endif  // SINGLE_ENCODER_IMAGE_CALLBACK_H
