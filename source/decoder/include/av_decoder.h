#ifndef SINGLE_ENCODER_X264DECODER_H
#define SINGLE_ENCODER_X264DECODER_H

#include <functional>

#include "video_decoder.h"
extern "C" {
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

namespace lmt
{
namespace decoder
{
class AvDecoder final : public VideoDecoder
{
  public:
    explicit AvDecoder(AVCodecID codecId);
    ~AvDecoder() override;

  private:
    cv::Mat decodeImpl(const std::unique_ptr<uint8_t[]>& encodedData, uint32_t frameSize) override;

  private:
    std::unique_ptr<AVCodecContext, std::function<void(AVCodecContext*)>> codecContext_{
        nullptr,
        [](AVCodecContext* context) { avcodec_free_context(&context); }};
    std::unique_ptr<SwsContext, decltype(&sws_freeContext)> swsContext_{nullptr, sws_freeContext};
};
}  // namespace decoder
}  // namespace lmt

#endif  // SINGLE_ENCODER_X264DECODER_H
