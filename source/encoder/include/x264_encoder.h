#ifndef SINGLE_ENCODER_X264ENCODER_H
#define SINGLE_ENCODER_X264ENCODER_H

#include <memory>

#include <x264.h>

#include "video_encoder.h"
extern "C" {
#include <libavutil/pixfmt.h>
#include <libswscale/swscale.h>
}

class SwsContext;

namespace lmt
{
namespace encoder
{
class X264Encoder final : public VideoEncoder
{
  public:
    X264Encoder(uint16_t outWidth, uint16_t outHeight, uint8_t qp);
    X264Encoder(uint16_t outWidth, uint16_t outHeight, int bitrate);
    ~X264Encoder() override;

  private:
    statistics::FrameStats encodeImpl(const cv_bridge::CvImageConstPtr& inputImg,
                                      std::unique_ptr<uint8_t[]>& encodedData) override;

    static x264_param_t getPresetParam(uint16_t outWidth, uint16_t outHeight);

    std::unique_ptr<x264_t, decltype(&x264_encoder_close)> encoder_{nullptr, x264_encoder_close};
    x264_picture_t picIn_{};
    x264_picture_t picOut_{};
    std::unique_ptr<SwsContext, decltype(&sws_freeContext)> swsContext_{nullptr, sws_freeContext};
    int nheader_{};
    x264_nal_t* nals_{nullptr};
    int numNals_{};
};
}  // namespace encoder
}  // namespace lmt

#endif  // SINGLE_ENCODER_X264ENCODER_H
