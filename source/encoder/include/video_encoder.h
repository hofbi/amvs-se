#ifndef SINGLE_ENCODER_VIDEOENCODER_H
#define SINGLE_ENCODER_VIDEOENCODER_H

#include <cstdint>
#include <memory>

#include <cv_bridge/cv_bridge.h>

#include "frame_stats.h"

namespace lmt
{
namespace encoder
{
class VideoEncoder
{
  public:
    VideoEncoder(uint16_t outWidth, uint16_t outHeight) noexcept;
    virtual ~VideoEncoder();

    statistics::FrameStats encode(const cv_bridge::CvImageConstPtr& inputImg, std::unique_ptr<uint8_t[]>& encodedData);

  private:
    virtual statistics::FrameStats encodeImpl(const cv_bridge::CvImageConstPtr& inputImg,
                                              std::unique_ptr<uint8_t[]>& encodedData) = 0;

  protected:
    uint16_t frameCounter_;
    uint16_t outWidth_;
    uint16_t outHeight_;
};
}  // namespace encoder
}  // namespace lmt

#endif  // SINGLE_ENCODER_X264ENCODER_H
