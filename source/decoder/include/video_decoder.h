#ifndef SINGLE_ENCODER_VIDEODECODER_H
#define SINGLE_ENCODER_VIDEODECODER_H

#include <memory>

#include <opencv2/core/mat.hpp>

namespace lmt
{
namespace decoder
{
class VideoDecoder
{
  public:
    virtual ~VideoDecoder();

    cv::Mat decode(const std::unique_ptr<uint8_t[]>& encodedData, uint32_t frameSize);

  private:
    virtual cv::Mat decodeImpl(const std::unique_ptr<uint8_t[]>& encodedData, uint32_t frameSize) = 0;
};
}  // namespace decoder
}  // namespace lmt

#endif  // SINGLE_ENCODER_VIDEODECODER_H
