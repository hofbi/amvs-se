#include "video_encoder.h"

using namespace lmt::encoder;

VideoEncoder::~VideoEncoder() = default;

VideoEncoder::VideoEncoder(uint16_t outWidth, uint16_t outHeight) noexcept
    : frameCounter_(0), outWidth_(outWidth), outHeight_(outHeight)
{
}

lmt::statistics::FrameStats VideoEncoder::encode(
    const cv_bridge::CvImageConstPtr& inputImg,
    std::unique_ptr<uint8_t[]>& encodedData)  // NOLINT(modernize-avoid-c-arrays)
{
    return encodeImpl(inputImg, encodedData);
}
