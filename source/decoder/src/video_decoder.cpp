#include "video_decoder.h"

using namespace lmt::decoder;

VideoDecoder::~VideoDecoder() = default;

// NOLINTNEXTLINE(modernize-avoid-c-arrays)
cv::Mat VideoDecoder::decode(const std::unique_ptr<uint8_t[]>& encodedData, uint32_t frameSize)
{
    return decodeImpl(encodedData, frameSize);
}
