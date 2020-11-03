#include "mda.h"

namespace lmt
{
namespace quality
{
uint8_t MDA::getSkipFrames() const noexcept
{
    return skipFrames_;
}

MDA::MDA(uint16_t width, uint16_t height) noexcept : width_(width), height_(height) {}

uint16_t MDA::getWidth() const noexcept
{
    return width_;
}

uint16_t MDA::getHeight() const noexcept
{
    return height_;
}

bool MDA::isNewCodingMode() const noexcept
{
    return newCodingMode_;
}

void MDA::updateFrame(const cv::Mat& image)
{
    vqmState_.update(image);
    ++minimumWaitingTimeCounter_;

    if (minimumWaitingTimeCounter_ > vqmState_.getFrameRateMax())
    {
        newCodingMode_ = true;
        minimumWaitingTimeCounter_ = 0;
    }
}
}  // namespace quality
}  // namespace lmt
