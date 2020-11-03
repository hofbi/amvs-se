#ifndef SINGLE_ENCODER_MDA_H
#define SINGLE_ENCODER_MDA_H

#include "vqm_state.h"

namespace lmt
{
namespace quality
{
class MDA
{
  public:
    MDA(uint16_t width, uint16_t height) noexcept;

    void updateFrame(const cv::Mat& image);

    bool isNewCodingMode() const noexcept;
    uint8_t getSkipFrames() const noexcept;
    uint16_t getWidth() const noexcept;
    uint16_t getHeight() const noexcept;

  private:
    uint16_t width_{0};
    uint16_t height_{0};
    uint8_t skipFrames_{0};
    VQMState vqmState_{skipFrames_};
    bool newCodingMode_{false};
    uint8_t minimumWaitingTimeCounter_{0};
};
}  // namespace quality
}  // namespace lmt

#endif  // SINGLE_ENCODER_MDA_H
