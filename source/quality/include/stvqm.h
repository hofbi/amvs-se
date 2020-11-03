#ifndef SINGLE_ENCODER_STVQM_H
#define SINGLE_ENCODER_STVQM_H

#include "stvqm_param.h"
#include "vqm_state.h"

namespace lmt
{
namespace quality
{
class STVQM
{
  public:
    explicit STVQM(uint8_t skipFrames) noexcept;

    double calculate(const cv::Mat& rawImage, const cv::Mat& image);

  private:
    STVQMParam param_;
    VQMState vqmState_;
};

double getTemporalCorrectionFactor(double frameRate, double frameRateMax, double a, double b, double ta) noexcept;
}  // namespace quality
}  // namespace lmt

#endif  // SINGLE_ENCODER_STVQM_H
