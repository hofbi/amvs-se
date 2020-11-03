#ifndef SINGLE_ENCODER_MDVQM_H
#define SINGLE_ENCODER_MDVQM_H

#include "mdvqm_param.h"
#include "vqm_state.h"

namespace lmt
{
namespace quality
{
class MDVQM
{
  public:
    explicit MDVQM(uint8_t skipFrames) noexcept;

    void setUnscaledSize(uint32_t size) noexcept;
    double calculate(const cv::Mat& image, uint32_t encodedImgSize);

  private:
    MDVQMParam param_;
    VQMState vqmState_;
    uint32_t sizeUnscaled_{0};
};

double getSpatialCorrectionFactor(double spatialScaleFactor, double bs, double sa) noexcept;
double getTemporalCorrectionFactor(double frameRate, double frameRateMax, double bt, double ta) noexcept;
}  // namespace quality
}  // namespace lmt

#endif  // SINGLE_ENCODER_MDVQM_H
