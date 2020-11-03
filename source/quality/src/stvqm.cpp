#include "stvqm.h"

#include <opencv2/core.hpp>

namespace lmt
{
namespace quality
{

STVQM::STVQM(uint8_t skipFrames) noexcept : vqmState_(skipFrames) {}

double STVQM::calculate(const cv::Mat& rawImage, const cv::Mat& image)
{
    vqmState_.update(image);

    const auto ta = vqmState_.getMeanTA();
    const auto sa = vqmState_.getMeanSA();
    const auto spsnr = cv::PSNR(rawImage, image);

    const auto svqm = 100.0 / (1 + std::exp(-1 * (spsnr + param_.ws * sa + param_.wt * ta - param_.mu) / param_.s));
    const auto temporalCorrectionFactor =
        getTemporalCorrectionFactor(vqmState_.getFrameRate(), vqmState_.getFrameRateMax(), param_.a, param_.b, ta);

    return svqm * temporalCorrectionFactor;
}

double getTemporalCorrectionFactor(double frameRate, double frameRateMax, double a, double b, double ta) noexcept
{
    return (1.0 + a * std::pow(ta, b)) / (1.0 + a * std::pow(ta, b) * (frameRateMax / frameRate));
}
}  // namespace quality
}  // namespace lmt
