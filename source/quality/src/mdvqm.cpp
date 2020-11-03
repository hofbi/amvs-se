#include "mdvqm.h"

namespace lmt
{
namespace quality
{
double MDVQM::calculate(const cv::Mat& image, uint32_t encodedImgSize)
{
    const auto rawImageSize = static_cast<double>(image.elemSize() * image.total());
    const auto spatialScaleFactor = rawImageSize / sizeUnscaled_;

    vqmState_.update(image);

    const auto sa = vqmState_.getMeanSA();
    const auto ta = vqmState_.getMeanTA();

    const auto spatialCorrectionFactor = getSpatialCorrectionFactor(spatialScaleFactor, param_.bs, sa);
    const auto temporalCorrectionFactor =
        getTemporalCorrectionFactor(vqmState_.getFrameRate(), vqmState_.getFrameRateMax(), param_.bt, ta);

    const auto ps = std::exp(-1 * param_.as * sa);
    const auto pt = std::exp(-1 * param_.at * ta);
    const auto bpp0 = static_cast<double>(encodedImgSize) * vqmState_.getFrameRateMax() / rawImageSize;
    const auto bpp =
        bpp0 * std::pow(spatialScaleFactor, ps) * std::pow(vqmState_.getFrameRateMax() / vqmState_.getFrameRate(), pt);
    const auto m = std::pow(ta, param_.a[0]) * std::pow(sa, param_.a[1]) * param_.a[2];
    const auto snrvq =
        100.0 / (1 + std::exp(-1 * (m * std::log(bpp) + param_.a[3] * sa + param_.a[4] * ta + param_.a[5])));

    return snrvq * std::min(spatialCorrectionFactor, temporalCorrectionFactor);
}

MDVQM::MDVQM(uint8_t skipFrames) noexcept : vqmState_(skipFrames) {}

void MDVQM::setUnscaledSize(uint32_t size) noexcept
{
    sizeUnscaled_ = size;
}

double getSpatialCorrectionFactor(double spatialScaleFactor, double bs, double sa) noexcept
{
    return std::pow(spatialScaleFactor, bs * sa);
}

double getTemporalCorrectionFactor(double frameRate, double frameRateMax, double bt, double ta) noexcept
{
    return (frameRate / frameRateMax) * ((1 + bt * frameRateMax / ta) / (1 + bt * frameRate / ta));
}
}  // namespace quality
}  // namespace lmt
