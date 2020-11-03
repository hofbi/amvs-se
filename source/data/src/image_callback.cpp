#include "image_callback.h"

#include "base_image_filter.h"
#include "mdvqm.h"
#include "stvqm.h"
#include "video_decoder.h"
#include "video_encoder.h"

using namespace lmt::data;
using namespace lmt::encoder;
using namespace lmt::decoder;
using namespace lmt::filter;
using namespace lmt::quality;

ImageCallback::ImageCallback(uint8_t index,
                             ProcessingCallback processingCallback,
                             std::unique_ptr<VideoEncoder> encoder,
                             std::unique_ptr<VideoDecoder> decoder,
                             std::vector<std::shared_ptr<filter::BaseImageFilter>> filters,
                             std::unique_ptr<quality::VQMState> vqmState,
                             std::unique_ptr<quality::MDVQM> mdvqm,
                             std::unique_ptr<quality::STVQM> stvqm) noexcept
    : index_(index),
      processingCallback_(std::move(processingCallback)),
      encoder_(std::move(encoder)),
      decoder_(std::move(decoder)),
      filters_(std::move(filters)),
      vqmState_(std::move(vqmState)),
      mdvqm_(std::move(mdvqm)),
      stvqm_(std::move(stvqm))
{
}

void ImageCallback::operator()(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(msg);

    vqmState_->update(image->image);

    for (auto& filter : filters_)
    {
        image = filter->filter(image);
    }

    const auto rawImgSize = image->image.elemSize() * image->image.total();
    auto encodedData = std::make_unique<uint8_t[]>(rawImgSize);  // NOLINT(modernize-avoid-c-arrays)
    auto stats = encoder_->encode(image, encodedData);

    stats.temporalActivity = vqmState_->getTA();
    stats.spatialActivity = vqmState_->getSA();

    const auto decodedImage = decoder_->decode(encodedData, stats.encodedFrameSize);

    mdvqm_->setUnscaledSize(rawImgSize);
    stats.mdvqmScores.push_back(mdvqm_->calculate(decodedImage, stats.encodedFrameSize));
    stats.stvqmScores.push_back(stvqm_->calculate(image->image, decodedImage));
    stats.psnrScores.push_back(cv::PSNR(image->image, decodedImage));

    processingCallback_(index_, image, stats);
}
