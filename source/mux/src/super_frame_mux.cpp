#include "super_frame_mux.h"

#include <boost/smart_ptr/make_shared.hpp>

#include "av_decoder.h"
#include "buffer_utils.h"
#include "vqm_state.h"
#include "x264_encoder.h"

using namespace lmt::statistics;

namespace lmt
{
namespace mux
{
void SuperFrameMux::init(uint8_t numberOfIndividualFrames,
                         std::unique_ptr<encoder::VideoEncoder> encoder,
                         std::unique_ptr<decoder::VideoDecoder> decoder) noexcept
{
    superFrameBuffer_ = SuperFrameBuffer(numberOfIndividualFrames);
    frameStatsBufferList_ = FrameStatsBufferList(numberOfIndividualFrames);

    encoder_ = std::move(encoder);
    decoder_ = std::move(decoder);
}

void SuperFrameMux::addImageSegment(int index, const cv_bridge::CvImageConstPtr& img, const FrameStats& stats)
{
    superFrameBuffer_[index].push_back(img->image);
    frameStatsBufferList_[index].push_back(stats);

    if (std::none_of(superFrameBuffer_.begin(), superFrameBuffer_.end(), [](const FrameBuffer& segmentBuffer) {
            return segmentBuffer.empty();
        }))
    {
        const auto allIndividualRawImages = getFirstOfEveryBuffer(superFrameBuffer_);
        const auto superFrame = moveFramesIntoSuperFrame(allIndividualRawImages);
        vqmState_->update(superFrame);
        const auto superFramePtr =
            boost::make_shared<cv_bridge::CvImage>(std_msgs::Header(), img->encoding, superFrame);

        // NOLINTNEXTLINE(modernize-avoid-c-arrays)
        auto encodedData = std::make_unique<uint8_t[]>(img->image.elemSize() * img->image.total());
        auto superFrameStats = encoder_->encode(superFramePtr, encodedData);
        superFrameStats.temporalActivity = vqmState_->getTA();
        superFrameStats.spatialActivity = vqmState_->getSA();

        const auto decodedSuperFrame = decoder_->decode(encodedData, superFrameStats.encodedFrameSize);

        auto allFrameStats = getFirstOfEveryBuffer(frameStatsBufferList_);
        allFrameStats.push_back(superFrameStats);

        superFrameCallback_(allFrameStats, decodedSuperFrame, allIndividualRawImages);

        popFirstOfEveryBuffer(frameStatsBufferList_);
        popFirstOfEveryBuffer(superFrameBuffer_);
    }
}

SuperFrameMux::SuperFrameMux(SuperFrameMux::SuperFrameCallback superFrameCallback,
                             std::unique_ptr<quality::VQMState> vqmState)
    : superFrameCallback_(std::move(superFrameCallback)), vqmState_(std::move(vqmState))
{
}

cv::Mat moveFramesIntoSuperFrame(const std::vector<cv::Mat>& individualRawImages)
{
    auto it = individualRawImages.begin();
    cv::Mat superFrame = *it;

    ++it;
    for (; it != individualRawImages.end(); ++it)
    {
        addSegmentToSuperFrame(superFrame, *it);
    }

    return superFrame;
}

void addSegmentToSuperFrame(cv::Mat& superFrame, const cv::Mat& segment)
{
    cv::Mat tmpSegment = segment;
    if (segment.rows < superFrame.rows)
    {
        tmpSegment = cv::Mat(superFrame.rows, segment.cols, superFrame.type(), cv::Scalar::all(0));
        segment.copyTo(tmpSegment(cv::Rect(0, 0, segment.cols, segment.rows)));
    }
    cv::hconcat(superFrame, tmpSegment, superFrame);
}

}  // namespace mux
}  // namespace lmt
