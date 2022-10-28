#include "super_frame_demux.h"

#include <opencv2/core.hpp>

#include "frame_stats.h"

using namespace lmt::quality;

namespace lmt
{
namespace mux
{

SuperFrameDemux::SuperFrameDemux(SuperFrameDemux::SuperFrameLayout superFrameLayout,
                                 std::vector<quality::MDVQM> mdvqmList,
                                 std::vector<quality::STVQM> stvqmList) noexcept
    : superFrameLayout_(std::move(superFrameLayout)), mdvqmList_(std::move(mdvqmList)), stvqmList_(std::move(stvqmList))
{
}

std::vector<cv::Mat> SuperFrameDemux::demuxSuperFrame(const cv::Mat& superFrame) const
{
    std::vector<cv::Mat> individualFrames;

    for (const auto& layout : superFrameLayout_)
    {
        individualFrames.push_back(superFrame(layout));
    }

    return individualFrames;
}

statistics::FrameStats SuperFrameDemux::applyVQMForIndividualFrame(int index,
                                                                   const cv::Mat& frame,
                                                                   const cv::Mat& rawFrame,
                                                                   const statistics::FrameStats& stats)
{
    const auto rawImgSize = frame.elemSize() * frame.total();
    auto newStats = stats;

    auto& mdvqm = mdvqmList_[index];
    mdvqm.setUnscaledSize(rawImgSize);
    newStats.mdvqmScores.push_back(mdvqm.calculate(frame, stats.encodedFrameSize));

    auto& stvqm = stvqmList_[index];
    newStats.stvqmScores.push_back(stvqm.calculate(rawFrame, frame));

    newStats.psnrScores.push_back(cv::PSNR(rawFrame, frame));

    return newStats;
}

SuperFrameDemux::SuperFrameLayout createSuperFrameLayoutFromStatistics(
    const std::vector<statistics::EncoderStatistics>& encoderStatisticsList) noexcept
{
    SuperFrameDemux::SuperFrameLayout superFrameLayout;

    auto xPos{0};
    for (const auto& encoderStatistics : encoderStatisticsList)
    {
        superFrameLayout.emplace_back(xPos, 0, encoderStatistics.getWidth(), encoderStatistics.getHeight());
        xPos += encoderStatistics.getWidth();
    }

    return superFrameLayout;
}
}  // namespace mux
}  // namespace lmt
