#ifndef SINGLE_ENCODER_SUPER_FRAME_DEMUX_H
#define SINGLE_ENCODER_SUPER_FRAME_DEMUX_H

#include <vector>

#include "encoder_statistics.h"
#include "mdvqm.h"
#include "stvqm.h"

namespace lmt
{
namespace statistics
{
struct FrameStats;
}  // namespace statistics
namespace mux
{
class SuperFrameDemux
{
  public:
    using SuperFrameLayout = std::vector<cv::Rect>;

    SuperFrameDemux(SuperFrameLayout superFrameLayout,
                    std::vector<quality::MDVQM> mdvqmList,
                    std::vector<quality::STVQM> stvqmList) noexcept;

    std::vector<cv::Mat> demuxSuperFrame(const cv::Mat& superFrame) const;
    statistics::FrameStats applyVQMForIndividualFrame(int index,
                                                      const cv::Mat& frame,
                                                      const cv::Mat& rawFrame,
                                                      const statistics::FrameStats& stats);

  private:
    SuperFrameLayout superFrameLayout_;
    std::vector<quality::MDVQM> mdvqmList_;
    std::vector<quality::STVQM> stvqmList_;
};

SuperFrameDemux::SuperFrameLayout createSuperFrameLayoutFromStatistics(
    const std::vector<statistics::EncoderStatistics>& encoderStatisticsList) noexcept;

template <typename T>
std::vector<T> createVQMListFromStatistics(
    const std::vector<statistics::EncoderStatistics>& encoderStatisticsList) noexcept
{
    std::vector<T> vqmList;

    for (const auto& encoderStatistics : encoderStatisticsList)
    {
        vqmList.emplace_back(encoderStatistics.getSkipFrames());
    }

    return vqmList;
}

}  // namespace mux
}  // namespace lmt

#endif  // SINGLE_ENCODER_SUPER_FRAME_DEMUX_H
