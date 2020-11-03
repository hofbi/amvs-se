#include "single_encoder.h"

#include <utility>

#include <ros/ros.h>

#include "av_decoder.h"
#include "base_image_filter_factory.h"
#include "channel_filter.h"
#include "image_callback.h"
#include "spatial_filter.h"
#include "temporal_filter.h"
#include "x264_encoder.h"

using namespace lmt;
using namespace lmt::statistics;
using namespace lmt::decoder;
using namespace lmt::encoder;
using namespace lmt::filter;
using namespace lmt::mux;
using namespace lmt::data;
using namespace lmt::quality;

SingleEncoder::SingleEncoder(ros::NodeHandle& pnh, std::vector<EncoderStatistics> statistics)
    : statisticsCollection_(std::move(statistics)),
      superFrameMux_([this](auto&&... args) { superFrameCallback(std::forward<decltype(args)>(args)...); },
                     std::make_unique<VQMState>(0)),
      superFrameDemux_(createSuperFrameLayoutFromStatistics(statisticsCollection_.getStats()),
                       createVQMListFromStatistics<MDVQM>(statisticsCollection_.getStats()),
                       createVQMListFromStatistics<STVQM>(statisticsCollection_.getStats()))
{
    auto superFrameStat = EncoderStatistics::createSuperFrameStatistics(statisticsCollection_.getStats());
    auto superFrameEncoder = createEncoder(pnh, superFrameStat);
    superFrameMux_.init(statisticsCollection_.getStats().size(),
                        std::move(superFrameEncoder),
                        std::make_unique<AvDecoder>(AV_CODEC_ID_H264));
    statisticsCollection_.addEncoderStatistic(std::move(superFrameStat));

    for (const auto& stat : statisticsCollection_.getStats())
    {
        auto encoder = createEncoder(pnh, stat);

        std::vector<std::shared_ptr<BaseImageFilter>> filters;
        filters.push_back(std::make_unique<TemporalFilter>(stat.getSkipFrames()));
        filters.push_back(std::make_unique<SpatialFilter>(stat.getWidth(), stat.getHeight()));
        filters.push_back(std::make_unique<ChannelFilter>(stat.isDropChroma()));
        filters.push_back(createFilterFromParameter(stat.getFilterParameter()));

        constexpr auto queueSize = 30;
        subscribers_.push_back(pnh.subscribe<sensor_msgs::Image>(stat.getTopicName(),
                                                                 queueSize,
                                                                 ImageCallback(
                                                                     stat.getEncoderId(),
                                                                     [objectPtr = &superFrameMux_](auto&&... args) {
                                                                         objectPtr->addImageSegment(
                                                                             std::forward<decltype(args)>(args)...);
                                                                     },
                                                                     std::move(encoder),
                                                                     std::make_unique<AvDecoder>(AV_CODEC_ID_H264),
                                                                     std::move(filters),
                                                                     std::make_unique<VQMState>(stat.getSkipFrames()),
                                                                     std::make_unique<MDVQM>(stat.getSkipFrames()),
                                                                     std::make_unique<STVQM>(stat.getSkipFrames()))));
        ROS_INFO_STREAM("Subscribe to image topic " << static_cast<int>(stat.getEncoderId()) << ": "
                                                    << stat.getTopicName());
    }
}

void SingleEncoder::writeStatistics()
{
    constexpr auto outFile = "stats.json";
    if (!statisticsCollection_.hasData())
    {
        std::cerr << "\nNot all configured encoders received data!\nNot writing a stats file.\n\n";
        return;
    }

    std::cout << "\nWrite statistics to file: " << outFile << "\n\n";
    statisticsCollection_.writeStatsFile(outFile);
}

void SingleEncoder::superFrameCallback(const std::vector<FrameStats>& stats,
                                       const cv::Mat& superFrame,
                                       const std::vector<cv::Mat>& individualRawImages)
{
    const auto individualFrames = superFrameDemux_.demuxSuperFrame(superFrame);

    auto frameStats = stats.begin();
    auto rawImg = individualRawImages.begin();
    for (const auto& frame : individualFrames)
    {
        const auto index = std::distance(stats.begin(), frameStats);

        const auto newStats = superFrameDemux_.applyVQMForIndividualFrame(index, frame, *rawImg, *frameStats);

        statisticsCollection_.addFrameStatsById(index, newStats);
        ++frameStats;
        ++rawImg;
    }

    statisticsCollection_.addFrameStatsById(static_cast<int>(stats.size() - 1), stats.back());
}

std::unique_ptr<encoder::VideoEncoder> SingleEncoder::createEncoder(ros::NodeHandle& pnh,
                                                                    const statistics::EncoderStatistics& statistics)
{
    if (pnh.param("cbr_mode", false))
    {
        return std::make_unique<X264Encoder>(statistics.getWidth(), statistics.getHeight(), statistics.getBitrate());
    }

    const uint8_t qp = pnh.param("qp", 25);
    return std::make_unique<X264Encoder>(statistics.getWidth(), statistics.getHeight(), qp);
}
