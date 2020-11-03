#include "encoder_statistics.h"

#include <algorithm>
#include <numeric>

#include "frame_stats.h"

using namespace lmt::statistics;

EncoderStatistics::EncoderStatistics(uint8_t encoderId,
                                     std::string topicName,
                                     uint8_t skipFrames,
                                     uint16_t width,
                                     uint16_t height,
                                     int bitrate,
                                     bool dropChroma,
                                     filter::FilterParameter filter) noexcept
    : encoderId_(encoderId),
      topicName_(std::move(topicName)),
      skipFrames_(skipFrames),
      width_(width),
      height_(height),
      bitrate_(bitrate),
      dropChroma_(dropChroma),
      filter_(std::move(filter))
{
}

void EncoderStatistics::addFrameStats(const FrameStats& encodingStats)
{
    timestamps_.push_back(encodingStats.timestamp);
    frameCounts_.push_back(encodingStats.frameCount);
    encodedFrameSizes_.push_back(encodingStats.encodedFrameSize);
    encodingTimes_.push_back(encodingStats.encodingTime);
    spatialActivities_.push_back(encodingStats.spatialActivity);
    temporalActivities_.push_back(encodingStats.temporalActivity);
    mdvqmScores_.push_back(encodingStats.mdvqmScores);
    stvqmScores_.push_back(encodingStats.stvqmScores);
    psnrScores_.push_back(encodingStats.psnrScores);
}

uint8_t EncoderStatistics::getEncoderId() const noexcept
{
    return encoderId_;
}

const std::string& EncoderStatistics::getTopicName() const noexcept
{
    return topicName_;
}

const std::vector<uint64_t>& EncoderStatistics::getTimestamps() const noexcept
{
    return timestamps_;
}

const lmt::filter::FilterParameter& EncoderStatistics::getFilterParameter() const noexcept
{
    return filter_;
}

const std::vector<uint16_t>& EncoderStatistics::getFrameCounts() const noexcept
{
    return frameCounts_;
}

const std::vector<uint32_t>& EncoderStatistics::getEncodedFrameSizes() const noexcept
{
    return encodedFrameSizes_;
}

const std::vector<int64_t>& EncoderStatistics::getEncodingTimes() const noexcept
{
    return encodingTimes_;
}

uint8_t EncoderStatistics::getSkipFrames() const noexcept
{
    return skipFrames_;
}

uint16_t EncoderStatistics::getWidth() const noexcept
{
    return width_;
}

uint16_t EncoderStatistics::getHeight() const noexcept
{
    return height_;
}

EncoderStatistics EncoderStatistics::createSuperFrameStatistics(const std::vector<EncoderStatistics>& statistics)
{
    if (statistics.empty())
    {
        return {0, "combined", 0, 0, 0, 0, false, {}};
    }

    return {static_cast<uint8_t>(statistics.size()),
            "combined",
            0,
            static_cast<uint16_t>(
                std::accumulate(statistics.begin(),
                                statistics.end(),
                                0,
                                [](int result, const EncoderStatistics& stat) { return result + stat.getWidth(); })),
            std::max_element(statistics.begin(),
                             statistics.end(),
                             [](const EncoderStatistics& lhs, const EncoderStatistics& rhs) {
                                 return lhs.getHeight() < rhs.getHeight();
                             })
                ->getHeight(),
            std::accumulate(statistics.begin(),
                            statistics.end(),
                            0,
                            [](int result, const EncoderStatistics& stat) { return result + stat.getBitrate(); }),
            false,
            {}};
}

int EncoderStatistics::getBitrate() const noexcept
{
    return bitrate_;
}

const std::vector<std::vector<double>>& EncoderStatistics::getMdvqmScores() const noexcept
{
    return mdvqmScores_;
}

const std::vector<std::vector<double>>& EncoderStatistics::getStvqmScores() const noexcept
{
    return stvqmScores_;
}

const std::vector<std::vector<double>>& EncoderStatistics::getPsnrScores() const noexcept
{
    return psnrScores_;
}

bool EncoderStatistics::isDropChroma() const noexcept
{
    return dropChroma_;
}

const std::vector<double>& EncoderStatistics::getSpatialActivities() const noexcept
{
    return spatialActivities_;
}

const std::vector<double>& EncoderStatistics::getTemporalActivities() const noexcept
{
    return temporalActivities_;
}
