#include "statistics_collection.h"

#include <fstream>

#include <nlohmann/json.hpp>

#include "frame_stats.h"

using namespace lmt::statistics;

void StatisticsCollection::addFrameStatsById(int encoderId, const FrameStats& stats)
{
    stats_[encoderId].addFrameStats(stats);
}

void StatisticsCollection::writeStatsFile(const std::string& fileName)
{
    std::ofstream outputFile(fileName);
    nlohmann::json output;

    for (const auto& stat : stats_)
    {
        nlohmann::json obj;
        obj["encoderId"] = stat.getEncoderId();
        obj["topic"] = stat.getTopicName();
        obj["skip_frames"] = stat.getSkipFrames();
        obj["width"] = stat.getWidth();
        obj["height"] = stat.getHeight();
        obj["bitrate"] = stat.getBitrate();
        obj["chroma_drop"] = stat.isDropChroma();
        obj["filter"] = stat.getFilterParameter().serialize();
        obj["timestamp"] = stat.getTimestamps();
        obj["frame_count"] = stat.getFrameCounts();
        obj["frame_size"] = stat.getEncodedFrameSizes();
        obj["encoding_time"] = stat.getEncodingTimes();
        obj["spatial_activities"] = stat.getSpatialActivities();
        obj["temporal_activities"] = stat.getTemporalActivities();
        obj["mdvqm_scores"] = stat.getMdvqmScores();
        obj["stvqm_scores"] = stat.getStvqmScores();
        obj["psnr_scores"] = stat.getPsnrScores();
        output.push_back(obj);
    }

    outputFile << output << std::endl;
}

StatisticsCollection::StatisticsCollection(std::vector<EncoderStatistics> statistics) noexcept
    : stats_(std::move(statistics))
{
}

void StatisticsCollection::addEncoderStatistic(EncoderStatistics encoderStatistic) noexcept
{
    stats_.push_back(std::move(encoderStatistic));
}

const std::vector<EncoderStatistics>& StatisticsCollection::getStats() const noexcept
{
    return stats_;
}

bool StatisticsCollection::hasData() const noexcept
{
    return std::none_of(stats_.begin(), stats_.end(), [](const EncoderStatistics& encoderStatistics) {
        return encoderStatistics.getTimestamps().empty();
    });
}
