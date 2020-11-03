#ifndef SINGLE_ENCODER_STATISTICS_COLLECTION_H
#define SINGLE_ENCODER_STATISTICS_COLLECTION_H

#include <string>

#include "encoder_statistics.h"

namespace lmt
{
namespace statistics
{
struct FrameStats;

class StatisticsCollection
{
  public:
    explicit StatisticsCollection(std::vector<EncoderStatistics> statistics) noexcept;

    bool hasData() const noexcept;

    void addEncoderStatistic(EncoderStatistics encoderStatistic) noexcept;

    void addFrameStatsById(int encoderId, const FrameStats& stats);
    void writeStatsFile(const std::string& fileName);

    const std::vector<EncoderStatistics>& getStats() const noexcept;

  private:
    std::vector<EncoderStatistics> stats_{};
};
}  // namespace statistics
}  // namespace lmt

#endif  // SINGLE_ENCODER_STATISTICS_COLLECTION_H
