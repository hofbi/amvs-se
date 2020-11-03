#ifndef SINGLE_ENCODER_ENCODER_STATISTICS_H
#define SINGLE_ENCODER_ENCODER_STATISTICS_H

#include <cstdint>
#include <vector>

#include "filter_parameter.h"

namespace lmt
{
namespace statistics
{
struct FrameStats;

class EncoderStatistics
{
  public:
    EncoderStatistics(uint8_t encoderId,
                      std::string topicName,
                      uint8_t skipFrames,
                      uint16_t width,
                      uint16_t height,
                      int bitrate,
                      bool dropChroma,
                      filter::FilterParameter filter) noexcept;

    void addFrameStats(const FrameStats& encodingStats);

    uint8_t getEncoderId() const noexcept;
    const std::string& getTopicName() const noexcept;
    uint8_t getSkipFrames() const noexcept;
    uint16_t getWidth() const noexcept;
    uint16_t getHeight() const noexcept;
    int getBitrate() const noexcept;
    bool isDropChroma() const noexcept;
    const filter::FilterParameter& getFilterParameter() const noexcept;
    const std::vector<uint64_t>& getTimestamps() const noexcept;
    const std::vector<uint16_t>& getFrameCounts() const noexcept;
    const std::vector<uint32_t>& getEncodedFrameSizes() const noexcept;
    const std::vector<int64_t>& getEncodingTimes() const noexcept;
    const std::vector<double>& getSpatialActivities() const noexcept;
    const std::vector<double>& getTemporalActivities() const noexcept;
    const std::vector<std::vector<double>>& getMdvqmScores() const noexcept;
    const std::vector<std::vector<double>>& getStvqmScores() const noexcept;
    const std::vector<std::vector<double>>& getPsnrScores() const noexcept;

  public:
    static EncoderStatistics createSuperFrameStatistics(const std::vector<statistics::EncoderStatistics>& statistics);

  private:
    uint8_t encoderId_{};
    std::string topicName_;
    uint8_t skipFrames_{0};
    uint16_t width_{0};
    uint16_t height_{0};
    int bitrate_{0};
    bool dropChroma_{false};
    filter::FilterParameter filter_;
    std::vector<uint64_t> timestamps_{};
    std::vector<uint16_t> frameCounts_{};
    std::vector<uint32_t> encodedFrameSizes_{};
    std::vector<int64_t> encodingTimes_{};
    std::vector<double> spatialActivities_{};
    std::vector<double> temporalActivities_{};
    std::vector<std::vector<double>> mdvqmScores_{};
    std::vector<std::vector<double>> stvqmScores_{};
    std::vector<std::vector<double>> psnrScores_{};
};
}  // namespace statistics
}  // namespace lmt

#endif  // SINGLE_ENCODER_ENCODER_STATISTICS_H
