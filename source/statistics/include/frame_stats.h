#ifndef SINGLE_ENCODER_FRAMESTATS_H
#define SINGLE_ENCODER_FRAMESTATS_H

#include <cstdint>
#include <vector>

namespace lmt
{
namespace statistics
{
struct FrameStats
{
    uint64_t timestamp;
    uint16_t frameCount;
    uint32_t encodedFrameSize;
    int64_t encodingTime;
    double spatialActivity;
    double temporalActivity;
    std::vector<double> mdvqmScores;
    std::vector<double> stvqmScores;
    std::vector<double> psnrScores;
};
}  // namespace statistics
}  // namespace lmt

#endif  // SINGLE_ENCODER_FRAMESTATS_H
