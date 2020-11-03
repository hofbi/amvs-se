#include "super_frame_demux.h"

#include "encoder_statistics.h"
#include "super_frame_mux.h"
#include "test_utils.h"

using namespace lmt::mux;
using namespace lmt::quality;
using namespace lmt::statistics;
using namespace lmt::util;

void testFrameStatsElementSize(const FrameStats& frameStats, size_t expectedSize)
{
    EXPECT_EQ(expectedSize, frameStats.mdvqmScores.size());
    EXPECT_EQ(expectedSize, frameStats.stvqmScores.size());
    EXPECT_EQ(expectedSize, frameStats.psnrScores.size());
}

class SuperFrameDemuxTest : public ::testing::Test
{
  protected:
    SuperFrameDemuxTest()
        : superFrameDemuxUnit_(createSuperFrameLayoutFromStatistics(encoderStatisticsList_),
                               createVQMListFromStatistics<MDVQM>(encoderStatisticsList_),
                               createVQMListFromStatistics<STVQM>(encoderStatisticsList_))
    {
    }

    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    std::vector<EncoderStatistics> encoderStatisticsList_{{0, "", 0, 20, 10, 0, false, {}},
                                                          {1, "", 0, 10, 5, 0, false, {}}};
    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    SuperFrameDemux superFrameDemuxUnit_;
};

TEST_F(SuperFrameDemuxTest, createSuperFrameLayoutFromStatistics_twoElementsInList_correct)
{
    const auto unit = createSuperFrameLayoutFromStatistics(encoderStatisticsList_);

    EXPECT_EQ(unit.size(), encoderStatisticsList_.size());

    EXPECT_EQ(unit.front().width, encoderStatisticsList_.front().getWidth());
    EXPECT_EQ(unit.front().height, encoderStatisticsList_.front().getHeight());

    EXPECT_EQ(unit.back().width, encoderStatisticsList_.back().getWidth());
    EXPECT_EQ(unit.back().height, encoderStatisticsList_.back().getHeight());
}

TEST_F(SuperFrameDemuxTest, createSuperFrameLayoutFromStatistics_noElementsInList_empty)
{
    const auto unit = createSuperFrameLayoutFromStatistics({});

    EXPECT_TRUE(unit.empty());
}

TEST_F(SuperFrameDemuxTest, createVQMListFromStatistics_twoElementsInList_correct)
{
    const auto unit = createVQMListFromStatistics<MDVQM>(encoderStatisticsList_);

    EXPECT_EQ(unit.size(), encoderStatisticsList_.size());
}

TEST_F(SuperFrameDemuxTest, createVQMListFromStatistics_noElementsInList_empty)
{
    const auto unit = createVQMListFromStatistics<MDVQM>({});

    EXPECT_TRUE(unit.empty());
}

TEST_F(SuperFrameDemuxTest, demuxSuperFrame_validSuperFrame_gotIndividualFrames)
{
    const auto individualFrames = superFrameDemuxUnit_.demuxSuperFrame(cv::Mat(10, 30, CV_8UC3));

    EXPECT_EQ(encoderStatisticsList_.size(), individualFrames.size());

    EXPECT_EQ(individualFrames.front().cols, encoderStatisticsList_.front().getWidth());
    EXPECT_EQ(individualFrames.front().rows, encoderStatisticsList_.front().getHeight());

    EXPECT_EQ(individualFrames.back().cols, encoderStatisticsList_.back().getWidth());
    EXPECT_EQ(individualFrames.back().rows, encoderStatisticsList_.back().getHeight());
}

TEST_F(SuperFrameDemuxTest, demuxSuperFrame_validSuperFrame_indivialFramesPixelSizeIdentical)
{
    cv::Mat first{10, 20, CV_8UC3, cv::Scalar::all(200)};
    cv::Mat second{5, 10, CV_8UC3, cv::Scalar::all(100)};
    cv::Mat superFrame = first;
    addSegmentToSuperFrame(superFrame, second);

    const auto individualFrames = superFrameDemuxUnit_.demuxSuperFrame(superFrame);

    compareCvMatPixelSize(first, individualFrames.front());
    compareCvMatPixelSize(second, individualFrames.back());
}

TEST_F(SuperFrameDemuxTest, demuxSuperFrame_validSuperFrame_indivialFramesPixelWiseIdentical)
{
    cv::Mat first{10, 20, CV_8UC3, cv::Scalar::all(200)};
    cv::Mat second{5, 10, CV_8UC3, cv::Scalar::all(100)};
    cv::Mat superFrame = first;
    addSegmentToSuperFrame(superFrame, second);

    const auto individualFrames = superFrameDemuxUnit_.demuxSuperFrame(superFrame);

    compareCvMatPixelWise(first, individualFrames.front());
    compareCvMatPixelWise(second, individualFrames.back());
}

TEST_F(SuperFrameDemuxTest, demuxSuperFrame_validSuperFrameAndLayoutLargerThanSuperFrame_gotIndividualFrames)
{
    EXPECT_THROW({ superFrameDemuxUnit_.demuxSuperFrame(cv::Mat(10, 20, CV_8UC3)); }, std::exception);
}

TEST_F(SuperFrameDemuxTest, applyVQMForIndividualFrame_defaultInputStats_statsCopied)
{
    cv::Mat frame{10, 20, CV_8UC3, cv::Scalar::all(200)};
    cv::Mat rawFrame{10, 20, CV_8UC3, cv::Scalar::all(100)};
    FrameStats inputStats;

    EXPECT_TRUE(inputStats.psnrScores.empty());
    const auto result = superFrameDemuxUnit_.applyVQMForIndividualFrame(0, frame, rawFrame, inputStats);
    EXPECT_FALSE(result.psnrScores.empty());
}

TEST_F(SuperFrameDemuxTest, applyVQMForIndividualFrame_inputStatsWithOneVQMValue_vqmElementsSizeIs2)
{
    cv::Mat frame{10, 20, CV_8UC3, cv::Scalar::all(200)};
    cv::Mat rawFrame{10, 20, CV_8UC3, cv::Scalar::all(100)};
    FrameStats inputStats{0, 1, 2, 3, 0, 1, {2}, {3}, {4}};

    testFrameStatsElementSize(inputStats, 1);
    const auto result = superFrameDemuxUnit_.applyVQMForIndividualFrame(0, frame, rawFrame, inputStats);
    testFrameStatsElementSize(result, 2);
}
