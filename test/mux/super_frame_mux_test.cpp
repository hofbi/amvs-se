#include "super_frame_mux.h"

#include <utility>

#include <boost/make_shared.hpp>
#include <gmock/gmock.h>

#include "test_utils.h"
#include "video_decoder.h"
#include "video_encoder.h"
#include "vqm_state.h"

using namespace lmt::mux;
using namespace lmt::encoder;
using namespace lmt::decoder;
using namespace lmt::statistics;
using namespace lmt::util;
using namespace lmt::quality;

class VideoEncoderFake : public VideoEncoder
{
  public:
    VideoEncoderFake() : VideoEncoder(10, 10) {}

    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes, modernize-avoid-c-arrays)
    MOCK_METHOD2(encodeImpl,
                 FrameStats(const cv_bridge::CvImageConstPtr& inputImg, std::unique_ptr<uint8_t[]>& encodedData));
};

class VideoDecoderFake : public VideoDecoder
{
  public:
    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes, modernize-avoid-c-arrays)
    MOCK_METHOD2(decodeImpl, cv::Mat(const std::unique_ptr<uint8_t[]>& encodedData, uint32_t frameSize));
};

class SuperFrameMuxTest : public ::testing::Test
{
  protected:
    SuperFrameMuxTest()
        : unit_([this](auto&&... args) { superFrameCallback(std::forward<decltype(args)>(args)...); },
                std::make_unique<VQMState>(0)),
          videoEncoder_(std::make_unique<VideoEncoderFake>()),
          videoDecoder_(std::make_unique<VideoDecoderFake>()),
          image_(boost::make_shared<cv_bridge::CvImage>(std_msgs::Header(),
                                                        "bgr8",
                                                        cv::Mat(25, 25, CV_8UC3, cv::Scalar::all(100)))),
          smallImage_(boost::make_shared<cv_bridge::CvImage>(std_msgs::Header(),
                                                             "bgr8",
                                                             cv::Mat(15, 15, CV_8UC3, cv::Scalar::all(200))))
    {
    }

    void superFrameCallback(const std::vector<FrameStats>& stats,
                            const cv::Mat& /*superFrame*/,
                            const std::vector<cv::Mat>& /*individualRawImages*/) const
    {
        EXPECT_EQ(expectedSize_, stats.size());
    }

    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    SuperFrameMux unit_;

    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    std::unique_ptr<VideoEncoderFake> videoEncoder_{nullptr};
    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    std::unique_ptr<VideoDecoderFake> videoDecoder_{nullptr};
    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    cv_bridge::CvImagePtr image_{nullptr};
    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    cv_bridge::CvImagePtr smallImage_{nullptr};
    // NOLINTNEXTLINE(misc-non-private-member-variables-in-classes)
    int expectedSize_{0};
};

TEST_F(SuperFrameMuxTest, superFrameCallback_initOnly_neverCalled)
{
    unit_.init(1, std::move(videoEncoder_), std::move(videoDecoder_));

    EXPECT_CALL(VideoEncoderFake(), encodeImpl(testing::_, testing::_)).Times(0);
}

TEST_F(SuperFrameMuxTest, superFrameCallback_initWithOneIndividualFrame_calledOnce)
{
    EXPECT_CALL(*videoEncoder_, encodeImpl(testing::_, testing::_)).Times(1);

    unit_.init(1, std::move(videoEncoder_), std::move(videoDecoder_));

    expectedSize_ = 2;
    unit_.addImageSegment(0, image_, {});
}

TEST_F(SuperFrameMuxTest, superFrameCallback_initWithTwoIndividualFramesAndAddEachSegmentOnce_calledOnce)
{
    EXPECT_CALL(*videoEncoder_, encodeImpl(testing::_, testing::_)).Times(1);

    unit_.init(2, std::move(videoEncoder_), std::move(videoDecoder_));

    expectedSize_ = 3;
    unit_.addImageSegment(0, image_, {});
    unit_.addImageSegment(1, image_, {});
}

TEST_F(SuperFrameMuxTest, superFrameCallback_initWithTwoIndividualFramesAndAddJustOneSegment_neverCalled)
{
    EXPECT_CALL(*videoEncoder_, encodeImpl(testing::_, testing::_)).Times(0);

    unit_.init(2, std::move(videoEncoder_), std::move(videoDecoder_));

    expectedSize_ = 3;
    unit_.addImageSegment(0, image_, {});
}

TEST_F(SuperFrameMuxTest, superFrameCallback_initWithTwoIndividualFramesAndAddOneSegmentThreeTimes_neverCalled)
{
    EXPECT_CALL(*videoEncoder_, encodeImpl(testing::_, testing::_)).Times(0);

    unit_.init(2, std::move(videoEncoder_), std::move(videoDecoder_));

    expectedSize_ = 3;
    unit_.addImageSegment(0, image_, {});
    unit_.addImageSegment(0, image_, {});
    unit_.addImageSegment(0, image_, {});
}

TEST_F(SuperFrameMuxTest,
       superFrameCallback_initWithTwoIndividualFramesAndAddOneSegmentThreeTimesAndTheOtherTwoTimes_calledTwice)
{
    EXPECT_CALL(*videoEncoder_, encodeImpl(testing::_, testing::_)).Times(2);

    unit_.init(2, std::move(videoEncoder_), std::move(videoDecoder_));

    expectedSize_ = 3;
    unit_.addImageSegment(0, image_, {});
    unit_.addImageSegment(1, image_, {});
    unit_.addImageSegment(0, image_, {});
    unit_.addImageSegment(0, image_, {});
    unit_.addImageSegment(1, image_, {});
}

TEST_F(SuperFrameMuxTest, superFrameCallback_initWithTwoIndividualFramesOfDifferentSize_calledOnce)
{
    EXPECT_CALL(*videoEncoder_, encodeImpl(testing::_, testing::_)).Times(1);

    unit_.init(2, std::move(videoEncoder_), std::move(videoDecoder_));

    expectedSize_ = 3;
    unit_.addImageSegment(0, image_, {});
    unit_.addImageSegment(1, smallImage_, {});
}

TEST_F(SuperFrameMuxTest, moveFramesIntoSuperFrame_oneFrame_equalInputAndOutputSize)
{
    std::vector<cv::Mat> frames{image_->image};

    auto muxedImage = moveFramesIntoSuperFrame(frames);

    compareCvMatSize(image_->image, muxedImage);
}

TEST_F(SuperFrameMuxTest, moveFramesIntoSuperFrame_twoFrames_combinedSuperFrameDimensionsCorrect)
{
    std::vector<cv::Mat> frames{image_->image, image_->image};

    auto muxedImage = moveFramesIntoSuperFrame(frames);

    EXPECT_EQ(2 * image_->image.cols, muxedImage.cols);
    EXPECT_EQ(image_->image.rows, muxedImage.rows);
}

TEST_F(SuperFrameMuxTest, moveFramesIntoSuperFrame_twoFramesWithDifferentHeight_combinedSuperFrameDimensionsCorrect)
{
    std::vector<cv::Mat> frames{image_->image, smallImage_->image};

    auto muxedImage = moveFramesIntoSuperFrame(frames);

    EXPECT_EQ(image_->image.cols + smallImage_->image.cols, muxedImage.cols);
    EXPECT_EQ(image_->image.rows, muxedImage.rows);
}

TEST_F(SuperFrameMuxTest, addSegmentToSuperFrame_sameHeight_combinedSuperFrameDimensionsCorrect)
{
    cv::Mat superFrame = image_->image;

    addSegmentToSuperFrame(superFrame, image_->image);

    EXPECT_EQ(2 * image_->image.cols, superFrame.cols);
    EXPECT_EQ(image_->image.rows, superFrame.rows);
}

TEST_F(SuperFrameMuxTest, addSegmentToSuperFrame_smallerHeightThanSuperFrame_combinedSuperFrameDimensionsCorrect)
{
    cv::Mat superFrame = image_->image;

    addSegmentToSuperFrame(superFrame, smallImage_->image);

    EXPECT_EQ(image_->image.cols + smallImage_->image.cols, superFrame.cols);
    EXPECT_EQ(image_->image.rows, superFrame.rows);
}

TEST_F(SuperFrameMuxTest, addSegmentToSuperFrame_sameHeight_superFramePixelWiseCorrect)
{
    cv::Mat superFrame = image_->image;

    addSegmentToSuperFrame(superFrame, image_->image);

    const cv::Rect firstSegment{0, 0, image_->image.cols, image_->image.rows};
    const cv::Rect secondSegment{image_->image.cols, 0, image_->image.cols, image_->image.rows};

    compareCvMatPixelWise(image_->image, superFrame(firstSegment));
    compareCvMatPixelWise(image_->image, superFrame(secondSegment));
}

TEST_F(SuperFrameMuxTest, addSegmentToSuperFrame_smallerHeightThanSuperFrame_superFramePixelWiseCorrect)
{
    cv::Mat superFrame = image_->image;

    addSegmentToSuperFrame(superFrame, smallImage_->image);

    const cv::Rect firstSegment{0, 0, image_->image.cols, image_->image.rows};
    const cv::Rect secondSegment{image_->image.cols, 0, smallImage_->image.cols, smallImage_->image.rows};
    const cv::Rect remainingSegment{image_->image.cols,
                                    smallImage_->image.rows,
                                    smallImage_->image.cols,
                                    image_->image.rows - smallImage_->image.rows};

    compareCvMatPixelWise(image_->image, superFrame(firstSegment));
    compareCvMatPixelWise(smallImage_->image, superFrame(secondSegment));
    compareCvMatPixelWise(
        {image_->image.rows - smallImage_->image.rows, smallImage_->image.cols, CV_8UC3, cv::Scalar::all(0)},
        superFrame(remainingSegment));
}
