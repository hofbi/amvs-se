#include "av_decoder.h"

#include <ros/console.h>
extern "C" {
#include <libavutil/imgutils.h>
}

using namespace lmt::decoder;

AvDecoder::~AvDecoder() = default;

// NOLINTNEXTLINE(modernize-avoid-c-arrays)
cv::Mat AvDecoder::decodeImpl(const std::unique_ptr<uint8_t[]>& encodedData, uint32_t frameSize)
{
    AVPacket packet;
    av_init_packet(&packet);

    packet.data = encodedData.get();
    packet.size = frameSize;

    std::unique_ptr<AVFrame, std::function<void(AVFrame*)>> picture{av_frame_alloc(),
                                                                    [](AVFrame* frame) { av_frame_free(&frame); }};

    if (!picture)
    {
        ROS_WARN("Could not allocate target picture.");
        return {};
    }

    int res = avcodec_send_packet(codecContext_.get(), &packet);
    if (res < 0)
    {
        ROS_WARN("Could not decode frame data!");
        return {};
    }
    res = avcodec_receive_frame(codecContext_.get(), picture.get());
    if (res < 0 && res != AVERROR(EAGAIN) && res != AVERROR_EOF)
    {
        return {};
    }

    cv::Mat result;
    if (res >= 0)
    {
        const auto width = codecContext_->coded_width;
        const auto height = codecContext_->coded_height;

        swsContext_.reset(sws_getContext(width,
                                         height,
                                         AV_PIX_FMT_YUV420P,
                                         width,
                                         height,
                                         AV_PIX_FMT_BGR24,
                                         SWS_FAST_BILINEAR,
                                         nullptr,
                                         nullptr,
                                         nullptr));

        result = cv::Mat(height, width, CV_8UC3);
        AVFrame dst;
        dst.data[0] = result.data;
        av_image_fill_arrays(dst.data, dst.linesize, result.data, AV_PIX_FMT_BGR24, width, height, 1);
        if (sws_scale(swsContext_.get(), picture->data, picture->linesize, 0, height, dst.data, dst.linesize) != height)
        {
            ROS_WARN("scale failed");
            return {};
        }
    }

    return result;
}

AvDecoder::AvDecoder(AVCodecID codecId)
{
    avcodec_register_all();

    AVCodec* codec = avcodec_find_decoder(codecId);
    codecContext_.reset(avcodec_alloc_context3(codec));

    if (!codec)
    {
        ROS_ERROR("Could not find codec.");
        return;
    }

    if (avcodec_open2(codecContext_.get(), codec, nullptr) < 0)
    {
        ROS_ERROR("Could not open codec.");
        return;
    }
}
