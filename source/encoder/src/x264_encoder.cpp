#include "x264_encoder.h"

extern "C" {
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
}
#include <ros/console.h>

using namespace lmt::encoder;
using namespace lmt::statistics;

X264Encoder::~X264Encoder()
{
    if (encoder_)
    {
        x264_picture_clean(&picIn_);
    }
}

X264Encoder::X264Encoder(uint16_t outWidth, uint16_t outHeight, uint8_t qp) : VideoEncoder(outWidth, outHeight)
{
    auto param = getPresetParam(outWidth, outHeight);

    param.rc.i_rc_method = X264_RC_CQP;
    param.rc.i_qp_constant = qp;
    param.rc.f_rf_constant = qp;
    param.rc.f_rf_constant_max = qp;

    encoder_.reset(x264_encoder_open(&param));
    x264_encoder_headers(encoder_.get(), &nals_, &nheader_);

    // Initialize X264 Picture
    x264_picture_alloc(&picIn_, param.i_csp, outWidth_, outHeight_);
}

FrameStats X264Encoder::encodeImpl(const cv_bridge::CvImageConstPtr& inputImg,
                                   std::unique_ptr<uint8_t[]>& encodedData)  // NOLINT(modernize-avoid-c-arrays)
{
    AVPixelFormat inPixelFormat = AV_PIX_FMT_BGR24;
    swsContext_.reset(sws_getContext(inputImg->image.cols,
                                     inputImg->image.rows,
                                     inPixelFormat,
                                     outWidth_,
                                     outHeight_,
                                     AV_PIX_FMT_YUV420P,
                                     SWS_FAST_BILINEAR,
                                     nullptr,
                                     nullptr,
                                     nullptr));

    // Put raw image data to AV picture
    AVFrame picRaw;
    if (av_image_fill_arrays(picRaw.data,
                             picRaw.linesize,
                             inputImg->image.data,
                             inPixelFormat,
                             inputImg->image.cols,
                             inputImg->image.rows,
                             1) < 0)
    {
        ROS_WARN("Cannot fill the raw input buffer");
        return {};
    }

    if (sws_scale(swsContext_.get(),
                  picRaw.data,
                  picRaw.linesize,
                  0,
                  inputImg->image.rows,
                  picIn_.img.plane,
                  picIn_.img.i_stride) != outHeight_)
    {
        ROS_WARN("scale failed");
        return {};
    }

    // Encode
    picIn_.i_pts = frameCounter_;
    ++frameCounter_;

    const auto start = ros::Time::now();
    uint32_t encodedFrameSize =
        x264_encoder_encode(encoder_.get(), &nals_, &numNals_, &picIn_, &picOut_);  // encodedFrameSize [byte]
    const auto end = ros::Time::now();
    const auto duration = end - start;

    std::copy(nals_[0].p_payload, nals_[0].p_payload + encodedFrameSize, encodedData.get());

    return {start.toNSec(), frameCounter_, encodedFrameSize, duration.toNSec()};
}

X264Encoder::X264Encoder(uint16_t outWidth, uint16_t outHeight, int bitrate) : VideoEncoder(outWidth, outHeight)
{
    auto param = getPresetParam(outWidth, outHeight);

    param.rc.i_rc_method = X264_RC_CRF;
    param.rc.i_vbv_max_bitrate = bitrate;
    const auto bufferSizeRatioOfMaxBitrate = 0.1;
    param.rc.i_vbv_buffer_size = static_cast<int>(bufferSizeRatioOfMaxBitrate * param.rc.i_vbv_max_bitrate);

    encoder_.reset(x264_encoder_open(&param));
    x264_encoder_headers(encoder_.get(), &nals_, &nheader_);

    // Initialize X264 Picture
    x264_picture_alloc(&picIn_, param.i_csp, outWidth_, outHeight_);
}

x264_param_t X264Encoder::getPresetParam(uint16_t outWidth, uint16_t outHeight)
{
    x264_param_t param;
    x264_param_default_preset(&param, "ultrafast", "zerolatency,fastdecode");
    x264_param_apply_profile(&param, "baseline");
    param.i_width = outWidth;
    param.i_height = outHeight;
    const auto frameRate = 20;
    param.i_fps_num = frameRate;
    param.i_fps_den = 1;
    param.i_csp = X264_CSP_I420;

    return param;
}
