#include "channel_filter.h"

#include <ros/console.h>

namespace lmt
{
namespace filter
{
ChannelFilter::ChannelFilter(bool dropChroma) noexcept : dropChroma_(dropChroma) {}

cv_bridge::CvImagePtr ChannelFilter::filterImpl(const cv_bridge::CvImagePtr& inputImg)
{
    cv_bridge::CvImagePtr image = inputImg;
    image->encoding = "bgr8";

    cv::cvtColor(inputImg->image, image->image, getCvColorConversionCodeFromSensorMsg(inputImg));

    if (dropChroma_)
    {
        cv::cvtColor(image->image, image->image, cv::COLOR_BGR2YUV_I420);
        dropChroma(image->image);
        cv::cvtColor(image->image, image->image, cv::COLOR_YUV2BGR_I420);
    }

    return image;
}

cv::ColorConversionCodes getCvColorConversionCodeFromSensorMsg(const cv_bridge::CvImageConstPtr& img) noexcept
{
    static const std::map<std::string, cv::ColorConversionCodes> knownFormats = {{
        {sensor_msgs::image_encodings::RGB8, cv::COLOR_RGB2BGR},
        {sensor_msgs::image_encodings::RGB16, cv::COLOR_RGB2BGR},
        {sensor_msgs::image_encodings::RGBA8, cv::COLOR_RGBA2BGR},
        {sensor_msgs::image_encodings::RGBA16, cv::COLOR_RGBA2BGR},
        {sensor_msgs::image_encodings::BGR8, cv::COLOR_BGRA2BGR},
        {sensor_msgs::image_encodings::BGR16, cv::COLOR_BGRA2BGR},
        {sensor_msgs::image_encodings::BGRA8, cv::COLOR_BGRA2BGR},
        {sensor_msgs::image_encodings::BGRA16, cv::COLOR_BGRA2BGR},
    }};

    auto format = knownFormats.find(img->encoding);
    if (format == knownFormats.end())
    {
        ROS_ERROR("Image format '%s' unknown", img->encoding.c_str());
        return cv::COLOR_BGRA2BGR;
    }

    return format->second;
}

void dropChroma(cv::Mat& yuvImage)
{
    const auto chroma = yuvImage(cv::Rect(0, yuvImage.rows * 2 / 3, yuvImage.cols, yuvImage.rows / 3));
    cv::multiply(chroma, 0, chroma);
}

}  // namespace filter
}  // namespace lmt
