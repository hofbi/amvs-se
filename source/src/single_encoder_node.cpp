#include <ros/ros.h>

#include "filter_parameter.h"
#include "single_encoder.h"

using namespace lmt;
using namespace lmt::statistics;
using namespace lmt::filter;

std::vector<EncoderStatistics> parseEncoderParams(const ros::NodeHandle& pnh)
{
    XmlRpc::XmlRpcValue xmlRpcValue;
    pnh.getParam("image_params", xmlRpcValue);
    ROS_ASSERT(xmlRpcValue.getType() == XmlRpc::XmlRpcValue::TypeArray);

    const auto numberImageTopics = xmlRpcValue.size();
    std::vector<EncoderStatistics> statistics;
    for (size_t i = 0; i < numberImageTopics; ++i)
    {
        auto imageParams = xmlRpcValue[i];
        statistics.emplace_back(i,
                                imageParams["topic"],
                                int(imageParams["skip_frames"]),
                                int(imageParams["width"]),
                                int(imageParams["height"]),
                                int(imageParams["bitrate"]),
                                bool(imageParams["chroma_drop"]),
                                FilterParameter{std::string(imageParams["filter"]["type"]),
                                                int(imageParams["filter"]["ksize"]),
                                                double(imageParams["filter"]["sigma"])});
    }
    return statistics;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "single_encoder");

    ros::NodeHandle pnh("~");

    SingleEncoder singleEncoder(pnh, parseEncoderParams(pnh));

    while (ros::ok())
    {
        ros::spinOnce();
        ROS_INFO_THROTTLE(3, "Running...");
    }

    singleEncoder.writeStatistics();

    return EXIT_SUCCESS;
}
