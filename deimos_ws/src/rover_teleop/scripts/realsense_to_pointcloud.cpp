#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/Point32.h>

class PointCloudConverter {
public:
    PointCloudConverter() {
        ros::NodeHandle nh;
        cloud_pub_ = nh.advertise<sensor_msgs::PointCloud>("cloud", 1);
        cloud_sub_ = nh.subscribe("/camera/depth/color/points", 1, &PointCloudConverter::cloudCallback, this);
    }

private:
    ros::Publisher cloud_pub_;
    ros::Subscriber cloud_sub_;

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg2) {
        static bool printed = false;
        if (!printed) {
            ROS_INFO("Received PointCloud2 message");
            printed = true;
        }

        sensor_msgs::PointCloud cloud_msg;
        cloud_msg.header = cloud_msg2->header;

        sensor_msgs::ChannelFloat32 intensity_channel;
        intensity_channel.name = "intensities";

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg2, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg2, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg2, "z");
        sensor_msgs::PointCloud2ConstIterator<float> iter_rgb(*cloud_msg2, "rgb");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_rgb) {
            geometry_msgs::Point32 p;
            p.x = *iter_x;
            p.y = *iter_y;
            p.z = *iter_z;
            cloud_msg.points.push_back(p);

            // Extract RGB from packed float
            union {
                float rgb_float;
                uint32_t rgb_uint;
            } rgb_union;
            rgb_union.rgb_float = *iter_rgb;

            uint8_t r = (rgb_union.rgb_uint >> 16) & 0xFF;
            uint8_t g = (rgb_union.rgb_uint >> 8) & 0xFF;
            uint8_t b = rgb_union.rgb_uint & 0xFF;

            float intensity = (r + g + b) / 3.0f;
            intensity_channel.values.push_back(intensity);
        }

        if (!cloud_msg.points.empty() && !intensity_channel.values.empty()) {
            cloud_msg.channels.push_back(intensity_channel);
        }

        cloud_pub_.publish(cloud_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "realsense_to_pointcloud_cpp");
    PointCloudConverter converter;
    ros::spin();
    return 0;
}
