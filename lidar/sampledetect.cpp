#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    float min_distance = std::numeric_limits<float>::max();

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        float range = scan->ranges[i];

        // Check for invalid ranges and minimum valid distance
        if (!std::isnan(range) && range > scan->range_min && range < scan->range_max) {
            min_distance = std::min(min_distance, range);
        }
    }

    if (min_distance < 0.5) { // Adjust the threshold as needed
        ROS_WARN("Object detected too close! Distance: %fm", min_distance);
        ros::NodeHandle nh;
        ros::Publisher danger_pub = nh.advertise<std_msgs::String>("danger_detected", 1);
        std_msgs::String msg;
        msg.data = "Object detected too close!";
        danger_pub.publish(msg);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_object_detection");
    ros::NodeHandle nh;

    ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 10, scanCallback);

    ros::spin();

    return 0;
}
