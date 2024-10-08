#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_pointcloud_publisher");
    ros::NodeHandle nh;

    // Publisher for the point cloud
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("random_points", 1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = 1000;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (auto& point : *cloud) {
        point.x = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        point.y = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        point.z = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    }

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = "base_link"; // Change this frame ID as needed

    ros::Rate loop_rate(1); // Publishing rate (1 Hz)
    while (ros::ok()) {
        cloud_msg.header.stamp = ros::Time::now();
        pub.publish(cloud_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
