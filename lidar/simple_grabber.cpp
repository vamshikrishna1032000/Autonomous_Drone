#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

void cloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) {
    pcl::visualization::CloudViewer viewer("Simple Grabber");
    viewer.showCloud(cloud);
}

int main() {
    pcl::io::OpenNI2Grabber grabber;
    boost::function<void(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f = cloudCallback;
    grabber.registerCallback(f);
    grabber.start();
    
    while (true) {}
    
    grabber.stop();
    return 0;
}
