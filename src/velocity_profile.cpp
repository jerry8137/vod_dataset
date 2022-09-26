#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <iostream>
#include <iomanip>
#include <cstdlib>
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> PointCloud;

struct MyPointType {
    PCL_ADD_POINT4D;
    float RCS;
    float v_r;
    float v_r_compensated;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    MyPointType,
    (float,x,x)
    (float,y,y)
    (float,z,z)
    (float,RCS,RCS)
    (float,v_r,v_r)
    (float,v_r_compensated,v_r_compensated)
)


void callback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    pcl::PointCloud<MyPointType> cloud;
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*msg, pcl_pc);
    for(int i=0; i<pcl_pc.fields.size(); i++) {
        std::cout << pcl_pc.fields[i] << std::endl;
    }
    sensor_msgs::PointCloud out_cloud;
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, out_cloud);
    for(int i=0; i<out_cloud.points.size();i++) {
        std::cout << out_cloud.points[i] << std::endl;
    }
}

int main(int argc, char** argv) {
    ros::init(argc,argv,"radar_velocity_profile");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("radar", 1, callback);

    ros::spin();

    return 0;
}