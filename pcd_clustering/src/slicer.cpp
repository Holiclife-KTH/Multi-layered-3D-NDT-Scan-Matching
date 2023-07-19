// ROS core
#include <ros/ros.h>

//library include
#include <iostream>
#include <cmath>
#include <vector>

//ros library
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Float32.h>

//pcl library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <set>
#include <boost/format.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>

//Define constant for PI
#define PI 3.14159265359
using namespace std;

// Clustering class definition
class Clustering{
    public:
        //Constructor and destructor
        Clustering() {}
        ~Clustering() {}

        // Initialization function
        void initClustering(ros::NodeHandle& nh);

        // Point cloud type definition
        typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
        typedef std::shared_ptr<Clustering> Ptr;

        // Filtered point cloud member
        pcl::PCLPointCloud2 cloud_filtered;
        pcl::PCLPointCloud2 top_cloud;
        pcl::PCLPointCloud2 bottom_cloud;
         pcl::PCLPointCloud2 middle_cloud;

        
    private:
        // Callback function for processing incoming point cloud data
        void Pointcallback(const sensor_msgs::PointCloud2& point);
        void rawPointcallback(const sensor_msgs::PointCloud2& point2);

        // ROS node, subscriber, and publisher members
        ros::NodeHandle node;
        ros::Subscriber sub;
        ros::Publisher pub;
        ros::Publisher pub2;
        ros::Publisher pub3;
};


// Implementation of initClustering method
void Clustering::initClustering(ros::NodeHandle& nh){
    node = nh;
    // Subscribe to "/ROI_points" topic
    sub = node.subscribe("/VLP16_pointcloud", 10, &Clustering::rawPointcallback, this);
    // Advertise "/RANSAC_points" topic
    pub = node.advertise<sensor_msgs::PointCloud2> ("/middle_layer_points", 100);
    pub2 = node.advertise<sensor_msgs::PointCloud2> ("/top_layer_points", 100);
    pub3 = node.advertise<sensor_msgs::PointCloud2> ("/bottom_layer_points", 100);

}

void Clustering::rawPointcallback(const sensor_msgs::PointCloud2& point2){

    pcl::PointCloud <pcl::PointXYZI>::Ptr raw_pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud <pcl::PointXYZI>::Ptr top_Points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud <pcl::PointXYZI>::Ptr bottom_Points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud <pcl::PointXYZI>::Ptr middle_Points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(point2, *raw_pcl_cloud);

    pcl::PassThrough<pcl::PointXYZI> pass;

   
    pass.setInputCloud(raw_pcl_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(1.0, 2.0);
    pass.filter(*top_Points);

    pass.setFilterLimits(-0.9867, 0.0);
    pass.filter(*bottom_Points);

    pass.setFilterLimits(0.0, 1.0);
    pass.filter(*middle_Points);  
    

    pcl::toPCLPointCloud2(*top_Points, top_cloud);  
    pcl::toPCLPointCloud2(*bottom_Points, bottom_cloud);
    pcl::toPCLPointCloud2(*middle_Points, middle_cloud);
    sensor_msgs::PointCloud2 top_pcd_msg;
    sensor_msgs::PointCloud2 bottom_pcd_msg;
    sensor_msgs::PointCloud2 middle_pcd_msg;
    pcl_conversions::fromPCL(top_cloud, top_pcd_msg);
    pcl_conversions::fromPCL(bottom_cloud, bottom_pcd_msg);
    pcl_conversions::fromPCL(middle_cloud, middle_pcd_msg);

    top_pcd_msg.header.frame_id = point2.header.frame_id;
    bottom_pcd_msg.header.frame_id = point2.header.frame_id;
    middle_pcd_msg.header.frame_id = point2.header.frame_id;
    top_pcd_msg.header.stamp = ros::Time::now();
    bottom_pcd_msg.header.stamp = ros::Time::now();
    bottom_pcd_msg.header.stamp = ros::Time::now();
    pub2.publish(top_pcd_msg);
    pub3.publish(bottom_pcd_msg);
    pub.publish(middle_pcd_msg);


}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "Slicer");
    ros::NodeHandle node("~");
    std::cout<< "Start node for clustering......" << std::endl;
    Clustering::Ptr clusteringPtr;
    clusteringPtr.reset(new Clustering);
    clusteringPtr->initClustering(node);
    ros::Duration(1.0).sleep();
    ros::spin();

}