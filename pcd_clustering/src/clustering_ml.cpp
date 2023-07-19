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
        pcl::PointXYZI minPt, maxPt;
        std_msgs::Float32 height_msg;
        
    private:
        // Callback function for processing incoming point cloud data
        void Pointcallback(const sensor_msgs::PointCloud2& point);
        void rawPointcallback(const sensor_msgs::PointCloud2& point2);

        // ROS node, subscriber, and publisher members
        ros::NodeHandle node;
        ros::Subscriber sub;
        ros::Subscriber sub2;
        ros::Publisher pub;
        ros::Publisher pub2;
        ros::Publisher pub3;
        ros::Publisher pub4;

        float lowerBoundary = 0.0;
        float upperBoundary = 0.0;
};


// Implementation of initClustering method
void Clustering::initClustering(ros::NodeHandle& nh){
    node = nh;
    // Subscribe to "/ROI_points" topic
    sub = node.subscribe("/ROI_points", 10, &Clustering::Pointcallback, this);
    sub2 = node.subscribe("/VLP16_pointcloud", 10, &Clustering::rawPointcallback, this);
    // Advertise "/RANSAC_points" topic
    pub = node.advertise<sensor_msgs::PointCloud2> ("/clustered_points", 100);
    pub2 = node.advertise<sensor_msgs::PointCloud2> ("/top_layer_points", 100);
    pub3 = node.advertise<sensor_msgs::PointCloud2> ("/bottom_layer_points", 100);
    pub4 = node.advertise<std_msgs::Float32> ("/object_height", 10);

    node.getParam("setSourceLowerBound", lowerBoundary);
    node.getParam("setSourceUpperBound", upperBoundary);

}

void Clustering::rawPointcallback(const sensor_msgs::PointCloud2& point2){

    pcl::PointCloud <pcl::PointXYZI>::Ptr raw_pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud <pcl::PointXYZI>::Ptr top_Points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud <pcl::PointXYZI>::Ptr bottom_Points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(point2, *raw_pcl_cloud);

    pcl::PassThrough<pcl::PointXYZI> pass2;
    pcl::PassThrough<pcl::PointXYZI> pass3;

    if (maxPt.z < -100){
        pass2.setInputCloud(raw_pcl_cloud);
        pass2.setFilterFieldName("z");
        pass2.setFilterLimits(upperBoundary, 2.0);
        pass2.filter(*top_Points);

        pass3.setInputCloud(raw_pcl_cloud);
        pass3.setFilterFieldName("z");
        pass3.setFilterLimits(lowerBoundary, upperBoundary);
        pass3.filter(*bottom_Points);
    }

    else
    {
        pass2.setInputCloud(raw_pcl_cloud);
        pass2.setFilterFieldName("z");
        pass2.setFilterLimits(maxPt.z, 2.0);
        pass2.filter(*top_Points);

        pass3.setInputCloud(raw_pcl_cloud);
        pass3.setFilterFieldName("z");
        pass3.setFilterLimits(-0.9867, maxPt.z);
        pass3.filter(*bottom_Points);
    
    }
    

    pcl::toPCLPointCloud2(*top_Points, top_cloud);  
    pcl::toPCLPointCloud2(*bottom_Points, bottom_cloud);
    sensor_msgs::PointCloud2 top_pcd_msg;
    sensor_msgs::PointCloud2 bottom_pcd_msg;
    pcl_conversions::fromPCL(top_cloud, top_pcd_msg);
    pcl_conversions::fromPCL(bottom_cloud, bottom_pcd_msg);

    top_pcd_msg.header.frame_id = point2.header.frame_id;
    bottom_pcd_msg.header.frame_id = point2.header.frame_id;
    top_pcd_msg.header.stamp = ros::Time::now();
    bottom_pcd_msg.header.stamp = ros::Time::now();
    pub2.publish(top_pcd_msg);
    pub3.publish(bottom_pcd_msg);

    height_msg.data = maxPt.z;

    
    pub4.publish(height_msg);

}

// Implementation of Pointcallback method
void Clustering::Pointcallback(const sensor_msgs::PointCloud2& point)
{
    // Convert incomping point cloud data from ROS message format to PCL point cloud
    pcl::PointCloud <pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud <pcl::PointXYZ>::Ptr filtered_Points(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_v2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(point, *cloud);

    // Apply a passthrough filter based on height(z-axis)
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.9867, 5.2);
    pass.filter (*filtered_Points);

    //Voxelization
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(filtered_Points);
    vg.setLeafSize(0.4f, 0.4f, 0.4f);
    vg.filter(*cloud_filtered_v2);

    //Creating the KdTree objectr for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered_v2);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (1); // 포인트와 포인트 간의 간격 -> 1m
    ec.setMinClusterSize (4);   // 한 군집의 최소 포인트 개수
    ec.setMaxClusterSize (20);  // 한 군집의 최대 포인트 개수
    ec.setSearchMethod (tree);  // 검색 방법: tree
    ec.setInputCloud (cloud_filtered_v2); // cloud_filtered_v2에 클러스터링 결과를 입력
    ec.extract (cluster_indices);

    pcl::PointCloud<pcl::PointXYZI> TotalCloud;

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
            pcl::PointXYZ pt = cloud_filtered_v2->points[*pit];
            pcl::PointXYZI pt2;
            pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
            pt2.intensity = (float)(j+1);
            TotalCloud.push_back(pt2);
        }
        j++;
    }
    pcl::getMinMax3D(TotalCloud, minPt, maxPt);

    // std::cout << "Maximum height: " << maxPt.z <<std::endl;


    // Convert filtered point cloud from PCL format back to ROS message
    pcl::toPCLPointCloud2(TotalCloud, cloud_filtered);  
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_filtered, output);
    output.header.frame_id = point.header.frame_id;
    output.header.stamp = ros::Time::now();
    pub.publish(output);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Clustering");
    ros::NodeHandle node("~");
    std::cout<< "Start node for clustering......" << std::endl;
    Clustering::Ptr clusteringPtr;
    clusteringPtr.reset(new Clustering);
    clusteringPtr->initClustering(node);
    ros::Duration(1.0).sleep();
    ros::spin();

}