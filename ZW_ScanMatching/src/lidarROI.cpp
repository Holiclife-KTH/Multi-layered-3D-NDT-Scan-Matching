// ROS core
#include <ros/ros.h>

//pcl::toROSMsg
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <Eigen/Dense>
#include <cmath>

//Topic msg
#include <sensor_msgs/PointCloud2.h>
#include <ZW_ScanMatching/plane.h>
#include <ZW_ScanMatching/Coordinate.h>

using namespace std;

class LiDARROI{
    public:
        LiDARROI() {}
        ~LiDARROI() {}

        void initLiDARROI(ros::NodeHandle& nh);

        typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
        typedef std::shared_ptr<LiDARROI> Ptr;

        std::pair<float, float> l_result;
        std::pair<float, float> r_result;
        pcl::PCLPointCloud2 pcl_pcd;

    private:
        void LPlanecallback(const ZW_ScanMatching::plane& plane_msg);
        void RPlanecallback(const ZW_ScanMatching::plane& plane_msg);
        void Pointcallback(const sensor_msgs::PointCloud2& points);
        std::pair<float, float> calc_ab(std::pair<float, float> x, std::pair<float, float> y);

        ros::NodeHandle node;
        ros::Subscriber lplane_cb;
        ros::Subscriber rplane_cb;
        ros::Subscriber pcd_cb;
        ros::Publisher pointcloud_publisher;

};

void LiDARROI::initLiDARROI(ros::NodeHandle& nh){
    node = nh;
    lplane_cb = node.subscribe("/L_points", 10, &LiDARROI::LPlanecallback, this);
    rplane_cb = node.subscribe("/R_points", 10, &LiDARROI::RPlanecallback, this);
    pcd_cb = node.subscribe("/VLP16_pointcloud", 10, &LiDARROI::Pointcallback, this);
    pointcloud_publisher = node.advertise<sensor_msgs::PointCloud2>("/ROI_points", 10);
}

void LiDARROI::LPlanecallback(const ZW_ScanMatching::plane& plane_msg){
    // Caculate x, y coordinate which represents middle line of the plane
    float front_x_mean = (plane_msg.plane[0].x + plane_msg.plane[3].x)/2;
    float front_y_mean = (plane_msg.plane[0].y + plane_msg.plane[3].y)/2;
    float rear_x_mean = (plane_msg.plane[1].x + plane_msg.plane[2].x)/2;
    float rear_y_mean = (plane_msg.plane[1].y + plane_msg.plane[2].y)/2;

    // Calculate gradient and bias
    if (front_x_mean != 0 && front_y_mean != 0){
        l_result = calc_ab({front_x_mean, rear_x_mean}, {front_y_mean, rear_y_mean});
    }
    

    
    // cout << "l_result: " << l_result.first<< " " << l_result.second << endl;
}

void LiDARROI::RPlanecallback(const ZW_ScanMatching::plane& plane_msg){
    float front_x_mean = (plane_msg.plane[0].x + plane_msg.plane[3].x)/2;
    float front_y_mean = (plane_msg.plane[0].y + plane_msg.plane[3].y)/2;
    float rear_x_mean = (plane_msg.plane[1].x + plane_msg.plane[2].x)/2;
    float rear_y_mean = (plane_msg.plane[1].y + plane_msg.plane[2].y)/2;

    r_result = calc_ab({front_x_mean, rear_x_mean}, {front_y_mean, rear_y_mean});
    // cout << "r_result: " << r_result.first <<" " <<r_result.second << endl;
}

void LiDARROI::Pointcallback(const sensor_msgs::PointCloud2& points){
    pcl::PCLPointCloud2 pcl_pc; // temporary PointCloud2 intermediary
    pcl_conversions::toPCL(points, pcl_pc);
    PointCloud::Ptr input_ptr(new PointCloud());
    pcl::fromPCLPointCloud2(pcl_pc, *input_ptr);

    // for (int point_id = 0; point_id < input_ptr->points.size(); ++point_id) {
    //     if((((input_ptr->points[point_id].x)*r_result.first + r_result.second + 0.35 - input_ptr->points[point_id].y)> 0)||(((input_ptr->points[point_id].x)*l_result.first + l_result.second - 0.35 - input_ptr->points[point_id].y)< 0)){
    //         input_ptr->points[point_id].x = 0;
    //         input_ptr->points[point_id].y = 0;
    //         input_ptr->points[point_id].z = 0;
    //     }    
    // }

    for (int point_id = 0; point_id < input_ptr->points.size(); ++point_id) {
        if(((input_ptr->points[point_id].y)> 1.7)||((input_ptr->points[point_id].y)< -1.7)){
            input_ptr->points[point_id].x = 0;
            input_ptr->points[point_id].y = 0;
            input_ptr->points[point_id].z = 0;
        }    
    }


    pcl::toPCLPointCloud2(*input_ptr, pcl_pcd);
    sensor_msgs::PointCloud2 ROI_output;
    pcl_conversions::fromPCL(pcl_pcd,ROI_output);
    ROI_output.header.frame_id="VLP16";
    ROI_output.header.stamp  = ros::Time::now();
    pointcloud_publisher.publish(ROI_output);
}

std::pair<float, float> LiDARROI::calc_ab(std::pair<float, float> x, std::pair<float, float> y){
    float a = (y.second-y.first)/(x.second-x.first);
    float b = (x.second*y.first-x.first*y.second)/(x.second-x.first);

    return std::make_pair(a, b);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "LiDAR_ROI");
    ros::NodeHandle node("~");

    std::cout<< "Start node to set LiDAR ROI...... " << std::endl;
 
    LiDARROI::Ptr lidarROI;

    lidarROI.reset(new LiDARROI);
    lidarROI->initLiDARROI(node);
    ros::Duration(1.0).sleep();
    ros::spin();

    return 0;
}