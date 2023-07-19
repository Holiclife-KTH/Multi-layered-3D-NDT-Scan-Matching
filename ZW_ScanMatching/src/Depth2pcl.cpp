// ROS core
#include <ros/ros.h>

//Image message
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>


//pcl::toROSMsg
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <Eigen/Dense>
#include <cmath>

using namespace std;

class DepthPcl{
    public:
        DepthPcl() {}
        ~DepthPcl() {}

        void initDepthPcl(ros::NodeHandle& nh);

        cv::Mat depth_pic;
        cv::Mat rgb_pic;
        cv::Mat color_filtered_pic;
        cv::Mat filtered_pic;
        std::vector<cv::Vec4i> lines;
        std::vector<std::pair<int, int>> pt_x;
        std::vector<std::pair<int, int>> pt_y;
        

        typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
        typedef std::shared_ptr<DepthPcl> Ptr;
    
    private:
        void depthCallback(const sensor_msgs::ImageConstPtr& depth_msgs);
        void stateCallback(const ros::TimerEvent&);
        void rgbCallback(const sensor_msgs::ImageConstPtr& rgb_msgs);
        cv::Mat RGB_color_masking(const cv::Mat& img);
        cv::Mat Img_filltering(const cv::Mat& img);
        std::vector<cv::Vec4i> calcHoughLines(const cv::Mat& img);
        std::pair<double, double> calc_ab(std::pair<int, int> x, std::pair<int, int> y);

        


        ros::NodeHandle node;
        ros::Subscriber depth_cb;
        ros::Subscriber rgb_cb;
        ros::Publisher pointcloud_publisher;
        ros::Timer state_timer_;
};

void DepthPcl::initDepthPcl(ros::NodeHandle& nh){
    node = nh;

    depth_cb = node.subscribe<sensor_msgs::Image>("/depth_img", 50, &DepthPcl::depthCallback, this);

    rgb_cb = node.subscribe<sensor_msgs::Image>("/rgb", 50, &DepthPcl::rgbCallback, this); 

    pointcloud_publisher = node.advertise<sensor_msgs::PointCloud2>("/depth/points", 10);

    state_timer_ = node.createTimer(ros::Duration(0.05), &DepthPcl::stateCallback, this);
}

void DepthPcl::depthCallback(const sensor_msgs::ImageConstPtr& depth_msg) {
    // std::cout << "image: " << depth_msg->header.stamp << std::endl;

    cv_bridge::CvImagePtr depth_ptr;
    try
    {
        depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);


    }
    catch(cv_bridge::Exception& e)
    {
        //ROS_ERROR("Could not convert from '%s' to 'mono16'.", depth_msg->encoding.c_str());
    }
    depth_pic = depth_ptr->image;
}

//subscribe rgb image from Isaac sim
void DepthPcl::rgbCallback(const sensor_msgs::ImageConstPtr& rgb_msg) {
    cv_bridge::CvImagePtr rgb_ptr;

    try
    {
        rgb_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
        //ROS_ERROR("Could not convert from '%s' to 'mono16'.", rgb_msg->encoding.c_str());
    }
    rgb_pic = rgb_ptr->image;
    color_filtered_pic = RGB_color_masking(rgb_pic);
    filtered_pic = Img_filltering(color_filtered_pic);
    lines = calcHoughLines(filtered_pic);

    for (size_t i = 0; i < lines.size(); i++){
        cv::Vec4i l = lines[0];
        double angle = std::abs(std::atan2(l[3]-l[1], l[2]-l[0]));
        if (angle > 0.1 && angle < 1.5){
            bool found = false;
            for (size_t j = 0; j < pt_x.size(); j++){
                if (std::abs(pt_x[j].first - l[0]) < 450 && std::abs(pt_y[j].first - l[1]) < 80) {
                            found = true;
                            break;
                }
            }
            if (!found){
                pt_x.emplace_back(l[0], l[2]);
                pt_y.emplace_back(l[1], l[3]);
            }
        }
    }

    // for(size_t i = 0; i < lines.size(); i++){
    //     cv::Vec4i l = lines[i];
    //     cv::line(rgb_pic, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 2, cv::LINE_AA);
    // }
    cout<<pt_x.size()<<endl;
    for (size_t j = 0; j < pt_x.size(); j++){
        cv::line(rgb_pic, cv::Point(pt_x[j].first, pt_y[j].first), cv::Point(pt_x[j].second, pt_y[j].second), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }
    pt_x.clear();
    pt_y.clear();
    lines.clear();



    // for (size_t i = 0; i < pt_x.size(); i++) {
    //     std::pair<double, double> result;
    //     result = calc_ab(pt_x[i], pt_y[i]);
    //     if (pt_x[i].first < 500) {
    //         cv::line(rgb_pic, cv::Point(0, int(result.second)), cv::Point(int(rgb_pic.cols / 2), int(rgb_pic.cols / 2 * result.first + result.second)), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    //     } 
    //     else {
    //         cv::line(rgb_pic, cv::Point(int(rgb_pic.cols / 2), int(rgb_pic.cols / 2 * result.first + result.second)), cv::Point(int(rgb_pic.cols), int(rgb_pic.cols * result.first + result.second)), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    //     }
    // }

    cv::imshow("image", rgb_pic);
    // cv::imshow("color_filter", color_filtered_pic);
    // cv::imshow("filter", filtered_pic);
    cv::waitKey(0);
}

cv::Mat DepthPcl::RGB_color_masking(const cv::Mat& img) {
    cv::Mat mask;
    cv::Mat orange_mask;
    cv::Mat yellow_mask;
    cv::Mat img_orange_filtered;
    cv::Mat img_yellow_filtered;
    cv::Mat img_color_filtered;

    // Orange Mask
    vector<int> lower_orange = {5, 20, 100};
    vector<int> upper_orange = {40, 180, 185};
    cv::inRange(img, lower_orange, upper_orange, orange_mask);
    cv::bitwise_and(img, img, img_orange_filtered, orange_mask);

    // Yellow Mask
    vector<int> lower_yellow = {0, 185, 185};
    vector<int> upper_yellow = {40, 255, 255};
    cv::inRange(img, lower_yellow, upper_yellow, yellow_mask);
    cv::bitwise_and(img, img, img_yellow_filtered, yellow_mask);

    cv::addWeighted(img_orange_filtered, 1.0, img_yellow_filtered, 1.0, 0.0, img_color_filtered);
    
    return img_color_filtered;
}

cv::Mat DepthPcl::Img_filltering(const cv::Mat& img){
    cv::Mat Gray;
    cv::Mat Blurring;
    cv::Mat Canny;

    cv::cvtColor(img, Gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(Gray, Blurring, cv::Size(5, 5), 0);
    cv::Canny(Blurring, Canny, 50, 150);

    return Canny;
}

std::vector<cv::Vec4i> DepthPcl::calcHoughLines(const cv::Mat& img){
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(img, lines, 1, 3.1415926535/180, 50, 65, 300);
    return lines;
}

std::pair<double, double> DepthPcl::calc_ab(std::pair<int, int> x, std::pair<int, int> y){
    double a = (y.second-y.first)/(x.second-x.first);
    double b = (x.second*y.first-x.first*y.second)/(x.second-x.first);

    return std::make_pair(a, b);
}

void DepthPcl::stateCallback(const ros::TimerEvent& ){
    sensor_msgs::PointCloud2 pub_pointcloud;
    PointCloud::Ptr cloud_msg (new PointCloud);

    // Use correct principal point && focal length from calibration
    const double camera_factor = 1;
    const double camera_cx = 400;
    const double camera_cy = 400;
    const double camera_fx = 24;
    const double camera_fy = 24;

    // Traverse the depth image
    for (int v = 0; v < depth_pic.rows; ++v )
    {
        for (int u = 0; u < depth_pic.cols; ++u){
            
            float d = depth_pic.ptr<float>(v)[u];

        // Check for invalid measurements
        if (d==0)
            continue;

        pcl::PointXYZ pt;

        pt.z = double(d) / camera_factor;
        pt.x = (u - camera_cx) * pt.z / camera_fx;
        pt.y = (v - camera_cy) * pt.z / camera_fy;

        // add p to the point cloud
        cloud_msg->points.push_back(pt);
        }

    }

    cloud_msg->height = 1;
    cloud_msg->width = cloud_msg->points.size();
    cloud_msg->is_dense = false;

    // converting a PCL point cloud to a ROS PCL message
    pcl::toROSMsg(*cloud_msg, pub_pointcloud);
    pub_pointcloud.header.frame_id = "Camera";
    pub_pointcloud.header.stamp = ros::Time::now();

    // Publishing our cloud image
    pointcloud_publisher.publish(pub_pointcloud);

    cloud_msg->points.clear();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_pointcloud");
    ros::NodeHandle node("~");

    std::cout<< "Starting to show full pointcloud message <pcl::PointCloud<pcl::PointXYZ>> .... " << std::endl;
    DepthPcl::Ptr depth_pcl;

    depth_pcl.reset(new DepthPcl);
    depth_pcl->initDepthPcl(node);

    ros::Duration(1.0).sleep();
    ros::spin();

    return 0;
}

 

