/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 Localization program using Normal Distributions Transform

 Yuki KITSUKAWA
 */

#include <pthread.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <velodyne_pcl/point_types.h>
#include <velodyne_pointcloud/rawdata.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ndt_cpu/NormalDistributionsTransform.h>
#include <pcl/registration/ndt.h>
#ifdef CUDA_FOUND
#include <ndt_gpu/NormalDistributionsTransform.h>
#endif
#ifdef USE_PCL_OPENMP
#include <pcl_omp_registration/ndt.h>
#endif

#include <autoware_config_msgs/ConfigNDT.h>

#include <autoware_msgs/NDTStat.h>

// headers in Autoware Health Checker
#include <autoware_health_checker/health_checker/health_checker.h>

#define PREDICT_POSE_THRESHOLD 0.5

#define Wa 0.4
#define Wb 0.3
#define Wc 0.3

static std::shared_ptr<autoware_health_checker::HealthChecker> health_checker_ptr_;

struct pose
{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

enum class MethodType
{
    PCL_GENERIC = 0,
    PCL_ANH = 1,
    PCL_ANH_GPU = 2,
    PCL_OPENMP = 3,
};
static MethodType _method_type = MethodType::PCL_GENERIC;

static pose initial_pose, bottom_predict_pose, bottom_previous_pose, bottom_ndt_pose, bottom_current_pose, bottom_localizer_pose,
    top_predict_pose, top_previous_pose, top_ndt_pose, top_current_pose, top_localizer_pose;

static double bottom_offset_x, bottom_offset_y, bottom_offset_z, bottom_offset_yaw;
static double top_offset_x, top_offset_y, top_offset_z, top_offset_yaw; // current_pos - previous_pose

static pcl::PointCloud<pcl::PointXYZ> map;

// If the map is loaded, map_loaded will be 1.
static int map_loaded = 0;
static int _use_gnss = 1;
static int init_pos_set = 0;

static pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
static cpu::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> anh_ndt;
#ifdef CUDA_FOUND
static std::shared_ptr<gpu::GNormalDistributionsTransform> anh_gpu_ndt_ptr =
    std::make_shared<gpu::GNormalDistributionsTransform>();
#endif
#ifdef USE_PCL_OPENMP
static pcl_omp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> omp_ndt;
#endif

// Default values
static int max_iter = 30;       // Maximum iterations
static float ndt_res = 1.0;     // Resolution
static double step_size = 0.1;  // Step size
static double trans_eps = 0.01; // Transformation epsilon
int feedback_check = 0;

static ros::Publisher bottom_predict_pose_pub;
static geometry_msgs::PoseStamped bottom_predict_pose_msg;

static ros::Publisher top_predict_pose_pub;
static geometry_msgs::PoseStamped top_predict_pose_msg;

static ros::Publisher bottom_ndt_pose_pub;
static geometry_msgs::PoseStamped bottom_ndt_pose_msg;

static ros::Publisher top_ndt_pose_pub;
static geometry_msgs::PoseStamped top_ndt_pose_msg;

static ros::Publisher bottom_localizer_pose_pub;
static ros::Publisher top_localizer_pose_pub;
static geometry_msgs::PoseWithCovarianceStamped bottom_localizer_pose_msg;
static geometry_msgs::PoseWithCovarianceStamped top_localizer_pose_msg;


static ros::Duration scan_duration;

static double bottom_exe_time = 0.0;
static bool bottom_has_converged;
static int bottom_iteration = 0;
static double bottom_fitness_score = 0.0;
static double bottom_trans_probability = 0.0;
static Eigen::Matrix<double, 6, 6> bottom_hessian_inv;
static Eigen::Matrix<double, 6, 6> bottom_cov;

static double top_exe_time = 0.0;
static bool top_has_converged;
static int top_iteration = 0;
static double top_fitness_score = 0.0;
static double top_trans_probability = 0.0;
static Eigen::Matrix<double, 6, 6> top_hessian_inv;
static Eigen::Matrix<double, 6, 6> top_cov;

static double bottom_diff = 0.0;
static double bottom_diff_x = 0.0, bottom_diff_y = 0.0, bottom_diff_z = 0.0, bottom_diff_yaw;

static double top_diff = 0.0;
static double top_diff_x = 0.0, top_diff_y = 0.0, top_diff_z = 0.0, top_diff_yaw;

static double bottom_current_velocity = 0.0, bottom_previous_velocity = 0.0, bottom_previous_previous_velocity = 0.0; // [m/s]
static double bottom_current_velocity_x = 0.0, bottom_previous_velocity_x = 0.0;
static double bottom_current_velocity_y = 0.0, bottom_previous_velocity_y = 0.0;
static double bottom_current_velocity_z = 0.0, bottom_previous_velocity_z = 0.0;

static double bottom_current_accel = 0.0, bottom_previous_accel = 0.0; // [m/s^2]
static double bottom_current_accel_x = 0.0;
static double bottom_current_accel_y = 0.0;
static double bottom_current_accel_z = 0.0;

static double bottom_angular_velocity = 0.0;

static double top_current_velocity = 0.0, top_previous_velocity = 0.0, top_previous_previous_velocity = 0.0; // [m/s]
static double top_current_velocity_x = 0.0, top_previous_velocity_x = 0.0;
static double top_current_velocity_y = 0.0, top_previous_velocity_y = 0.0;
static double top_current_velocity_z = 0.0, top_previous_velocity_z = 0.0;

static double top_current_accel = 0.0, top_previous_accel = 0.0; // [m/s^2]
static double top_current_accel_x = 0.0;
static double top_current_accel_y = 0.0;
static double top_current_accel_z = 0.0;

static double top_angular_velocity = 0.0;

static int bottom_use_predict_pose = 0;
static int top_use_predict_pose = 0;

static std::chrono::time_point<std::chrono::system_clock> bottom_matching_start, bottom_matching_end, top_matching_start, top_matching_end;

static ros::Publisher bottom_time_ndt_matching_pub;
static std_msgs::Float32 bottom_time_ndt_matching;

static ros::Publisher top_time_ndt_matching_pub;
static std_msgs::Float32 top_time_ndt_matching;

static int _queue_size = 1;

static ros::Publisher top_ndt_stat_pub;
static autoware_msgs::NDTStat top_ndt_stat_msg;

static ros::Publisher bottom_ndt_stat_pub;
static autoware_msgs::NDTStat bottom_ndt_stat_msg;

static double bottom_predict_pose_error = 0.0;
static double top_predict_pose_error = 0.0;

// hold transfrom from baselink to primary lidar
static Eigen::Matrix4f tf_btol;

static std::string _offset = "linear"; // linear, zero, quadratic

static ros::Publisher bottom_ndt_reliability_pub;
static std_msgs::Float32 bottom_ndt_reliability;

static ros::Publisher top_ndt_reliability_pub;
static std_msgs::Float32 top_ndt_reliability;

static bool _get_height = false;
static bool _use_local_transform = false;
static bool _use_imu = false;
static bool _use_odom = false;
static bool _imu_upside_down = false;
static bool _output_log_data = false;
static std::string _output_tf_frame_id = "chassis_link";
static std::string _map_topic = "/points_map";

static std::string _imu_topic = "/imu_raw";

static std::ofstream ofs;
static std::string filename;

static tf2::Stamped<tf2::Transform> local_transform;

static unsigned int points_map_num = 0;

pthread_mutex_t mutex;

static pose convertPoseIntoRelativeCoordinate(const pose &target_pose, const pose &reference_pose)
{
    tf2::Quaternion target_q;
    target_q.setRPY(target_pose.roll, target_pose.pitch, target_pose.yaw);
    tf2::Vector3 target_v(target_pose.x, target_pose.y, target_pose.z);
    tf2::Transform target_tf(target_q, target_v);

    tf2::Quaternion reference_q;
    reference_q.setRPY(reference_pose.roll, reference_pose.pitch, reference_pose.yaw);
    tf2::Vector3 reference_v(reference_pose.x, reference_pose.y, reference_pose.z);
    tf2::Transform reference_tf(reference_q, reference_v);

    tf2::Transform trans_target_tf = reference_tf.inverse() * target_tf;

    pose trans_target_pose;
    trans_target_pose.x = trans_target_tf.getOrigin().getX();
    trans_target_pose.y = trans_target_tf.getOrigin().getY();
    trans_target_pose.z = trans_target_tf.getOrigin().getZ();
    tf2::Matrix3x3 tmp_m(trans_target_tf.getRotation());
    tmp_m.getRPY(trans_target_pose.roll, trans_target_pose.pitch, trans_target_pose.yaw);

    return trans_target_pose;
}

static void map_callback(const sensor_msgs::PointCloud2::ConstPtr &input)
{
    // if (map_loaded == 0)
    if (points_map_num != input->width)
    {
        std::cout << "Update points_map." << std::endl;

        points_map_num = input->width;

        // Convert the data type(from sensor_msgs to pcl).
        pcl::fromROSMsg(*input, map);

        if (_use_local_transform == true)
        {
            tf2_ros::Buffer tf_buffer;
            tf2_ros::TransformListener tf_listener(tf_buffer);
            geometry_msgs::TransformStamped local_transform_msg;
            try
            {
                local_transform_msg = tf_buffer.lookupTransform("map", "world", ros::Time::now(), ros::Duration(3.0));
            }
            catch (tf2::TransformException &ex)
            {
                ROS_ERROR("%s", ex.what());
            }

            tf2::fromMsg(local_transform_msg, local_transform);
            pcl::transformPointCloud(map, map, tf2::transformToEigen(local_transform_msg).matrix().inverse().cast<float>());
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZ>(map));

        // Setting point cloud to be aligned to.
        if (_method_type == MethodType::PCL_GENERIC)
        {
            pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> new_ndt;
            pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            new_ndt.setResolution(ndt_res);
            new_ndt.setInputTarget(map_ptr);
            new_ndt.setMaximumIterations(max_iter);
            new_ndt.setStepSize(step_size);
            new_ndt.setTransformationEpsilon(trans_eps);

            new_ndt.align(*output_cloud, Eigen::Matrix4f::Identity());

            pthread_mutex_lock(&mutex);
            ndt = new_ndt;
            pthread_mutex_unlock(&mutex);
        }
        else if (_method_type == MethodType::PCL_ANH)
        {
            cpu::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> new_anh_ndt;
            new_anh_ndt.setResolution(ndt_res);
            new_anh_ndt.setInputTarget(map_ptr);
            new_anh_ndt.setMaximumIterations(max_iter);
            new_anh_ndt.setStepSize(step_size);
            new_anh_ndt.setTransformationEpsilon(trans_eps);

            pcl::PointCloud<pcl::PointXYZ>::Ptr dummy_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointXYZ dummy_point;
            dummy_scan_ptr->push_back(dummy_point);
            new_anh_ndt.setInputSource(dummy_scan_ptr);

            new_anh_ndt.align(Eigen::Matrix4f::Identity());

            pthread_mutex_lock(&mutex);
            anh_ndt = new_anh_ndt;
            pthread_mutex_unlock(&mutex);
        }
        map_loaded = 1;
    }
}

static void initialpose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &input)
{
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    geometry_msgs::TransformStamped tf_msg;
    try
    {
        tf_msg = tf_buffer.lookupTransform("map", input->header.frame_id, ros::Time::now(), ros::Duration(3.0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    tf2::Quaternion q(input->pose.pose.orientation.x, input->pose.pose.orientation.y, input->pose.pose.orientation.z,
                      input->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);

    if (_use_local_transform == true)
    {
        bottom_current_pose.x = input->pose.pose.position.x;
        bottom_current_pose.y = input->pose.pose.position.y;
        bottom_current_pose.z = input->pose.pose.position.z;
        top_current_pose.x = input->pose.pose.position.x;
        top_current_pose.y = input->pose.pose.position.y;
        top_current_pose.z = input->pose.pose.position.z;
    }
    else
    {
        bottom_current_pose.x = input->pose.pose.position.x + tf_msg.transform.translation.x;
        bottom_current_pose.y = input->pose.pose.position.y + tf_msg.transform.translation.y;
        bottom_current_pose.z = input->pose.pose.position.z + tf_msg.transform.translation.z;
        top_current_pose.x = input->pose.pose.position.x + tf_msg.transform.translation.x;
        top_current_pose.y = input->pose.pose.position.y + tf_msg.transform.translation.y;
        top_current_pose.z = input->pose.pose.position.z + tf_msg.transform.translation.z;
    }
    m.getRPY(bottom_current_pose.roll, bottom_current_pose.pitch, bottom_current_pose.yaw);
    m.getRPY(top_current_pose.roll, top_current_pose.pitch, top_current_pose.yaw);
    if (_get_height == true && map_loaded == 1)
    {
        double min_distance = DBL_MAX;
        double nearest_z = top_current_pose.z;
        for (const auto &p : map)
        {
            double distance = hypot(top_current_pose.x - p.x, top_current_pose.y - p.y);
            if (distance < min_distance)
            {
                min_distance = distance;
                nearest_z = p.z;
            }
        }
        bottom_current_pose.z = nearest_z;
        top_current_pose.z = nearest_z;
    }

    bottom_previous_pose.x = bottom_current_pose.x;
    bottom_previous_pose.y = bottom_current_pose.y;
    bottom_previous_pose.z = bottom_current_pose.z;
    bottom_previous_pose.roll = bottom_current_pose.roll;
    bottom_previous_pose.pitch = bottom_current_pose.pitch;
    bottom_previous_pose.yaw = bottom_current_pose.yaw;
    top_previous_pose.x = bottom_current_pose.x;
    top_previous_pose.y = bottom_current_pose.y;
    top_previous_pose.z = bottom_current_pose.z;
    top_previous_pose.roll = bottom_current_pose.roll;
    top_previous_pose.pitch = bottom_current_pose.pitch;
    top_previous_pose.yaw = bottom_current_pose.yaw;

    bottom_offset_x = 0.0;
    bottom_offset_y = 0.0;
    bottom_offset_z = 0.0;
    bottom_offset_yaw = 0.0;

    top_offset_x = 0.0;
    top_offset_y = 0.0;
    top_offset_z = 0.0;
    top_offset_yaw = 0.0;

    init_pos_set = 1;
}

static double wrapToPm(double a_num, const double a_max)
{
    if (a_num >= a_max)
    {
        a_num -= 2.0 * a_max;
    }
    return a_num;
}

static double wrapToPmPi(const double a_angle_rad)
{
    return wrapToPm(a_angle_rad, M_PI);
}

static double calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
    double diff_rad = lhs_rad - rhs_rad;
    if (diff_rad >= M_PI)
        diff_rad = diff_rad - 2 * M_PI;
    else if (diff_rad < -M_PI)
        diff_rad = diff_rad + 2 * M_PI;
    return diff_rad;
}

static void bottom_points_callback(const sensor_msgs::PointCloud2::ConstPtr &input)
{
    health_checker_ptr_->CHECK_RATE("topic_rate_filtered_points_slow", 8, 5, 1, "topic filtered_points subscribe rate slow.");
    if (map_loaded == 1 && init_pos_set == 1)
    {
        bottom_matching_start = std::chrono::system_clock::now();

        static tf2_ros::TransformBroadcaster br;
        tf2::Transform transform;
        tf2::Quaternion predict_q, ndt_q, current_q, localizer_q;

        pcl::PointXYZ p;
        pcl::PointCloud<pcl::PointXYZ> filtered_scan;

        ros::Time current_scan_time = ros::Time().now();
        // ros::Time current_scan_time = input->header.stamp;
        static ros::Time previous_scan_time = current_scan_time;

        pcl::fromROSMsg(*input, filtered_scan);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(filtered_scan));
        int scan_points_num = filtered_scan_ptr->size();
        // Eigen::Matrix<double, 6, 1> deviation;

        Eigen::Matrix4f t(Eigen::Matrix4f::Identity());  // base_link
        Eigen::Matrix4f t2(Eigen::Matrix4f::Identity()); // localizer

        std::chrono::time_point<std::chrono::system_clock> align_start, align_end, getFitnessScore_start,
            getFitnessScore_end;
        static double align_time, getFitnessScore_time = 0.0;

        pthread_mutex_lock(&mutex);

        if (_method_type == MethodType::PCL_GENERIC)
            ndt.setInputSource(filtered_scan_ptr);
        else if (_method_type == MethodType::PCL_ANH)
            anh_ndt.setInputSource(filtered_scan_ptr);

        // Guess the initial gross estimation of the transformation
        double diff_time = (current_scan_time - previous_scan_time).toSec();

        if (_offset == "linear")
        {
            bottom_offset_x = bottom_current_velocity_x * diff_time;
            bottom_offset_y = bottom_current_velocity_y * diff_time;
            bottom_offset_z = bottom_current_velocity_z * diff_time;
            bottom_offset_yaw = bottom_angular_velocity * diff_time;
        }
        else if (_offset == "quadratic")
        {
            bottom_offset_x = (bottom_current_velocity_x + bottom_current_accel_x * diff_time) * diff_time;
            bottom_offset_y = (bottom_current_velocity_y + bottom_current_accel_y * diff_time) * diff_time;
            bottom_offset_z = bottom_current_velocity_z * diff_time;
            bottom_offset_yaw = bottom_angular_velocity * diff_time;
        }
        else if (_offset == "zero")
        {
            bottom_offset_x = 0.0;
            bottom_offset_y = 0.0;
            bottom_offset_z = 0.0;
            bottom_offset_yaw = 0.0;
        }

        bottom_predict_pose.x = bottom_previous_pose.x + bottom_offset_x;
        bottom_predict_pose.y = bottom_previous_pose.y + bottom_offset_y;
        bottom_predict_pose.z = -0.49;
        bottom_predict_pose.roll = 0;
        bottom_predict_pose.pitch = 0;
        bottom_predict_pose.yaw = bottom_previous_pose.yaw + bottom_offset_yaw;

        pose predict_pose_for_ndt;
        predict_pose_for_ndt = bottom_predict_pose;

        Eigen::Translation3f init_translation(predict_pose_for_ndt.x, predict_pose_for_ndt.y, predict_pose_for_ndt.z);
        Eigen::AngleAxisf init_rotation_x(predict_pose_for_ndt.roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf init_rotation_y(predict_pose_for_ndt.pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf init_rotation_z(predict_pose_for_ndt.yaw, Eigen::Vector3f::UnitZ());
        Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x) * tf_btol;

        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        if (_method_type == MethodType::PCL_GENERIC)
        {
            align_start = std::chrono::system_clock::now();
            ndt.align(*output_cloud, init_guess);
            align_end = std::chrono::system_clock::now();

            bottom_has_converged = ndt.hasConverged();

            t = ndt.getFinalTransformation();
            bottom_iteration = ndt.getFinalNumIteration();

            getFitnessScore_start = std::chrono::system_clock::now();
            bottom_fitness_score = ndt.getFitnessScore();
            getFitnessScore_end = std::chrono::system_clock::now();

            bottom_trans_probability = ndt.getTransformationProbability();
        }
        else if (_method_type == MethodType::PCL_ANH)
        {
            align_start = std::chrono::system_clock::now();
            anh_ndt.align(init_guess);
            align_end = std::chrono::system_clock::now();

            bottom_has_converged = anh_ndt.hasConverged();

            t = anh_ndt.getFinalTransformation();
            bottom_iteration = anh_ndt.getFinalNumIteration();

            getFitnessScore_start = std::chrono::system_clock::now();
            bottom_fitness_score = anh_ndt.getFitnessScore();
            getFitnessScore_end = std::chrono::system_clock::now();

            bottom_trans_probability = anh_ndt.getTransformationProbability();

            bottom_hessian_inv = anh_ndt.getHessian();
            // Calculate covariance of estimated posoe
            bottom_cov = bottom_fitness_score * bottom_hessian_inv;
        }

        align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end - align_start).count() / 1000.0;

        t2 = t * tf_btol.inverse();

        getFitnessScore_time =
            std::chrono::duration_cast<std::chrono::microseconds>(getFitnessScore_end - getFitnessScore_start).count() /
            1000.0;

        pthread_mutex_unlock(&mutex);

        tf2::Matrix3x3 mat_l; // localizer
        mat_l.setValue(static_cast<double>(t(0, 0)), static_cast<double>(t(0, 1)), static_cast<double>(t(0, 2)),
                       static_cast<double>(t(1, 0)), static_cast<double>(t(1, 1)), static_cast<double>(t(1, 2)),
                       static_cast<double>(t(2, 0)), static_cast<double>(t(2, 1)), static_cast<double>(t(2, 2)));

        // Update localizer_pose
        bottom_localizer_pose.x = t(0, 3);
        bottom_localizer_pose.y = t(1, 3);
        bottom_localizer_pose.z = t(2, 3);
        mat_l.getRPY(bottom_localizer_pose.roll, bottom_localizer_pose.pitch, bottom_localizer_pose.yaw, 1);
        // mat_l.getRPY(0.0, 0.0, localizer_pose.yaw, 1);

        tf2::Matrix3x3 mat_b; // base_link
        mat_b.setValue(static_cast<double>(t2(0, 0)), static_cast<double>(t2(0, 1)), static_cast<double>(t2(0, 2)),
                       static_cast<double>(t2(1, 0)), static_cast<double>(t2(1, 1)), static_cast<double>(t2(1, 2)),
                       static_cast<double>(t2(2, 0)), static_cast<double>(t2(2, 1)), static_cast<double>(t2(2, 2)));

        // Update ndt_pose
        bottom_ndt_pose.x = t2(0, 3);
        bottom_ndt_pose.y = t2(1, 3);
        bottom_ndt_pose.z = t2(2, 3);
        mat_b.getRPY(bottom_ndt_pose.roll, bottom_ndt_pose.pitch, bottom_ndt_pose.yaw, 1);

        // Calculate the difference between ndt_pose and predict_pose
        bottom_predict_pose_error = sqrt((bottom_ndt_pose.x - predict_pose_for_ndt.x) * (bottom_ndt_pose.x - predict_pose_for_ndt.x) +
                                  (bottom_ndt_pose.y - predict_pose_for_ndt.y) * (bottom_ndt_pose.y - predict_pose_for_ndt.y) +
                                  (bottom_ndt_pose.z - predict_pose_for_ndt.z) * (bottom_ndt_pose.z - predict_pose_for_ndt.z));

        if (bottom_predict_pose_error <= PREDICT_POSE_THRESHOLD)
        {
            bottom_use_predict_pose = 0;
        }
        else
        {
            bottom_use_predict_pose = 1;
        }
        bottom_use_predict_pose = 0;

        if (bottom_use_predict_pose == 0)
        {
            bottom_current_pose.x = bottom_ndt_pose.x;
            bottom_current_pose.y = bottom_ndt_pose.y;
            bottom_current_pose.z = -0.49;
            bottom_current_pose.roll = 0;
            bottom_current_pose.pitch = 0;
            bottom_current_pose.yaw = bottom_ndt_pose.yaw;
        }
        else
        {
            bottom_current_pose.x = predict_pose_for_ndt.x;
            bottom_current_pose.y = predict_pose_for_ndt.y;
            bottom_current_pose.z = -0.49;
            bottom_current_pose.roll = 0;
            bottom_current_pose.pitch = 0;
            bottom_current_pose.yaw = predict_pose_for_ndt.yaw;
        }

        // Compute the velocity and acceleration
        bottom_diff_x = bottom_current_pose.x - bottom_previous_pose.x;
        bottom_diff_y = bottom_current_pose.y - bottom_previous_pose.y;
        bottom_diff_z = bottom_current_pose.z - bottom_previous_pose.z;
        bottom_diff_yaw = calcDiffForRadian(bottom_current_pose.yaw, bottom_previous_pose.yaw);
        bottom_diff = sqrt(bottom_diff_x * bottom_diff_x + bottom_diff_y * bottom_diff_y + bottom_diff_z * bottom_diff_z);

        const pose trans_current_pose = convertPoseIntoRelativeCoordinate(bottom_current_pose, bottom_previous_pose);

        bottom_current_velocity = (diff_time > 0) ? (bottom_diff / diff_time) : 0;
        bottom_current_velocity = (trans_current_pose.x >= 0) ? bottom_current_velocity : -bottom_current_velocity;
        bottom_current_velocity_x = (diff_time > 0) ? (bottom_diff_x / diff_time) : 0;
        bottom_current_velocity_y = (diff_time > 0) ? (bottom_diff_y / diff_time) : 0;
        bottom_current_velocity_z = (diff_time > 0) ? (bottom_diff_z / diff_time) : 0;
        bottom_angular_velocity = (diff_time > 0) ? (bottom_diff_yaw / diff_time) : 0;


        bottom_current_accel = (diff_time > 0) ? ((bottom_current_velocity - bottom_previous_velocity) / diff_time) : 0;
        bottom_current_accel_x = (diff_time > 0) ? ((bottom_current_velocity_x - bottom_previous_velocity_x) / diff_time) : 0;
        bottom_current_accel_y = (diff_time > 0) ? ((bottom_current_velocity_y - bottom_previous_velocity_y) / diff_time) : 0;
        bottom_current_accel_z = (diff_time > 0) ? ((bottom_current_velocity_z - bottom_previous_velocity_z) / diff_time) : 0;


        // Set values for publishing pose
        predict_q.setRPY(bottom_predict_pose.roll, bottom_predict_pose.pitch, bottom_predict_pose.yaw);
        if (_use_local_transform == true)
        {
            tf2::Vector3 v(bottom_predict_pose.x, bottom_predict_pose.y, bottom_predict_pose.z);
            tf2::Transform transform(predict_q, v);
            bottom_predict_pose_msg.header.frame_id = "map";
            bottom_predict_pose_msg.header.stamp = current_scan_time;
            bottom_predict_pose_msg.pose.position.x = (local_transform * transform).getOrigin().getX();
            bottom_predict_pose_msg.pose.position.y = (local_transform * transform).getOrigin().getY();
            bottom_predict_pose_msg.pose.position.z = (local_transform * transform).getOrigin().getZ();
            bottom_predict_pose_msg.pose.orientation.x = (local_transform * transform).getRotation().x();
            bottom_predict_pose_msg.pose.orientation.y = (local_transform * transform).getRotation().y();
            bottom_predict_pose_msg.pose.orientation.z = (local_transform * transform).getRotation().z();
            bottom_predict_pose_msg.pose.orientation.w = (local_transform * transform).getRotation().w();
        }
        else
        {
            bottom_predict_pose_msg.header.frame_id = "map";
            bottom_predict_pose_msg.header.stamp = current_scan_time;
            bottom_predict_pose_msg.pose.position.x = bottom_predict_pose.x;
            bottom_predict_pose_msg.pose.position.y = bottom_predict_pose.y;
            bottom_predict_pose_msg.pose.position.z = bottom_predict_pose.z;
            bottom_predict_pose_msg.pose.orientation.x = predict_q.x();
            bottom_predict_pose_msg.pose.orientation.y = predict_q.y();
            bottom_predict_pose_msg.pose.orientation.z = predict_q.z();
            bottom_predict_pose_msg.pose.orientation.w = predict_q.w();
        }

        ndt_q.setRPY(bottom_ndt_pose.roll, bottom_ndt_pose.pitch, bottom_ndt_pose.yaw);
        if (_use_local_transform == true)
        {
            tf2::Vector3 v(bottom_ndt_pose.x, bottom_ndt_pose.y, bottom_ndt_pose.z);
            tf2::Transform transform(ndt_q, v);
            bottom_ndt_pose_msg.header.frame_id = "map";
            bottom_ndt_pose_msg.header.stamp = current_scan_time;
            bottom_ndt_pose_msg.pose.position.x = (local_transform * transform).getOrigin().getX();
            bottom_ndt_pose_msg.pose.position.y = (local_transform * transform).getOrigin().getY();
            bottom_ndt_pose_msg.pose.position.z = (local_transform * transform).getOrigin().getZ();
            bottom_ndt_pose_msg.pose.orientation.x = (local_transform * transform).getRotation().x();
            bottom_ndt_pose_msg.pose.orientation.y = (local_transform * transform).getRotation().y();
            bottom_ndt_pose_msg.pose.orientation.z = (local_transform * transform).getRotation().z();
            bottom_ndt_pose_msg.pose.orientation.w = (local_transform * transform).getRotation().w();
        }
        else
        {
            bottom_ndt_pose_msg.header.frame_id = "map";
            bottom_ndt_pose_msg.header.stamp = current_scan_time;
            bottom_ndt_pose_msg.pose.position.x = bottom_ndt_pose.x;
            bottom_ndt_pose_msg.pose.position.y = bottom_ndt_pose.y;
            bottom_ndt_pose_msg.pose.position.z = bottom_ndt_pose.z;
            bottom_ndt_pose_msg.pose.orientation.x = ndt_q.x();
            bottom_ndt_pose_msg.pose.orientation.y = ndt_q.y();
            bottom_ndt_pose_msg.pose.orientation.z = ndt_q.z();
            bottom_ndt_pose_msg.pose.orientation.w = ndt_q.w();
        }

        current_q.setRPY(bottom_current_pose.roll, bottom_current_pose.pitch, bottom_current_pose.yaw);

        localizer_q.setRPY(0.0, 0.0, bottom_localizer_pose.yaw);
        if (_use_local_transform == true)
        {
            tf2::Vector3 v(bottom_localizer_pose.x, bottom_localizer_pose.y, bottom_localizer_pose.z);
            tf2::Transform transform(localizer_q, v);
            bottom_localizer_pose_msg.header.frame_id = "map";
            bottom_localizer_pose_msg.header.stamp = current_scan_time;
            bottom_localizer_pose_msg.pose.pose.position.x = (local_transform * transform).getOrigin().getX();
            bottom_localizer_pose_msg.pose.pose.position.y = (local_transform * transform).getOrigin().getY();
            bottom_localizer_pose_msg.pose.pose.position.z = (local_transform * transform).getOrigin().getZ();
            bottom_localizer_pose_msg.pose.pose.orientation.x = (local_transform * transform).getRotation().x();
            bottom_localizer_pose_msg.pose.pose.orientation.y = (local_transform * transform).getRotation().y();
            bottom_localizer_pose_msg.pose.pose.orientation.z = (local_transform * transform).getRotation().z();
            bottom_localizer_pose_msg.pose.pose.orientation.w = (local_transform * transform).getRotation().w();
        }
        else
        {
            bottom_localizer_pose_msg.header.frame_id = "map";
            bottom_localizer_pose_msg.header.stamp = current_scan_time;
            bottom_localizer_pose_msg.pose.pose.position.x = bottom_localizer_pose.x;
            bottom_localizer_pose_msg.pose.pose.position.y = bottom_localizer_pose.y;
            bottom_localizer_pose_msg.pose.pose.position.z = -0.49;
            bottom_localizer_pose_msg.pose.pose.orientation.x = localizer_q.x();
            bottom_localizer_pose_msg.pose.pose.orientation.y = localizer_q.y();
            bottom_localizer_pose_msg.pose.pose.orientation.z = localizer_q.z();
            bottom_localizer_pose_msg.pose.pose.orientation.w = localizer_q.w();

            bottom_localizer_pose_msg.pose.covariance[0] = bottom_cov(0, 0);
            bottom_localizer_pose_msg.pose.covariance[1] = bottom_cov(0, 1);
            bottom_localizer_pose_msg.pose.covariance[2] = bottom_cov(0, 2);
            bottom_localizer_pose_msg.pose.covariance[3] = bottom_cov(0, 3);
            bottom_localizer_pose_msg.pose.covariance[4] = bottom_cov(0, 4);
            bottom_localizer_pose_msg.pose.covariance[5] = bottom_cov(0, 5);
            bottom_localizer_pose_msg.pose.covariance[6] = bottom_cov(1, 0);
            bottom_localizer_pose_msg.pose.covariance[7] = bottom_cov(1, 1);
            bottom_localizer_pose_msg.pose.covariance[8] = bottom_cov(1, 2);
            bottom_localizer_pose_msg.pose.covariance[9] = bottom_cov(1, 3);
            bottom_localizer_pose_msg.pose.covariance[10] = bottom_cov(1, 4);
            bottom_localizer_pose_msg.pose.covariance[11] = bottom_cov(1, 5);
            bottom_localizer_pose_msg.pose.covariance[12] = bottom_cov(2, 0);
            bottom_localizer_pose_msg.pose.covariance[13] = bottom_cov(2, 1);
            bottom_localizer_pose_msg.pose.covariance[14] = bottom_cov(2, 2);
            bottom_localizer_pose_msg.pose.covariance[15] = bottom_cov(2, 3);
            bottom_localizer_pose_msg.pose.covariance[16] = bottom_cov(2, 4);
            bottom_localizer_pose_msg.pose.covariance[17] = bottom_cov(2, 5);
            bottom_localizer_pose_msg.pose.covariance[18] = bottom_cov(3, 0);
            bottom_localizer_pose_msg.pose.covariance[19] = bottom_cov(3, 1);
            bottom_localizer_pose_msg.pose.covariance[20] = bottom_cov(3, 2);
            bottom_localizer_pose_msg.pose.covariance[21] = bottom_cov(3, 3);
            bottom_localizer_pose_msg.pose.covariance[22] = bottom_cov(3, 4);
            bottom_localizer_pose_msg.pose.covariance[23] = bottom_cov(3, 5);
            bottom_localizer_pose_msg.pose.covariance[24] = bottom_cov(4, 0);
            bottom_localizer_pose_msg.pose.covariance[25] = bottom_cov(4, 1);
            bottom_localizer_pose_msg.pose.covariance[26] = bottom_cov(4, 2);
            bottom_localizer_pose_msg.pose.covariance[27] = bottom_cov(4, 3);
            bottom_localizer_pose_msg.pose.covariance[28] = bottom_cov(4, 4);
            bottom_localizer_pose_msg.pose.covariance[29] = bottom_cov(4, 5);
            bottom_localizer_pose_msg.pose.covariance[30] = bottom_cov(5, 0);
            bottom_localizer_pose_msg.pose.covariance[31] = bottom_cov(5, 1);
            bottom_localizer_pose_msg.pose.covariance[32] = bottom_cov(5, 2);
            bottom_localizer_pose_msg.pose.covariance[33] = bottom_cov(5, 3);
            bottom_localizer_pose_msg.pose.covariance[34] = bottom_cov(5, 4);
            bottom_localizer_pose_msg.pose.covariance[35] = bottom_cov(5, 5);
        }

        bottom_predict_pose_pub.publish(bottom_predict_pose_msg);
        health_checker_ptr_->CHECK_RATE("topic_rate_ndt_pose_slow", 8, 5, 1, "topic ndt_pose publish rate slow.");
        bottom_ndt_pose_pub.publish(bottom_ndt_pose_msg);
        bottom_localizer_pose_pub.publish(bottom_localizer_pose_msg);

        // Send TF "base_link" to "map"
        transform.setOrigin(tf2::Vector3(bottom_current_pose.x, bottom_current_pose.y, bottom_current_pose.z));
        transform.setRotation(current_q);
        if (_use_local_transform == true)
        {
            transform = local_transform * transform;
        }
        tf2::Stamped<tf2::Transform> tf(transform, current_scan_time, "map");
        geometry_msgs::TransformStamped tf_msg = tf2::toMsg(tf);
        tf_msg.child_frame_id = _output_tf_frame_id;
        br.sendTransform(tf_msg);

        bottom_matching_end = std::chrono::system_clock::now();
        bottom_exe_time = std::chrono::duration_cast<std::chrono::microseconds>(bottom_matching_end - bottom_matching_start).count() / 1000.0;
        bottom_time_ndt_matching.data = bottom_exe_time;
        health_checker_ptr_->CHECK_MAX_VALUE("time_ndt_matching", bottom_time_ndt_matching.data, 50, 70, 100, "value time_ndt_matching is too high.");
        bottom_time_ndt_matching_pub.publish(bottom_time_ndt_matching);


        // Set values for /ndt_stat
        bottom_ndt_stat_msg.header.stamp = current_scan_time;
        bottom_ndt_stat_msg.exe_time = bottom_time_ndt_matching.data;
        bottom_ndt_stat_msg.iteration = bottom_iteration;
        bottom_ndt_stat_msg.score = bottom_fitness_score;
        bottom_ndt_stat_msg.velocity = bottom_current_velocity;
        bottom_ndt_stat_msg.acceleration = bottom_current_accel;
        bottom_ndt_stat_msg.use_predict_pose = 0;

        bottom_ndt_stat_pub.publish(bottom_ndt_stat_msg);
        /* Compute NDT_Reliability */
        bottom_ndt_reliability.data = Wa * (bottom_exe_time / 100.0) * 100.0 + Wb * (bottom_iteration / 10.0) * 100.0 +
                               Wc * (-std::fabs(2.0 - bottom_trans_probability) / 2.0) * 100.0;
        bottom_ndt_reliability_pub.publish(bottom_ndt_reliability);

        // Write log
        // if (_output_log_data)
        // {
        //     if (!ofs)
        //     {
        //         std::cerr << "Could not open " << filename << "." << std::endl;
        //     }
        //     else
        //     {
        //         ofs << input->header.seq << "," << scan_points_num << "," << step_size << "," << trans_eps << "," << std::fixed
        //             << std::setprecision(5) << current_pose.x << "," << std::fixed << std::setprecision(5) << current_pose.y << ","
        //             << std::fixed << std::setprecision(5) << current_pose.z << "," << current_pose.roll << "," << current_pose.pitch
        //             << "," << current_pose.yaw << "," << predict_pose.x << "," << predict_pose.y << "," << predict_pose.z << ","
        //             << predict_pose.roll << "," << predict_pose.pitch << "," << predict_pose.yaw << ","
        //             << current_pose.x - predict_pose.x << "," << current_pose.y - predict_pose.y << ","
        //             << current_pose.z - predict_pose.z << "," << current_pose.roll - predict_pose.roll << ","
        //             << current_pose.pitch - predict_pose.pitch << "," << current_pose.yaw - predict_pose.yaw << ","
        //             << predict_pose_error << "," << iteration << "," << fitness_score << "," << trans_probability << ","
        //             << ndt_reliability.data << "," << current_velocity << "," << current_velocity_smooth << "," << current_accel
        //             << "," << angular_velocity << "," << time_ndt_matching.data << "," << align_time << "," << getFitnessScore_time
        //             << std::endl;
        //     }
        // }

        std::cout << "-----------------------------------------------------------------" << std::endl;
        // std::cout << "Sequence: " << input->header.seq << std::endl;
        // std::cout << "Timestamp: " << input->header.stamp << std::endl;
        // std::cout << "Frame ID: " << input->header.frame_id << std::endl;
        //    std::cout << "Number of Scan Points: " << scan_ptr->size() << " points." << std::endl;
        std::cout << "Number of Filtered Scan Points of bottom layer: " << scan_points_num << " points." << std::endl;
        std::cout << "NDT of bottom layer has converged: " << bottom_has_converged << std::endl;
        std::cout << "Bottom layer Fitness Score: " << bottom_fitness_score << std::endl;
        std::cout << "Bottom layer Transformation Probability: " << bottom_trans_probability << std::endl;
        std::cout << "Bottom layer Execution Time: " << bottom_exe_time << " ms." << std::endl;
        std::cout << "Number of Iterations of bottom layer: " << bottom_iteration << std::endl;
        std::cout << "Bottom layer NDT Reliability: " << bottom_ndt_reliability.data << std::endl;
        std::cout << "Bottom (x,y,z,roll,pitch,yaw): " << std::endl;
        std::cout << "(" << bottom_current_pose.x << ", " << bottom_current_pose.y << ", " << bottom_current_pose.z << ", " << bottom_current_pose.roll
                  << ", " << bottom_current_pose.pitch << ", " << bottom_current_pose.yaw << ")" << std::endl;
        std::cout << "Transformation Matrix of bottom layer: " << std::endl;
        std::cout << t << std::endl;
        // std::cout << "Align time: " << align_time << std::endl;
        // std::cout << "Get fitness score time: " << getFitnessScore_time << std::endl;
        std::cout << "-----------------------------------------------------------------" << std::endl;

        std::cout << "Covariance of bottom layer: "<< std::endl;
        std::cout << bottom_cov << std::endl;
        std::cout << "-----------------------------------------------------------------" << std::endl;

        // Update previous_***

        bottom_previous_pose.x = bottom_current_pose.x;
        bottom_previous_pose.y = bottom_current_pose.y;
        bottom_previous_pose.z = bottom_current_pose.z;

        bottom_previous_pose.roll = bottom_current_pose.roll;
        bottom_previous_pose.pitch = bottom_current_pose.pitch;
        bottom_previous_pose.yaw = bottom_current_pose.yaw;

        previous_scan_time = current_scan_time;

        bottom_previous_previous_velocity = bottom_previous_velocity;
        bottom_previous_velocity = bottom_current_velocity;
        bottom_previous_velocity_x = bottom_current_velocity_x;
        bottom_previous_velocity_y = bottom_current_velocity_y;
        bottom_previous_velocity_z = bottom_current_velocity_z;
        bottom_previous_accel = bottom_current_accel;

    }
}

static void top_points_callback(const sensor_msgs::PointCloud2::ConstPtr &input)
{
    health_checker_ptr_->CHECK_RATE("topic_rate_filtered_points_slow", 8, 5, 1, "topic filtered_points subscribe rate slow.");
    if (map_loaded == 1 && init_pos_set == 1)
    {
        top_matching_start = std::chrono::system_clock::now();

        static tf2_ros::TransformBroadcaster br;
        tf2::Transform transform;
        tf2::Quaternion predict_q, ndt_q, current_q, localizer_q;

        pcl::PointXYZ p;
        pcl::PointCloud<pcl::PointXYZ> filtered_scan;

        ros::Time current_scan_time = ros::Time().now();
        // ros::Time current_scan_time = input->header.stamp;
        static ros::Time previous_scan_time = current_scan_time;

        pcl::fromROSMsg(*input, filtered_scan);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(filtered_scan));
        int scan_points_num = filtered_scan_ptr->size();
        // Eigen::Matrix<double, 6, 1> deviation;

        Eigen::Matrix4f t(Eigen::Matrix4f::Identity());  // base_link
        Eigen::Matrix4f t2(Eigen::Matrix4f::Identity()); // localizer

        std::chrono::time_point<std::chrono::system_clock> align_start, align_end, getFitnessScore_start,
            getFitnessScore_end;
        static double align_time, getFitnessScore_time = 0.0;

        pthread_mutex_lock(&mutex);

        if (_method_type == MethodType::PCL_GENERIC)
            ndt.setInputSource(filtered_scan_ptr);
        else if (_method_type == MethodType::PCL_ANH)
            anh_ndt.setInputSource(filtered_scan_ptr);

        // Guess the initial gross estimation of the transformation
        double diff_time = (current_scan_time - previous_scan_time).toSec();

        if (_offset == "linear")
        {
            top_offset_x = top_current_velocity_x * diff_time;
            top_offset_y = top_current_velocity_y * diff_time;
            top_offset_z = top_current_velocity_z * diff_time;
            top_offset_yaw = top_angular_velocity * diff_time;
        }
        else if (_offset == "quadratic")
        {
            top_offset_x = (top_current_velocity_x + top_current_accel_x * diff_time) * diff_time;
            top_offset_y = (top_current_velocity_y + top_current_accel_y * diff_time) * diff_time;
            top_offset_z = top_current_velocity_z * diff_time;
            top_offset_yaw = top_angular_velocity * diff_time;
        }
        else if (_offset == "zero")
        {
            top_offset_x = 0.0;
            top_offset_y = 0.0;
            top_offset_z = 0.0;
            top_offset_yaw = 0.0;
        }

        top_predict_pose.x = top_previous_pose.x + top_offset_x;
        top_predict_pose.y = top_previous_pose.y + top_offset_y;
        top_predict_pose.z = -0.49;
        top_predict_pose.roll = 0;
        top_predict_pose.pitch = 0;
        top_predict_pose.yaw = top_previous_pose.yaw + top_offset_yaw;

        pose predict_pose_for_ndt;
        
        predict_pose_for_ndt = top_predict_pose;

        Eigen::Translation3f init_translation(predict_pose_for_ndt.x, predict_pose_for_ndt.y, predict_pose_for_ndt.z);
        Eigen::AngleAxisf init_rotation_x(predict_pose_for_ndt.roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf init_rotation_y(predict_pose_for_ndt.pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf init_rotation_z(predict_pose_for_ndt.yaw, Eigen::Vector3f::UnitZ());
        Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x) * tf_btol;

        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        if (_method_type == MethodType::PCL_GENERIC)
        {
            align_start = std::chrono::system_clock::now();
            ndt.align(*output_cloud, init_guess);
            align_end = std::chrono::system_clock::now();

            top_has_converged = ndt.hasConverged();

            t = ndt.getFinalTransformation();
            top_iteration = ndt.getFinalNumIteration();

            getFitnessScore_start = std::chrono::system_clock::now();
            top_fitness_score = ndt.getFitnessScore();
            getFitnessScore_end = std::chrono::system_clock::now();

            top_trans_probability = ndt.getTransformationProbability();
        }
        else if (_method_type == MethodType::PCL_ANH)
        {
            align_start = std::chrono::system_clock::now();
            anh_ndt.align(init_guess);
            align_end = std::chrono::system_clock::now();

            top_has_converged = anh_ndt.hasConverged();

            t = anh_ndt.getFinalTransformation();
            top_iteration = anh_ndt.getFinalNumIteration();

            getFitnessScore_start = std::chrono::system_clock::now();
            top_fitness_score = anh_ndt.getFitnessScore();
            getFitnessScore_end = std::chrono::system_clock::now();

            top_trans_probability = anh_ndt.getTransformationProbability();

            top_hessian_inv = anh_ndt.getHessian();
            // Calculate covariance of estimated posoe
            top_cov = top_fitness_score * top_hessian_inv;
        }

        align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end - align_start).count() / 1000.0;

        t2 = t * tf_btol.inverse();

        getFitnessScore_time =
            std::chrono::duration_cast<std::chrono::microseconds>(getFitnessScore_end - getFitnessScore_start).count() /
            1000.0;

        pthread_mutex_unlock(&mutex);

        tf2::Matrix3x3 mat_l; // localizer
        mat_l.setValue(static_cast<double>(t(0, 0)), static_cast<double>(t(0, 1)), static_cast<double>(t(0, 2)),
                       static_cast<double>(t(1, 0)), static_cast<double>(t(1, 1)), static_cast<double>(t(1, 2)),
                       static_cast<double>(t(2, 0)), static_cast<double>(t(2, 1)), static_cast<double>(t(2, 2)));

        // Update localizer_pose
        top_localizer_pose.x = t(0, 3);
        top_localizer_pose.y = t(1, 3);
        top_localizer_pose.z = t(2, 3);
        mat_l.getRPY(top_localizer_pose.roll, top_localizer_pose.pitch, top_localizer_pose.yaw, 1);
        // mat_l.getRPY(0.0, 0.0, localizer_pose.yaw, 1);

        tf2::Matrix3x3 mat_b; // base_link
        mat_b.setValue(static_cast<double>(t2(0, 0)), static_cast<double>(t2(0, 1)), static_cast<double>(t2(0, 2)),
                       static_cast<double>(t2(1, 0)), static_cast<double>(t2(1, 1)), static_cast<double>(t2(1, 2)),
                       static_cast<double>(t2(2, 0)), static_cast<double>(t2(2, 1)), static_cast<double>(t2(2, 2)));

        // Update ndt_pose
        top_ndt_pose.x = t2(0, 3);
        top_ndt_pose.y = t2(1, 3);
        top_ndt_pose.z = t2(2, 3);
        mat_b.getRPY(top_ndt_pose.roll, top_ndt_pose.pitch, top_ndt_pose.yaw, 1);

        // Calculate the difference between ndt_pose and predict_pose
        top_predict_pose_error = sqrt((top_ndt_pose.x - predict_pose_for_ndt.x) * (top_ndt_pose.x - predict_pose_for_ndt.x) +
                                  (top_ndt_pose.y - predict_pose_for_ndt.y) * (top_ndt_pose.y - predict_pose_for_ndt.y) +
                                  (top_ndt_pose.z - predict_pose_for_ndt.z) * (top_ndt_pose.z - predict_pose_for_ndt.z));

        if (top_predict_pose_error <= PREDICT_POSE_THRESHOLD)
        {
            top_use_predict_pose = 0;
        }
        else
        {
            top_use_predict_pose = 1;
        }
        top_use_predict_pose = 0;

        if (top_use_predict_pose == 0)
        {
            top_current_pose.x = top_ndt_pose.x;
            top_current_pose.y = top_ndt_pose.y;
            top_current_pose.z = -0.49;
            top_current_pose.roll = 0;
            top_current_pose.pitch = 0;
            top_current_pose.yaw = top_ndt_pose.yaw;
        }
        else
        {
            top_current_pose.x = predict_pose_for_ndt.x;
            top_current_pose.y = predict_pose_for_ndt.y;
            top_current_pose.z = -0.49;
            top_current_pose.roll = 0;
            top_current_pose.pitch = 0;
            top_current_pose.yaw = predict_pose_for_ndt.yaw;
        }

        // Compute the velocity and acceleration
        top_diff_x = top_current_pose.x - top_previous_pose.x;
        top_diff_y = top_current_pose.y - top_previous_pose.y;
        top_diff_z = top_current_pose.z - top_previous_pose.z;
        top_diff_yaw = calcDiffForRadian(top_current_pose.yaw, top_previous_pose.yaw);
        top_diff = sqrt(top_diff_x * top_diff_x + top_diff_y * top_diff_y + top_diff_z * top_diff_z);

        const pose trans_current_pose = convertPoseIntoRelativeCoordinate(top_current_pose, top_previous_pose);

        top_current_velocity = (diff_time > 0) ? (top_diff / diff_time) : 0;
        top_current_velocity = (trans_current_pose.x >= 0) ? top_current_velocity : -top_current_velocity;
        top_current_velocity_x = (diff_time > 0) ? (top_diff_x / diff_time) : 0;
        top_current_velocity_y = (diff_time > 0) ? (top_diff_y / diff_time) : 0;
        top_current_velocity_z = (diff_time > 0) ? (top_diff_z / diff_time) : 0;
        top_angular_velocity = (diff_time > 0) ? (top_diff_yaw / diff_time) : 0;

        top_current_accel = (diff_time > 0) ? ((top_current_velocity - top_previous_velocity) / diff_time) : 0;
        top_current_accel_x = (diff_time > 0) ? ((top_current_velocity_x - top_previous_velocity_x) / diff_time) : 0;
        top_current_accel_y = (diff_time > 0) ? ((top_current_velocity_y - top_previous_velocity_y) / diff_time) : 0;
        top_current_accel_z = (diff_time > 0) ? ((top_current_velocity_z - top_previous_velocity_z) / diff_time) : 0;

        // Set values for publishing pose
        predict_q.setRPY(top_predict_pose.roll, top_predict_pose.pitch, top_predict_pose.yaw);
        if (_use_local_transform == true)
        {
            tf2::Vector3 v(top_predict_pose.x, top_predict_pose.y, top_predict_pose.z);
            tf2::Transform transform(predict_q, v);
            top_predict_pose_msg.header.frame_id = "map";
            top_predict_pose_msg.header.stamp = current_scan_time;
            top_predict_pose_msg.pose.position.x = (local_transform * transform).getOrigin().getX();
            top_predict_pose_msg.pose.position.y = (local_transform * transform).getOrigin().getY();
            top_predict_pose_msg.pose.position.z = (local_transform * transform).getOrigin().getZ();
            top_predict_pose_msg.pose.orientation.x = (local_transform * transform).getRotation().x();
            top_predict_pose_msg.pose.orientation.y = (local_transform * transform).getRotation().y();
            top_predict_pose_msg.pose.orientation.z = (local_transform * transform).getRotation().z();
            top_predict_pose_msg.pose.orientation.w = (local_transform * transform).getRotation().w();
        }
        else
        {
            top_predict_pose_msg.header.frame_id = "map";
            top_predict_pose_msg.header.stamp = current_scan_time;
            top_predict_pose_msg.pose.position.x = top_predict_pose.x;
            top_predict_pose_msg.pose.position.y = top_predict_pose.y;
            top_predict_pose_msg.pose.position.z = top_predict_pose.z;
            top_predict_pose_msg.pose.orientation.x = predict_q.x();
            top_predict_pose_msg.pose.orientation.y = predict_q.y();
            top_predict_pose_msg.pose.orientation.z = predict_q.z();
            top_predict_pose_msg.pose.orientation.w = predict_q.w();
        }

        ndt_q.setRPY(top_ndt_pose.roll, top_ndt_pose.pitch, top_ndt_pose.yaw);
        if (_use_local_transform == true)
        {
            tf2::Vector3 v(top_ndt_pose.x, top_ndt_pose.y, top_ndt_pose.z);
            tf2::Transform transform(ndt_q, v);
            top_ndt_pose_msg.header.frame_id = "map";
            top_ndt_pose_msg.header.stamp = current_scan_time;
            top_ndt_pose_msg.pose.position.x = (local_transform * transform).getOrigin().getX();
            top_ndt_pose_msg.pose.position.y = (local_transform * transform).getOrigin().getY();
            top_ndt_pose_msg.pose.position.z = (local_transform * transform).getOrigin().getZ();
            top_ndt_pose_msg.pose.orientation.x = (local_transform * transform).getRotation().x();
            top_ndt_pose_msg.pose.orientation.y = (local_transform * transform).getRotation().y();
            top_ndt_pose_msg.pose.orientation.z = (local_transform * transform).getRotation().z();
            top_ndt_pose_msg.pose.orientation.w = (local_transform * transform).getRotation().w();
        }
        else
        {
            top_ndt_pose_msg.header.frame_id = "map";
            top_ndt_pose_msg.header.stamp = current_scan_time;
            top_ndt_pose_msg.pose.position.x = top_ndt_pose.x;
            top_ndt_pose_msg.pose.position.y = top_ndt_pose.y;
            top_ndt_pose_msg.pose.position.z = top_ndt_pose.z;
            top_ndt_pose_msg.pose.orientation.x = ndt_q.x();
            top_ndt_pose_msg.pose.orientation.y = ndt_q.y();
            top_ndt_pose_msg.pose.orientation.z = ndt_q.z();
            top_ndt_pose_msg.pose.orientation.w = ndt_q.w();
        }

        current_q.setRPY(top_current_pose.roll, top_current_pose.pitch, top_current_pose.yaw);

        localizer_q.setRPY(0.0, 0.0, top_localizer_pose.yaw);
        if (_use_local_transform == true)
        {
            tf2::Vector3 v(top_localizer_pose.x, top_localizer_pose.y, top_localizer_pose.z);
            tf2::Transform transform(localizer_q, v);
            top_localizer_pose_msg.header.frame_id = "map";
            top_localizer_pose_msg.header.stamp = current_scan_time;
            top_localizer_pose_msg.pose.pose.position.x = (local_transform * transform).getOrigin().getX();
            top_localizer_pose_msg.pose.pose.position.y = (local_transform * transform).getOrigin().getY();
            top_localizer_pose_msg.pose.pose.position.z = (local_transform * transform).getOrigin().getZ();
            top_localizer_pose_msg.pose.pose.orientation.x = (local_transform * transform).getRotation().x();
            top_localizer_pose_msg.pose.pose.orientation.y = (local_transform * transform).getRotation().y();
            top_localizer_pose_msg.pose.pose.orientation.z = (local_transform * transform).getRotation().z();
            top_localizer_pose_msg.pose.pose.orientation.w = (local_transform * transform).getRotation().w();
        }
        else
        {
            top_localizer_pose_msg.header.frame_id = "map";
            top_localizer_pose_msg.header.stamp = current_scan_time;
            top_localizer_pose_msg.pose.pose.position.x = top_localizer_pose.x;
            top_localizer_pose_msg.pose.pose.position.y = top_localizer_pose.y;
            top_localizer_pose_msg.pose.pose.position.z = -0.49;
            top_localizer_pose_msg.pose.pose.orientation.x = localizer_q.x();
            top_localizer_pose_msg.pose.pose.orientation.y = localizer_q.y();
            top_localizer_pose_msg.pose.pose.orientation.z = localizer_q.z();
            top_localizer_pose_msg.pose.pose.orientation.w = localizer_q.w();

            top_localizer_pose_msg.pose.covariance[0] = top_cov(0, 0);
            top_localizer_pose_msg.pose.covariance[1] = top_cov(0, 1);
            top_localizer_pose_msg.pose.covariance[2] = top_cov(0, 2);
            top_localizer_pose_msg.pose.covariance[3] = top_cov(0, 3);
            top_localizer_pose_msg.pose.covariance[4] = top_cov(0, 4);
            top_localizer_pose_msg.pose.covariance[5] = top_cov(0, 5);
            top_localizer_pose_msg.pose.covariance[6] = top_cov(1, 0);
            top_localizer_pose_msg.pose.covariance[7] = top_cov(1, 1);
            top_localizer_pose_msg.pose.covariance[8] = top_cov(1, 2);
            top_localizer_pose_msg.pose.covariance[9] = top_cov(1, 3);
            top_localizer_pose_msg.pose.covariance[10] = top_cov(1, 4);
            top_localizer_pose_msg.pose.covariance[11] = top_cov(1, 5);
            top_localizer_pose_msg.pose.covariance[12] = top_cov(2, 0);
            top_localizer_pose_msg.pose.covariance[13] = top_cov(2, 1);
            top_localizer_pose_msg.pose.covariance[14] = top_cov(2, 2);
            top_localizer_pose_msg.pose.covariance[15] = top_cov(2, 3);
            top_localizer_pose_msg.pose.covariance[16] = top_cov(2, 4);
            top_localizer_pose_msg.pose.covariance[17] = top_cov(2, 5);
            top_localizer_pose_msg.pose.covariance[18] = top_cov(3, 0);
            top_localizer_pose_msg.pose.covariance[19] = top_cov(3, 1);
            top_localizer_pose_msg.pose.covariance[20] = top_cov(3, 2);
            top_localizer_pose_msg.pose.covariance[21] = top_cov(3, 3);
            top_localizer_pose_msg.pose.covariance[22] = top_cov(3, 4);
            top_localizer_pose_msg.pose.covariance[23] = top_cov(3, 5);
            top_localizer_pose_msg.pose.covariance[24] = top_cov(4, 0);
            top_localizer_pose_msg.pose.covariance[25] = top_cov(4, 1);
            top_localizer_pose_msg.pose.covariance[26] = top_cov(4, 2);
            top_localizer_pose_msg.pose.covariance[27] = top_cov(4, 3);
            top_localizer_pose_msg.pose.covariance[28] = top_cov(4, 4);
            top_localizer_pose_msg.pose.covariance[29] = top_cov(4, 5);
            top_localizer_pose_msg.pose.covariance[30] = top_cov(5, 0);
            top_localizer_pose_msg.pose.covariance[31] = top_cov(5, 1);
            top_localizer_pose_msg.pose.covariance[32] = top_cov(5, 2);
            top_localizer_pose_msg.pose.covariance[33] = top_cov(5, 3);
            top_localizer_pose_msg.pose.covariance[34] = top_cov(5, 4);
            top_localizer_pose_msg.pose.covariance[35] = top_cov(5, 5);
        }

        top_predict_pose_pub.publish(top_predict_pose_msg);
        health_checker_ptr_->CHECK_RATE("topic_rate_ndt_pose_slow", 8, 5, 1, "topic ndt_pose publish rate slow.");
        top_ndt_pose_pub.publish(top_ndt_pose_msg);
        top_localizer_pose_pub.publish(top_localizer_pose_msg);

        // Send TF "base_link" to "map"
        transform.setOrigin(tf2::Vector3(top_current_pose.x, top_current_pose.y, top_current_pose.z));
        transform.setRotation(current_q);
        if (_use_local_transform == true)
        {
            transform = local_transform * transform;
        }
        tf2::Stamped<tf2::Transform> tf(transform, current_scan_time, "map");
        geometry_msgs::TransformStamped tf_msg = tf2::toMsg(tf);
        tf_msg.child_frame_id = _output_tf_frame_id;
        br.sendTransform(tf_msg);

        top_matching_end = std::chrono::system_clock::now();
        top_exe_time = std::chrono::duration_cast<std::chrono::microseconds>(top_matching_end - top_matching_start).count() / 1000.0;
        top_time_ndt_matching.data = top_exe_time;
        health_checker_ptr_->CHECK_MAX_VALUE("time_ndt_matching", top_time_ndt_matching.data, 50, 70, 100, "value time_ndt_matching is too high.");
        top_time_ndt_matching_pub.publish(top_time_ndt_matching);

        
        // Set values for /ndt_stat
        top_ndt_stat_msg.header.stamp = current_scan_time;
        top_ndt_stat_msg.exe_time = top_time_ndt_matching.data;
        top_ndt_stat_msg.iteration = top_iteration;
        top_ndt_stat_msg.score = top_fitness_score;
        top_ndt_stat_msg.velocity = top_current_velocity;
        top_ndt_stat_msg.acceleration = top_current_accel;
        top_ndt_stat_msg.use_predict_pose = 0;

        top_ndt_stat_pub.publish(top_ndt_stat_msg);
        /* Compute NDT_Reliability */
        top_ndt_reliability.data = Wa * (top_exe_time / 100.0) * 100.0 + Wb * (top_iteration / 10.0) * 100.0 +
                               Wc * (-std::fabs(2.0 - top_trans_probability) / 2.0) * 100.0;
        top_ndt_reliability_pub.publish(top_ndt_reliability);


        // Write log
        // if (_output_log_data)
        // {
        //     if (!ofs)
        //     {
        //         std::cerr << "Could not open " << filename << "." << std::endl;
        //     }
        //     else
        //     {
        //         ofs << input->header.seq << "," << scan_points_num << "," << step_size << "," << trans_eps << "," << std::fixed
        //             << std::setprecision(5) << current_pose.x << "," << std::fixed << std::setprecision(5) << current_pose.y << ","
        //             << std::fixed << std::setprecision(5) << current_pose.z << "," << current_pose.roll << "," << current_pose.pitch
        //             << "," << current_pose.yaw << "," << predict_pose.x << "," << predict_pose.y << "," << predict_pose.z << ","
        //             << predict_pose.roll << "," << predict_pose.pitch << "," << predict_pose.yaw << ","
        //             << current_pose.x - predict_pose.x << "," << current_pose.y - predict_pose.y << ","
        //             << current_pose.z - predict_pose.z << "," << current_pose.roll - predict_pose.roll << ","
        //             << current_pose.pitch - predict_pose.pitch << "," << current_pose.yaw - predict_pose.yaw << ","
        //             << predict_pose_error << "," << iteration << "," << fitness_score << "," << trans_probability << ","
        //             << ndt_reliability.data << "," << current_velocity << "," << current_velocity_smooth << "," << current_accel
        //             << "," << angular_velocity << "," << time_ndt_matching.data << "," << align_time << "," << getFitnessScore_time
        //             << std::endl;
        //     }
        // }

        std::cout << "-----------------------------------------------------------------" << std::endl;
        // std::cout << "Sequence: " << input->header.seq << std::endl;
        // std::cout << "Timestamp: " << input->header.stamp << std::endl;
        // std::cout << "Frame ID: " << input->header.frame_id << std::endl;
        //    std::cout << "Number of Scan Points: " << scan_ptr->size() << " points." << std::endl;
        std::cout << "Number of Filtered Scan Points of Top layer: " << scan_points_num << " points." << std::endl;
        std::cout << "NDT of Top layer has converged: " << top_has_converged << std::endl;
        std::cout << "Top layer Fitness Score: " << top_fitness_score << std::endl;
        std::cout << "Transformation Probability of top layer: " << top_trans_probability << std::endl;
        std::cout << "Execution Time of top layer: " << top_exe_time << " ms." << std::endl;
        std::cout << "Number of Iterations of top layer: " << top_iteration << std::endl;
        std::cout << "NDT Reliability of Top layer: " << top_ndt_reliability.data << std::endl;
        std::cout << "Top (x,y,z,roll,pitch,yaw): " << std::endl;
        std::cout << "(" << top_current_pose.x << ", " << top_current_pose.y << ", " << top_current_pose.z << ", " << top_current_pose.roll
                  << ", " << top_current_pose.pitch << ", " << top_current_pose.yaw << ")" << std::endl;
        std::cout << "Transformation Matrix of Top layer: " << std::endl;
        std::cout << t << std::endl;
        // std::cout << "Align time: " << align_time << std::endl;
        // std::cout << "Get fitness score time: " << getFitnessScore_time << std::endl;
        std::cout << "-----------------------------------------------------------------" << std::endl;

        std::cout << "Covariance of top layer: "<< std::endl;
        std::cout << top_cov << std::endl;
        std::cout << "-----------------------------------------------------------------" << std::endl;

        // Update previous_***

        top_previous_pose.x = top_current_pose.x;
        top_previous_pose.y = top_current_pose.y;
        top_previous_pose.z = top_current_pose.z;

        top_previous_pose.roll = top_current_pose.roll;
        top_previous_pose.pitch = top_current_pose.pitch;
        top_previous_pose.yaw = top_current_pose.yaw;

        previous_scan_time = current_scan_time;

        top_previous_previous_velocity = top_previous_velocity;
        top_previous_velocity = top_current_velocity;
        top_previous_velocity_x = top_current_velocity_x;
        top_previous_velocity_y = top_current_velocity_y;
        top_previous_velocity_z = top_current_velocity_z;
        top_previous_accel = top_current_accel;

    }
}

static void feedback_callback(const geometry_msgs::PoseStamped::ConstPtr &input)
{

    bottom_previous_pose.x = input->pose.position.x;
    bottom_previous_pose.y = input->pose.position.y;

    top_previous_pose.x = input->pose.position.x;
    top_previous_pose.y = input->pose.position.y;
    
}
void *thread_func(void *args)
{
    ros::NodeHandle nh_map;
    ros::CallbackQueue map_callback_queue;
    nh_map.setCallbackQueue(&map_callback_queue);

    ros::Subscriber map_sub = nh_map.subscribe(_map_topic, 10, map_callback);
    ros::Rate ros_rate(10);
    while (nh_map.ok())
    {
        map_callback_queue.callAvailable(ros::WallDuration());
        ros_rate.sleep();
    }

    return nullptr;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ndt_matching");
    pthread_mutex_init(&mutex, NULL);

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    health_checker_ptr_ = std::make_shared<autoware_health_checker::HealthChecker>(nh, private_nh);
    health_checker_ptr_->ENABLE();
    health_checker_ptr_->NODE_ACTIVATE();

    // Set log file name.
    private_nh.getParam("output_log_data", _output_log_data);
    if (_output_log_data)
    {
        char buffer[80];
        std::time_t now = std::time(NULL);
        std::tm *pnow = std::localtime(&now);
        std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);
        std::string directory_name = "/tmp/Autoware/log/ndt_matching";
        filename = directory_name + "/" + std::string(buffer) + ".csv";
        boost::filesystem::create_directories(boost::filesystem::path(directory_name));
        ofs.open(filename.c_str(), std::ios::app);
    }

    // Geting parameters
    int method_type_tmp = 0;
    private_nh.getParam("method_type", method_type_tmp);
    _method_type = static_cast<MethodType>(method_type_tmp);
    private_nh.getParam("use_gnss", _use_gnss);
    private_nh.getParam("queue_size", _queue_size);
    private_nh.getParam("offset", _offset);
    private_nh.getParam("get_height", _get_height);
    private_nh.getParam("use_local_transform", _use_local_transform);
    private_nh.getParam("use_imu", _use_imu);
    private_nh.getParam("use_odom", _use_odom);
    private_nh.getParam("imu_upside_down", _imu_upside_down);
    private_nh.getParam("imu_topic", _imu_topic);
    private_nh.getParam("output_tf_frame_id", _output_tf_frame_id);
    private_nh.getParam("map_param", _map_topic);
    std::string lidar_frame;
    nh.param("localizer", lidar_frame, std::string("chassis_link"));
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    geometry_msgs::TransformStamped tf_baselink2primarylidar;
    bool received_tf = true;

    // 1. Try getting base_link -> lidar TF from TF tree
    try
    {
        tf_baselink2primarylidar =
            tf_buffer.lookupTransform("VLP16", lidar_frame, ros::Time().now(), ros::Duration(3.0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Query base_link to primary lidar frame through TF tree failed: %s", ex.what());
        received_tf = false;
    }

    // 2. Try getting base_link -> lidar TF from tf_baselink2primarylidar param
    if (!received_tf)
    {
        std::vector<double> bl2pl_vec;
        if (nh.getParam("tf_baselink2primarylidar", bl2pl_vec) && bl2pl_vec.size() == 6)
        {
            tf2::Vector3 tf_trans(bl2pl_vec[0], bl2pl_vec[1], bl2pl_vec[2]);
            tf2::Quaternion tf_quat;
            tf_quat.setRPY(bl2pl_vec[5], bl2pl_vec[4], bl2pl_vec[3]);
            tf_baselink2primarylidar.transform.translation = tf2::toMsg(tf_trans);
            tf_baselink2primarylidar.transform.rotation = tf2::toMsg(tf_quat);

            received_tf = true;
        }
        else
        {
            ROS_WARN("Query base_link to primary lidar frame through tf_baselink2primarylidar param failed");
        }
    }

    // 3. Try getting base_link -> lidar TF from tf_* params
    if (!received_tf)
    {
        float tf_x, tf_y, tf_z, tf_roll, tf_pitch, tf_yaw;
        if (nh.getParam("tf_x", tf_x) &&
            nh.getParam("tf_y", tf_y) &&
            nh.getParam("tf_z", tf_z) &&
            nh.getParam("tf_roll", tf_roll) &&
            nh.getParam("tf_pitch", tf_pitch) &&
            nh.getParam("tf_yaw", tf_yaw))
        {
            tf2::Vector3 tf_trans(tf_x, tf_y, tf_z);
            tf2::Quaternion tf_quat;
            tf_quat.setRPY(tf_roll, tf_pitch, tf_yaw);
            tf_baselink2primarylidar.transform.translation = tf2::toMsg(tf_trans);
            tf_baselink2primarylidar.transform.rotation = tf2::toMsg(tf_quat);

            received_tf = true;
        }
        else
        {
            ROS_WARN("Query base_link to primary lidar frame through tf_* params failed");
        }
    }

    if (received_tf)
    {
        ROS_INFO("base_link to primary lidar transform queried successfully");
    }
    else
    {
        ROS_ERROR("Failed to query base_link to primary lidar transform");
        return 1;
    }

    tf_btol = tf2::transformToEigen(tf_baselink2primarylidar).matrix().cast<float>();

    // std::cout << "-----------------------------------------------------------------" << std::endl;
    // std::cout << "Log file: " << filename << std::endl;
    // std::cout << "method_type: " << static_cast<int>(_method_type) << std::endl;
    // std::cout << "use_gnss: " << _use_gnss << std::endl;
    // std::cout << "queue_size: " << _queue_size << std::endl;
    // std::cout << "offset: " << _offset << std::endl;
    // std::cout << "get_height: " << _get_height << std::endl;
    // std::cout << "use_local_transform: " << _use_local_transform << std::endl;
    // std::cout << "use_odom: " << _use_odom << std::endl;
    // std::cout << "use_imu: " << _use_imu << std::endl;
    // std::cout << "imu_upside_down: " << _imu_upside_down << std::endl;
    // std::cout << "imu_topic: " << _imu_topic << std::endl;
    // std::cout << "localizer: " << lidar_frame << std::endl;
    // std::cout << "gnss_reinit_fitness: " << _gnss_reinit_fitness << std::endl;
    // std::cout << "tf_baselink2primarylidar: \n"
    //           << tf_btol << std::endl;
    // std::cout << "-----------------------------------------------------------------" << std::endl;


    // Updated in initialpose_callback or gnss_callback
    initial_pose.x = 0.0;
    initial_pose.y = 0.0;
    initial_pose.z = 0.0;
    initial_pose.roll = 0.0;
    initial_pose.pitch = 0.0;
    initial_pose.yaw = 0.0;

    // Publishers
    bottom_predict_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/bottom_predict_pose", 10);
    bottom_ndt_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/bottom_ndt_pose", 10);
    top_predict_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/top_predict_pose", 10);
    top_ndt_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/top_ndt_pose", 10);
    // current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);
    bottom_localizer_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/bottom_localizer_pose", 10);
    top_localizer_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/top_localizer_pose", 10);
    
    bottom_time_ndt_matching_pub = nh.advertise<std_msgs::Float32>("/bottom_time_ndt_matching", 10);
    bottom_ndt_stat_pub = nh.advertise<autoware_msgs::NDTStat>("/bottom_ndt_stat", 10);
    bottom_ndt_reliability_pub = nh.advertise<std_msgs::Float32>("/bottom_ndt_reliability", 10);
    top_time_ndt_matching_pub = nh.advertise<std_msgs::Float32>("/top_time_ndt_matching", 10);
    top_ndt_stat_pub = nh.advertise<autoware_msgs::NDTStat>("/top_ndt_stat", 10);
    top_ndt_reliability_pub = nh.advertise<std_msgs::Float32>("/top_ndt_reliability", 10);



    //  ros::Subscriber map_sub = nh.subscribe("points_map", 1, map_callback);
    ros::Subscriber initialpose_sub = nh.subscribe("initialpose", 10, initialpose_callback);
    ros::Subscriber bottom_points_sub = nh.subscribe("filtered_bottom_points", _queue_size, bottom_points_callback);
    ros::Subscriber top_points_sub = nh.subscribe("filtered_top_points", _queue_size, top_points_callback);

    ros::Subscriber feedback_sub = nh.subscribe("fusion_position", 1, feedback_callback);

    pthread_t thread;
    pthread_create(&thread, NULL, thread_func, NULL);

    ros::spin();

    return 0;
}
