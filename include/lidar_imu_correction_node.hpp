#ifndef LIDAR_IMU_CORRECTION_HPP
#define LIDAR_IMU_CORRECTION_HPP

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <deque>
#include <mutex>

// Define types for convenience
typedef pcl::PointCloud<pcl::PointXYZI> PointCloudXYZI;

struct SensorGroup {
    std::deque<sensor_msgs::Imu::Ptr> imu;
    std::deque<sensor_msgs::PointCloud2::Ptr> lidar;
};

class LidarImuCorrection {
public:
    LidarImuCorrection(ros::NodeHandle& nh);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg_in);
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg_in);
    bool sync_packages(SensorGroup &meas);
    void process_synced_data(const sensor_msgs::Imu::Ptr& imu_msg, const sensor_msgs::PointCloud2::Ptr& cloud_msg);

private:
    ros::Subscriber imu_sub;
    ros::Subscriber lidar_sub;
    ros::Publisher lidar_pub;

    std::deque<sensor_msgs::Imu::Ptr> imu_buffer;
    std::deque<sensor_msgs::PointCloud2::Ptr> lidar_buffer;
    sensor_msgs::Imu::Ptr imu_buff;
    sensor_msgs::PointCloud2::Ptr lidar_buff;

    double current_pitch;
    double current_roll;
    double last_timestamp_imu;
    double last_timestamp_lidar;
    double sync_time;

    Eigen::Matrix3f rotation_imu_to_lidar;
    Eigen::Vector3f translation_imu_to_lidar;

    std::mutex mtx_buffer; // mutex for protecting the buffer

    void loadParameters(ros::NodeHandle& nh);
};

#endif // LIDAR_IMU_CORRECTION_HPP