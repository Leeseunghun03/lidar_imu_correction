#include "lidar_imu_correction_node.hpp"

LidarImuCorrection::LidarImuCorrection(ros::NodeHandle &nh)
{
    // Load IMU and LiDAR topic names from config file
    std::string imu_topic, lidar_topic;
    nh.getParam("/lidar_imu_correction/imu_topic", imu_topic);
    nh.getParam("/lidar_imu_correction/lidar_topic", lidar_topic);

    imu_sub = nh.subscribe(imu_topic, 1000, &LidarImuCorrection::imuCallback, this);
    lidar_sub = nh.subscribe(lidar_topic, 1000, &LidarImuCorrection::lidarCallback, this);
    lidar_pub = nh.advertise<PointCloudXYZI>("corrected_lidar", 1);

    current_pitch = 0.0;
    current_roll = 0.0;
    last_timestamp_imu = 0.0;
    last_timestamp_lidar = 0.0;

    loadParameters(nh);
}

void LidarImuCorrection::loadParameters(ros::NodeHandle &nh)
{
    // Load rotation and translation matrices between IMU and LiDAR from ROS parameter server
    std::vector<double> rotation_matrix;
    nh.getParam("/lidar_imu_correction/rotation_matrix", rotation_matrix);
    if (rotation_matrix.size() == 9)
    {
        rotation_imu_to_lidar << rotation_matrix[0], rotation_matrix[1], rotation_matrix[2],
            rotation_matrix[3], rotation_matrix[4], rotation_matrix[5],
            rotation_matrix[6], rotation_matrix[7], rotation_matrix[8];
    }
    else
    {
        ROS_ERROR("Invalid rotation_matrix size, expected 9 elements.");
    }

    std::vector<double> translation_vector;
    nh.getParam("/lidar_imu_correction/translation_vector", translation_vector);
    if (translation_vector.size() == 3)
    {
        translation_imu_to_lidar << translation_vector[0], translation_vector[1], translation_vector[2];
    }
    else
    {
        ROS_ERROR("Invalid translation_vector size, expected 3 elements.");
    }
}

void LidarImuCorrection::imuCallback(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
    msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec());

    double timestamp = msg->header.stamp.toSec();

    std::lock_guard<std::mutex> lock(mtx_buffer);

    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("IMU loop back, clear buffer");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;
    imu_buffer.push_back(msg);
    imu_buff = msg;
}

void LidarImuCorrection::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg_in)
{
    sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2(*msg_in));
    msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec());

    double timestamp = msg->header.stamp.toSec();

    std::lock_guard<std::mutex> lock(mtx_buffer);

    if (timestamp < last_timestamp_lidar)
    {
        ROS_WARN("LiDAR loop back, clear buffer");
        lidar_buffer.clear();
    }

    last_timestamp_lidar = timestamp;
    lidar_buffer.push_back(msg);
    lidar_buff = msg;
}

bool LidarImuCorrection::sync_packages(SensorGroup &meas)
{
    if (imu_buffer.empty() || lidar_buffer.empty())
    {
        return false;
    }

    sync_time = ros::Time::now().toSec();

    double lidar_time = lidar_buffer.front()->header.stamp.toSec();
    meas.lidar.clear();
    while ((!lidar_buffer.empty()) && (lidar_time < sync_time))
    {
        lidar_time = lidar_buffer.front()->header.stamp.toSec();
        if (lidar_time > sync_time)
            break;
        meas.lidar.push_back(lidar_buffer.front());
        lidar_buffer.pop_front();
    }

    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < sync_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if (imu_time > sync_time)
            break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    return true;
}

void LidarImuCorrection::process_synced_data(const sensor_msgs::Imu::Ptr &imu_msg, const sensor_msgs::PointCloud2::Ptr &cloud_msg)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(imu_msg->orientation, quat);

    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // Store the roll and pitch
    current_roll = roll;
    current_pitch = pitch;

    double roll_deg = roll * 180.0 / M_PI; 
    double pitch_deg = pitch * 180.0 / M_PI;
    ROS_INFO("Current Roll: %.2f degrees, Current Pitch: %.2f degrees", roll_deg, pitch_deg);

    PointCloudXYZI::Ptr cloud(new PointCloudXYZI);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    Eigen::Matrix3f pitch_rotation;
    pitch_rotation = Eigen::AngleAxisf(current_pitch, Eigen::Vector3f::UnitY());

    Eigen::Matrix3f roll_rotation;
    roll_rotation = Eigen::AngleAxisf(current_roll, Eigen::Vector3f::UnitX());

    Eigen::Matrix3f total_rotation = roll_rotation * pitch_rotation;

    total_rotation = rotation_imu_to_lidar * total_rotation;

    for (auto &point : cloud->points)
    {
        Eigen::Vector3f point_vector(point.x, point.y, point.z);
        Eigen::Vector3f transformed_point = total_rotation * point_vector;

        point.x = transformed_point.x();
        point.y = transformed_point.y();
        point.z = transformed_point.z();
    }

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud, output_msg);
    output_msg.header = cloud_msg->header;

    lidar_pub.publish(output_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_imu_correction_node");
    ros::NodeHandle nh;

    LidarImuCorrection lidar_imu_correction(nh);

    SensorGroup sensor_data;

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        if (lidar_imu_correction.sync_packages(sensor_data))
        {
            // Process the synchronized data
            lidar_imu_correction.process_synced_data(sensor_data.imu.back(), sensor_data.lidar.back());
            // ROS_INFO("IMU and LiDAR data synchronized and processed.");
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}