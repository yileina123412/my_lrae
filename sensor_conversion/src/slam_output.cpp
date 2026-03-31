//
// Created by hjl on 2021/9/18.
// Modified by Qingchen Bi on 2022/11/05
//

/**
 * 同步激光雷达点云和里程计数据，将它们统一转换到以起始点为原点的全局坐标系中，并拼接生成全局点云地图。
 *
 * 处理架构：
 * - 高频同步回调：完成位姿归一化/TF 广播/里程计发布，并缓存最新同步帧（点云 + 位姿）。
 * - 低频 Timer：消费一帧缓存数据，执行下采样、坐标变换与全局点云累加，避免在高频回调中进行重计算。
 */

#include "slam_simulation/slam_output.h"

#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>

SlamOutput::SlamOutput(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) : nh_(nh), nh_private_(nh_private),
                                                                                       rate(100), vec_length(2.0),
                                                                                       is_get_first(false) {
    const std::string &ns = ros::this_node::getName();
    frame_id = "map";
    if (!ros::param::get(ns + "/frame_id", frame_id)) {
        ROS_WARN("No frame_id specified. Looking for %s. Default is 'map'.",
                 (ns + "/frame_id").c_str());
    }

    child_frame_id = "sensor";
    if (!ros::param::get(ns + "/child_frame_id", child_frame_id)) {
        ROS_WARN("No child_frame_id specified. Looking for %s. Default is 'sensor'.",
                 (ns + "/child_frame_id").c_str());
    }

    down_voxel_size = 0.1;
    if (!ros::param::get(ns + "/down_voxel_size", down_voxel_size)) {
        ROS_WARN("No down_voxel_size specified. Looking for %s. Default is 'down_voxel_size'.",
                 (ns + "/down_voxel_size").c_str());
    }

    T_B_W = tf::Transform::getIdentity();

    downSizeFilter.setLeafSize(down_voxel_size, down_voxel_size, down_voxel_size);
    exploredAreaDwzFilter.setLeafSize(exploredAreaVoxelSize, exploredAreaVoxelSize, exploredAreaVoxelSize);
    odom_pub = nh_.advertise<nav_msgs::Odometry>("odometry_init", 1);
    reg_pub = nh_.advertise<sensor_msgs::PointCloud2>("registered_scan", 1);
    dwz_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("dwz_scan_cloud", 1);

    // explored_volume_pub = nh_.advertise<std_msgs::Float32>("exploredVolume", 1);
    // 使用message_filters多元同步数据
    local_cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "point_cloud", 1));
    local_odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "odometry", 100));
    sync_local_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyLocalCloudOdom>(
            SyncPolicyLocalCloudOdom(100), *local_cloud_sub_, *local_odom_sub_));
    sync_local_cloud_odom_->registerCallback(boost::bind(&SlamOutput::pointCloudOdomCallback, this, _1, _2));

    execution_timer_ = nh_.createTimer(ros::Duration(0.2), &SlamOutput::execute, this); 
}
// 位姿处理和点云处理的回调函数，接收同步的点云和里程计数据，并将它们转换到全局坐标系中，同时发布里程计信息和注册后的点云数据。
void SlamOutput::pointCloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr &scanIn,
                                        const nav_msgs::OdometryConstPtr &input) {

    tf::Quaternion quaternion(input->pose.pose.orientation.x, input->pose.pose.orientation.y,
                              input->pose.pose.orientation.z, input->pose.pose.orientation.w);
    tf::Vector3 vector3(input->pose.pose.position.x, input->pose.pose.position.y, input->pose.pose.position.z);
    tf::Transform T_W_Bi(quaternion, vector3);
    if (!is_get_first) {
        T_B_W = T_W_Bi.inverse();
        is_get_first = true;
    }
    tf::StampedTransform ST_B_Bi = tf::StampedTransform(T_B_W * T_W_Bi, scanIn->header.stamp, frame_id,
                                                        child_frame_id); 

    broadcaster.sendTransform(ST_B_Bi); 

    nav_msgs::Odometry odom_msg;
    odom_msg.child_frame_id = child_frame_id;
    odom_msg.header.frame_id = frame_id;
    odom_msg.header.stamp = scanIn->header.stamp; 
    odom_msg.header.seq = input->header.seq;
    odom_msg.pose.pose.orientation.x = ST_B_Bi.getRotation().getX();
    odom_msg.pose.pose.orientation.y = ST_B_Bi.getRotation().getY();
    odom_msg.pose.pose.orientation.z = ST_B_Bi.getRotation().getZ();
    odom_msg.pose.pose.orientation.w = ST_B_Bi.getRotation().getW();
    odom_msg.pose.pose.position.x = ST_B_Bi.getOrigin().getX();
    odom_msg.pose.pose.position.y = ST_B_Bi.getOrigin().getY();
    odom_msg.pose.pose.position.z = ST_B_Bi.getOrigin().getZ();

    odom_msg.twist = input->twist; 

    odom_pub.publish(odom_msg); // laser_odom_init

    // Cache the latest synchronized frame for the timer consumer.
    // 写入缓存，生产者写入最新帧数据，消费者定期读取并处理，避免在高频回调中进行重计算。
    {
        std::lock_guard<std::mutex> lk(latest_mutex_);
        latest_scan_ = scanIn;  // 缓存点云消息指针
        latest_tf_ = ST_B_Bi;
        latest_stamp_ = scanIn->header.stamp;
        latest_seq_ = scanIn->header.seq;
        latest_ready_.store(true, std::memory_order_release);
    }
}
// 定时器回调函数，定期执行点云处理和地图更新的操作。它将当前的点云数据进行下采样和坐标变换，并将处理后的点云发布到注册点云话题上。
void SlamOutput::execute(const ros::TimerEvent&)
{   
    if (!is_get_first) {
        return;
    }

    if (!latest_ready_.load(std::memory_order_acquire)) {
        return;
    }
    // 锁内取走一帧，锁外重新计算
    // 作为“本次消费”的快照。后面重计算很耗时，不能一直拿着锁；拷贝到局部变量后，锁就可以释放。
    sensor_msgs::PointCloud2ConstPtr scanIn;
    tf::StampedTransform T_b_bi;
    ros::Time stamp;
    uint32_t seq = 0;
    {
        std::lock_guard<std::mutex> lk(latest_mutex_);
        if (!latest_ready_.load(std::memory_order_relaxed) || !latest_scan_) {
            return;
        }
        scanIn = latest_scan_;
        T_b_bi = latest_tf_;
        stamp = latest_stamp_;
        seq = latest_seq_;
        latest_ready_.store(false, std::memory_order_release);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr scan = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::fromROSMsg(*scanIn, *scan);

    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_data = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    std::vector<int> scan_index;
    pcl::removeNaNFromPointCloud(*scan, *scan_data, scan_index);

    downSizeFilter.setInputCloud(scan_data);
    pcl::PointCloud<pcl::PointXYZI> scan_dwz;
    downSizeFilter.filter(scan_dwz);

    Eigen::Matrix4f pose;
    pose << T_b_bi.getBasis()[0][0], T_b_bi.getBasis()[0][1], T_b_bi.getBasis()[0][2], T_b_bi.getOrigin()[0],
            T_b_bi.getBasis()[1][0], T_b_bi.getBasis()[1][1], T_b_bi.getBasis()[1][2], T_b_bi.getOrigin()[1],
            T_b_bi.getBasis()[2][0], T_b_bi.getBasis()[2][1], T_b_bi.getBasis()[2][2], T_b_bi.getOrigin()[2],
            0, 0, 0, 1;

    pcl::PointCloud<pcl::PointXYZI> scan_tf;
    pcl::transformPointCloud(scan_dwz, scan_tf, pose);

    pcl::PointCloud<pcl::PointXYZ>::Ptr registered_scan = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    registered_scan->points.reserve(scan_tf.points.size());
    for (const auto &point: scan_tf.points) {
        registered_scan->points.emplace_back(point.x, point.y, point.z);
    }

    *exploredAreaCloud += *registered_scan;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInDwz(new pcl::PointCloud<pcl::PointXYZ>());
    exploredAreaDwzFilter.setInputCloud(exploredAreaCloud);
    exploredAreaDwzFilter.filter(*cloudInDwz);

    sensor_msgs::PointCloud2 scan_data_msg;
    pcl::toROSMsg(*cloudInDwz, scan_data_msg);
    scan_data_msg.header.stamp = stamp;
    scan_data_msg.header.frame_id = frame_id;
    scan_data_msg.header.seq = seq;
    reg_pub.publish(scan_data_msg);
}
