#include <ros/icp_doppler_.hpp>
#include <pipeline/icp_doppler.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>

#include <Eigen/Core>
#include <sophus/se3.hpp>

namespace doppler {

IcpDopplerNode::IcpDopplerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_{nh}, pnh_{pnh}
{
    loadParameters();

    pose_.setIdentity();
    prev_rel_pose_.setIdentity();
    is_initial_ = true;

    pcl_sub_ = nh_.subscribe(point_cloud_topic_, 10, &IcpDopplerNode::pclCallback, this);

    pose_pub_       = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("variant_icp_PoseWithCovarianceStamped", 100);
    path_pub_       = nh_.advertise<nav_msgs::Path>("variant_icp_Path", 100);
    global_map_pub_ = nh_.advertise<sensor_msgs::PointCloud>("global_pcl", 100);
    odom_pub_       = nh_.advertise<nav_msgs::Odometry>("/odom", 100);

    path_msg_.header.frame_id = "map";
    ROS_INFO_STREAM("[ICP Doppler] Node initialised. Subscribed to " << point_cloud_topic_);
}

void IcpDopplerNode::loadParameters()
{
    pnh_.param<std::string>("point_cloud_topic",  point_cloud_topic_,  "/radar_enhanced_pcl");
    pnh_.param<int>       ("max_iters",          max_iters_,          20);
    pnh_.param<double>    ("max_corr_distance",  max_corr_distance_,  2.0);
    pnh_.param<int>       ("downsample_factor",  downsample_factor_,  1);
    pnh_.param<double>    ("lambda_doppler",     lambda_doppler_,     1.0);
    pnh_.param<double>    ("period",             period_,             0.083);
    pnh_.param<double>    ("huber_threshold",    huberThreshold_,     0.5); 
    pnh_.param<double>    ("doppler_spatial_corr_distance", doppler_spatial_corr_distance_, 3.0);
    pnh_.param<double>    ("doppler_corr_distance", doppler_corr_distance_, 8.0);

    ROS_INFO_STREAM("----------- 4D Doppler ICP Params -----------");
    ROS_INFO_STREAM("  max_iters            : " << max_iters_);
    ROS_INFO_STREAM("  max_corr_distance    : " << max_corr_distance_);
    ROS_INFO_STREAM("  lambda_doppler       : " << lambda_doppler_);
    ROS_INFO_STREAM("  period         : " << period_);
    ROS_INFO_STREAM("  huber_threshold      : " << huberThreshold_);
    ROS_INFO_STREAM("  doppler_spatial_corr_distance : " << doppler_spatial_corr_distance_);
    ROS_INFO_STREAM("  doppler_corr_distance : " << doppler_corr_distance_);
    ROS_INFO_STREAM("  down_sampling_factor : " << downsample_factor_);
}

void IcpDopplerNode::pclCallback(const sensor_msgs::PointCloud& msg)
{
    if (is_initial_) {
        prev_pcl_   = msg;
        publishPose(msg.header.stamp);
        pushPath(msg.header.stamp);
        publishOdometry(msg.header.stamp);
        is_initial_ = false;
        return;
    }

    // map publish
    mapping();
    global_map_pub_.publish(global_pcl_);

    // pointcloud to eigen
    curr_pcl_ = msg;
    std::vector<Eigen::Vector3d> prev_pts, curr_pts;
    std::vector<double>          prev_dop, curr_dop;
    parseRosPointCloud(prev_pcl_, prev_pts, prev_dop);
    parseRosPointCloud(curr_pcl_, curr_pts, curr_dop);

    // Doppler 4D ICP
    Eigen::Matrix4d dT = icp_doppler(prev_pts, prev_dop,
                                     curr_pts, curr_dop,
                                     prev_rel_pose_,
                                     downsample_factor_,
                                     lambda_doppler_, max_iters_,
                                     max_corr_distance_,
                                     doppler_spatial_corr_distance_, doppler_corr_distance_,
                                     huberThreshold_,
                                     period_);

    prev_rel_pose_ = dT;
    pose_          = pose_ * dT.inverse();

    // publish pose, path, and odometry
    publishPose(msg.header.stamp);
    pushPath(msg.header.stamp);
    publishOdometry(msg.header.stamp);

    prev_pcl_ = curr_pcl_;
}

void IcpDopplerNode::parseRosPointCloud(const sensor_msgs::PointCloud& pc,
                                        std::vector<Eigen::Vector3d>&  pts,
                                        std::vector<double>&           dop)
{
    const size_t N = pc.points.size();
    pts.resize(N);
    dop.assign(N, 0.0);

    int dop_idx = -1;
    for (size_t i = 0; i < pc.channels.size(); ++i)
        if (pc.channels[i].name == "Doppler") { dop_idx = static_cast<int>(i); break; }

    for (size_t i = 0; i < N; ++i) {
        pts[i] = {pc.points[i].x, pc.points[i].y, pc.points[i].z};
        if (dop_idx >= 0) dop[i] = pc.channels[dop_idx].values[i];
    }
}

void IcpDopplerNode::mapping()
{
    std::vector<Eigen::Vector3d> pts; std::vector<double> dops;
    parseRosPointCloud(prev_pcl_, pts, dops);

    global_pcl_.header.frame_id = "map";
    global_pcl_.header.stamp    = prev_pcl_.header.stamp;

    const size_t N = pts.size();
    global_pcl_.points.resize(N);
    global_pcl_.channels.clear();
    sensor_msgs::ChannelFloat32 dop_ch; dop_ch.name = "Doppler"; dop_ch.values.resize(N);

    for (size_t i = 0; i < N; ++i) {
        const Eigen::Vector4d p_h(pts[i].x(), pts[i].y(), pts[i].z(), 1.0);
        const Eigen::Vector4d w  = pose_ * p_h;
        geometry_msgs::Point32 p; p.x = w.x(); p.y = w.y(); p.z = w.z();
        global_pcl_.points[i]  = p;
        dop_ch.values[i]       = dops[i];
    }
    global_pcl_.channels.push_back(dop_ch);
}

void IcpDopplerNode::publishPose(const ros::Time& stamp)
{
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp    = stamp;

    Eigen::Vector3d t = pose_.block<3,1>(0,3);
    Eigen::Quaterniond q(pose_.block<3,3>(0,0)); q.normalize();

    msg.pose.pose.position.x = t.x(); msg.pose.pose.position.y = t.y(); msg.pose.pose.position.z = t.z();
    msg.pose.pose.orientation.x = q.x(); msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z(); msg.pose.pose.orientation.w = q.w();

    pose_pub_.publish(msg);
}

void IcpDopplerNode::pushPath(const ros::Time& stamp)
{
    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = "map"; ps.header.stamp = stamp;

    Eigen::Vector3d t = pose_.block<3,1>(0,3);
    Eigen::Quaterniond q(pose_.block<3,3>(0,0)); q.normalize();

    ps.pose.position.x = t.x(); ps.pose.position.y = t.y(); ps.pose.position.z = t.z();
    ps.pose.orientation.x = q.x(); ps.pose.orientation.y = q.y(); ps.pose.orientation.z = q.z(); ps.pose.orientation.w = q.w();

    path_msg_.poses.push_back(ps);
    path_pub_.publish(path_msg_);
}

void IcpDopplerNode::publishOdometry(const ros::Time& stamp)
{
    nav_msgs::Odometry odom;
    odom.header.frame_id = "map"; 
    odom.child_frame_id = "base_link";
    odom.header.stamp = stamp;

    Eigen::Vector3d t = pose_.block<3,1>(0,3);
    Eigen::Quaterniond q(pose_.block<3,3>(0,0)); q.normalize();

    odom.pose.pose.position.x = t.x();
    odom.pose.pose.position.y = t.y();
    odom.pose.pose.position.z = t.z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom_pub_.publish(odom);
}

} // namespace doppler
