#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <Eigen/Dense>

namespace doppler {

class IcpDopplerNode
{
public:
    IcpDopplerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~IcpDopplerNode() = default;

private: 
    /// @brief Load parameters from the parameter server
    ros::NodeHandle  nh_;
    ros::NodeHandle  pnh_;
    ros::Subscriber  pcl_sub_;
    ros::Publisher   pose_pub_;
    ros::Publisher   path_pub_;
    ros::Publisher   global_map_pub_;
    ros::Publisher   odom_pub_;

private: 
    /// @brief Parameters for scan matching
    std::string point_cloud_topic_;  ///< Topic to subscribe for point clouds
    int    max_iters_;               /// Maximum number of ICP iterations
    int    downsample_factor_      ; ///< Uniform downsampling factor
    double max_corr_distance_      ; ///< Maximum spatial correspondence distance (meter), used for closest point correspondence
    double lambda_doppler_         ; ///< Weight factor for Doppler term [0,1], 0 means only closest point correspondence, 1 means only Doppler Correspondence
    double period_                 ; ///< Time period between previous and current scans (second)
    double huberThreshold_         ; ///< Huber robust threshold (meter)
    double doppler_spatial_corr_distance_ ; /// Maximum Doppler Correspondence spatial distance (meter)
    double doppler_corr_distance_  ; /// Maximum Doppler Correspondence distance (meter^2)

private:
    /// @brief State variables
    bool   is_initial_  ;  ///< first frame flag

    Eigen::Matrix4d pose_          {Eigen::Matrix4d::Identity()}; ///< current pose w.r.t the global frame
    Eigen::Matrix4d prev_rel_pose_ {Eigen::Matrix4d::Identity()}; ///< previous relative pose w.r.t target frame

    sensor_msgs::PointCloud prev_pcl_; ///< previous point cloud
    sensor_msgs::PointCloud curr_pcl_; ///< current point cloud
    sensor_msgs::PointCloud global_pcl_; ///< global point cloud for mapping

    nav_msgs::Path path_msg_;

private:
    /// @brief Functions

    /// Pull parameters from parameter server into the member variables above
    void loadParameters();

    /// Main callback: runs Doppler-4D-ICP on incoming cloud
    void pclCallback(const sensor_msgs::PointCloud& msg);

    /// Convert ROS `PointCloud` → Eigen XYZ + Doppler vector
    void parseRosPointCloud(const sensor_msgs::PointCloud& pc,
                            std::vector<Eigen::Vector3d>& pts,
                            std::vector<double>&          dop);

    /// Transform `prev_pcl_` into map frame and accumulate in `global_pcl_`
    void mapping();

    /// Publish current pose
    void publishPose(const ros::Time& stamp);

    /// Append current pose to `path_msg_` and publish
    void pushPath(const ros::Time& stamp);

    /// Publish odometry message
    void publishOdometry(const ros::Time& stamp);
};
} // namespace doppler
