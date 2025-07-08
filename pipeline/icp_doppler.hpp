#include <Eigen/Dense>
#include <vector>

namespace doppler {
/// @brief Doppler 4D Iterative Closest Point registration (Local Point Cloud Registration)
Eigen::Matrix4d icp_doppler(const std::vector<Eigen::Vector3d>& prev_pts, ///< Previous scan points (source)
                            const std::vector<double>&          prev_dop, ///< Doppler values paired with @p prev_pts (m/s)
                            const std::vector<Eigen::Vector3d>& curr_pts, ///< Current scan points (target)
                            const std::vector<double>&          curr_dop, ///< Doppler values paired with @p curr_pts
                            const Eigen::Matrix4d&              init_T, ///< Initial guess SE(3) (prev→curr) 
                            int    downsample_factor,
                            double lambda_dop, ///< Weight factor for Doppler term [0,1]
                            int    max_iters, ///< Maximum ICP iterations
                            double max_corr_dist, ///< Maximum spatial correspondence distance (meter)
                            double doppler_spatial_corr_distance, ///< Doppler Correspondence spatial correlation distance (meter)
                            double doppler_corr_distance, ///< Doppler Correspondence distance (meter^2)
                            double huber_thresh, ///< Huber robust threshold (meter)
                            double period); ///< Time period between previous and current scans (second)
/// @return SE(3) matrix representing refined prev→curr transform
} // namespace doppler
