#include <Eigen/Dense>
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace doppler {

/// @brief Run iterative closest-point refinement with Doppler Correspondence + Closest Point Correspondence
Eigen::Matrix4d iterate_icp(std::vector<Eigen::Vector3d>&      src_spa, ///< Source points for using closest point correspondence
                            const std::vector<Eigen::Vector3d>& tgt_pts, ///< Target points for using closest point correspondence
                            std::vector<Eigen::Vector3d>&      src_dop, ///< Source points for using Doppler correspondence
                            const std::vector<Eigen::Vector3d>& tgt_dop, ///< Target points for using Doppler correspondence
                            pcl::KdTreeFLANN<pcl::PointXYZ>&    kdtree, ///< KD-tree built on @p tgt_pts for NN queries
                            int   max_iters, ///< Maximum ICP iterations
                            double max_corr_dist, ///< Maximum correspondence distance (meter)
                            double lambda_dop, ///< Weight for Doppler term (0 → spatial-only, 1 → Doppler-only)
                            double huber_thresh); ///< Huber robust threshold (meter)
/// @return SE(3) representing cumulative source2target transform
} // namespace doppler
