#include <pipeline/icp_doppler.hpp>
#include <pipeline/correspondence.hpp>
#include <pipeline/icp_iter.hpp>

#include <core/math_utils.hpp>
#include <core/kdtree_util.hpp>

#include <Eigen/Dense>
#include <vector>
#include <utility>

#include <chrono>

namespace doppler {
using namespace math;      
using namespace corr;      
using namespace kdtree;    

/// @brief Pipeline for Doppler 4D ICP registration
Eigen::Matrix4d icp_doppler(const std::vector<Eigen::Vector3d>& prev_pts,
                            const std::vector<double>&          prev_dop,
                            const std::vector<Eigen::Vector3d>& curr_pts,
                            const std::vector<double>&          curr_dop,
                            const Eigen::Matrix4d&              init_T,
                            int    downsample_factor,
                            double lambda_dop,
                            int    max_iters,
                            double max_corr_dist,
                            double doppler_spatial_corr_distance,
                            double doppler_corr_distance,
                            double huber_thresh,
                            double period)
{
    // Downsample the point clouds
    std::vector<Eigen::Vector3d> src_raw, tgt_pts;
    std::vector<double>          src_dop_raw, tgt_dop;

    uniform_downsample(prev_pts, prev_dop, downsample_factor, src_raw, src_dop_raw);
    uniform_downsample(curr_pts, curr_dop, downsample_factor, tgt_pts, tgt_dop);

    // Compute Doppler Correspondence
    std::vector<Eigen::Vector3d> src_dop_pts, tgt_dop_pts;
    build_doppler_matches(src_raw, src_dop_raw,
                          tgt_pts, tgt_dop,
                          period,
                          doppler_corr_distance,
                          doppler_spatial_corr_distance,
                          src_dop_pts, tgt_dop_pts);

    // Build KD-Tree for target points    
    auto kdtree = build_tree(tgt_pts);

    // Prediction
    std::vector<Eigen::Vector3d> src_spa = src_raw; 
    auto applyT = [](std::vector<Eigen::Vector3d>& V, const Eigen::Matrix4d& T){
        const Eigen::Matrix3d R = T.block<3,3>(0,0);
        const Eigen::Vector3d t = T.block<3,1>(0,3);
        for (std::size_t i = 0; i < V.size(); ++i) V[i] = R*V[i] + t;
    };
    applyT(src_spa,    init_T);
    applyT(src_dop_pts,init_T);
    
    // Doppler 4D ICP for refinement
    Eigen::Matrix4d deltaT = iterate_icp(src_spa, tgt_pts,
                                         src_dop_pts, tgt_dop_pts,
                                         kdtree,
                                         max_iters, max_corr_dist,
                                         lambda_dop, huber_thresh);

    return deltaT * init_T; 
}

} // namespace doppler
