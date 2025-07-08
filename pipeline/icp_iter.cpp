// -----------------------------------------------------------------------------
//  core/icp_iter.cpp
//  Implementation of iterate_icp() declared in icp_iter.hpp.
// -----------------------------------------------------------------------------
#include <pipeline/icp_iter.hpp>

#include <core/alignment.hpp>
#include <core/math_utils.hpp>

#include <sophus/se3.hpp>
#include <iostream>
#include <cmath>

namespace doppler {
using namespace align; 

/// @brief Apply SE(3) transformation to a vector of 3D points
static inline void applyT(std::vector<Eigen::Vector3d>& P, const Eigen::Matrix4d& T)
{
    const Eigen::Matrix3d R = T.block<3,3>(0,0);
    const Eigen::Vector3d t = T.block<3,1>(0,3);
    for (std::size_t i = 0; i < P.size(); ++i)
        P[i] = R * P[i] + t;
}

Eigen::Matrix4d iterate_icp(std::vector<Eigen::Vector3d>&      src_spa,
                            const std::vector<Eigen::Vector3d>& tgt_pts,
                            std::vector<Eigen::Vector3d>&      src_dop,
                            const std::vector<Eigen::Vector3d>& tgt_dop,
                            pcl::KdTreeFLANN<pcl::PointXYZ>&    kdtree,
                            int   max_iters,
                            double max_corr_dist,
                            double lambda_dop,
                            double huber_thresh)
{
    const double max_corr_sq = max_corr_dist * max_corr_dist;
    std::vector<int>   nn_idx(1);
    std::vector<float> nn_dist(1);

    Eigen::Matrix4d cumulative = Eigen::Matrix4d::Identity();
    double prev_err = 1e9;
    constexpr double eps = 1e-4;

    for (int iter = 0; iter < max_iters; ++iter) {
        // Find closest point correspondences
        std::vector<Eigen::Vector3d> in_src, in_tgt;
        in_src.reserve(src_spa.size());
        in_tgt.reserve(src_spa.size());

        for (std::size_t i = 0; i < src_spa.size(); ++i) {
            pcl::PointXYZ q; q.x = static_cast<float>(src_spa[i].x());
            q.y = static_cast<float>(src_spa[i].y()); q.z = static_cast<float>(src_spa[i].z());
            if (kdtree.nearestKSearch(q, 1, nn_idx, nn_dist) > 0 && nn_dist[0] < max_corr_sq) {
                {
                    in_src.push_back(src_spa[i]);
                    in_tgt.push_back(tgt_pts[nn_idx[0]]);
                }
            }
        }

        // Make Ax=b from closet and doppler correspondence
        Eigen::Matrix<double,6,6> A; Eigen::Matrix<double,6,1> b;
        accumulate_Ab(in_src, in_tgt, src_dop, tgt_dop,
                      lambda_dop, huber_thresh, A, b);

        // Solve
        Eigen::Matrix<double,6,1> xi = A.ldlt().solve(b);
        Sophus::SE3d dT = Sophus::SE3d::exp(xi);
        Eigen::Matrix4d dTmat = dT.matrix();
        cumulative = dTmat * cumulative;

        // Update
        applyT(src_spa,   dTmat);
        applyT(src_dop,   dTmat);

        // Convergence check
        double mean_err = 0.0;
        if (!in_src.empty()) {
            Eigen::Matrix<double,3,Eigen::Dynamic> S(3,in_src.size()),
                                                   D(3,in_src.size());
            for (std::size_t i = 0; i < in_src.size(); ++i) {
                S.col(i) = in_src[i];
                D.col(i) = in_tgt[i];
            }
            mean_err = (S - D).colwise().norm().mean();
        }
        if (std::fabs(mean_err - prev_err) < eps) {
            break;
        }
        prev_err = mean_err;
    }

    return cumulative;
}

} // namespace doppler
