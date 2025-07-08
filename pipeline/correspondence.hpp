#pragma once

#include <Eigen/Dense>
#include <vector>

namespace doppler::corr {

/// @brief Build Doppler Correspondence
void build_doppler_matches(
    const std::vector<Eigen::Vector3d>& src_pts, ///< Source points
    const std::vector<double>&          src_dop, ///< Source Doppler measurements
    const std::vector<Eigen::Vector3d>& tgt_pts, ///< Target points
    const std::vector<double>&          tgt_dop, ///< Target Doppler measurements
    double  period, ///< Time gap Δt between scans
    double  range_tol,   ///< Acceptable tolerance distance between Doppler features [m^2]
    double  eucl_tol,    ///< Acceptable Euclidean distance tolerance between points [m]
    std::vector<Eigen::Vector3d>& src_out, ///< Matched source points
    std::vector<Eigen::Vector3d>& tgt_out); ///< Matched target points
/// @return void
}   // namespace doppler::corr
