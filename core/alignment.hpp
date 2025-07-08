#include <Eigen/Dense>
#include <vector>

namespace doppler::align {

// returns the residual and Jacobian
void residual_and_jacobian(const Eigen::Vector3d& src,
                           const Eigen::Vector3d& tgt,
                           Eigen::Vector3d&       r,
                           Eigen::Matrix<double,3,6>& J);

// Ax = b , Accumulate A and b
void accumulate_Ab(const std::vector<Eigen::Vector3d>& src_spa,
                   const std::vector<Eigen::Vector3d>& tgt_spa,
                   const std::vector<Eigen::Vector3d>& src_dop,
                   const std::vector<Eigen::Vector3d>& tgt_dop,
                   double lambda_dop,
                   double huber_thresh,
                   Eigen::Matrix<double,6,6>& A,
                   Eigen::Matrix<double,6,1>& b);
} // namespace doppler::align
