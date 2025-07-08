#include <core/alignment.hpp>
#include <core/math_utils.hpp>

#include <Eigen/Dense>
#include <vector>

namespace doppler::align {
using doppler::math::huber_weight;

void residual_and_jacobian(const Eigen::Vector3d&  src,
                           const Eigen::Vector3d&  tgt,
                           Eigen::Vector3d&        r,
                           Eigen::Matrix<double,3,6>& J)
{
    r = tgt - src;

    Eigen::Matrix3d cross;
    cross <<      0, -src.z(),  src.y(),
              src.z(),       0, -src.x(),
             -src.y(),  src.x(),       0;

    J.block<3,3>(0,0).setIdentity();    
    J.block<3,3>(0,3) = cross.transpose(); 
}

void accumulate_Ab(const std::vector<Eigen::Vector3d>& src_spa,
                   const std::vector<Eigen::Vector3d>& tgt_spa,
                   const std::vector<Eigen::Vector3d>& src_dop,
                   const std::vector<Eigen::Vector3d>& tgt_dop,
                   double lambda_dop,
                   double huber_thresh,
                   Eigen::Matrix<double,6,6>& A_out,
                   Eigen::Matrix<double,6,1>& b_out)
{
    A_out.setZero(); b_out.setZero();

    const double w_spa = 1.0 - lambda_dop;
    const double w_dop = lambda_dop;


    {
        Eigen::Matrix<double,6,6> A_loc = Eigen::Matrix<double,6,6>::Zero();
        Eigen::Matrix<double,6,1> b_loc = Eigen::Matrix<double,6,1>::Zero();

        for (std::size_t i = 0; i < src_spa.size(); ++i) {
            Eigen::Vector3d r; Eigen::Matrix<double,3,6> J;
            residual_and_jacobian(src_spa[i], tgt_spa[i], r, J);
            const double w = huber_weight(r.norm(), huber_thresh);
            A_loc.noalias() += w_spa * w * (J.transpose() * J);
            b_loc.noalias() += w_spa * w * (J.transpose() * r);
        }

        for (std::size_t i = 0; i < src_dop.size(); ++i) {
            Eigen::Vector3d r; Eigen::Matrix<double,3,6> J;
            residual_and_jacobian(src_dop[i], tgt_dop[i], r, J);
            const double w = huber_weight(r.norm(), huber_thresh);
            A_loc.noalias() += w_dop * w * (J.transpose() * J);
            b_loc.noalias() += w_dop * w * (J.transpose() * r);
        }

        {
            A_out += A_loc;
            b_out += b_loc;
        }
    }
}

} // namespace doppler::align
