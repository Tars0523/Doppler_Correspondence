#include <core/math_utils.hpp>

#include <Eigen/Dense>
#include <unordered_map>
#include <cmath>

namespace doppler::math {

std::vector<double> compute_norms(const std::vector<Eigen::Vector3d>& pts)
{
    const std::size_t N = pts.size();
    if (N == 0) return {};
    Eigen::Matrix<double,3,Eigen::Dynamic> M(3,N);
    for(std::size_t i=0;i<N;++i) M.col(static_cast<Eigen::Index>(i)) = pts[i];
    Eigen::RowVectorXd n = M.colwise().norm();
    return {n.data(), n.data()+n.size()};
}

std::vector<double> doppler_range_src(const std::vector<double>& norms,
                                      const std::vector<double>& dop,
                                      double dt)
{
    Eigen::Map<const Eigen::VectorXd> n(norms.data(), norms.size());
    Eigen::Map<const Eigen::VectorXd> d(dop.data(),  dop.size());
    Eigen::VectorXd out = n.array().square() + n.array()*d.array()*dt;
    return {out.data(), out.data()+out.size()};
}

std::vector<double> doppler_range_tgt(const std::vector<double>& norms,
                                      const std::vector<double>& dop,
                                      double dt)
{
    Eigen::Map<const Eigen::VectorXd> n(norms.data(), norms.size());
    Eigen::Map<const Eigen::VectorXd> d(dop.data(),  dop.size());
    Eigen::VectorXd out = n.array().square() - n.array()*d.array()*dt;
    return {out.data(), out.data()+out.size()};
}

double huber_weight(double e, double t)
{
    double ae = std::abs(e);
    return (ae <= t) ? 1.0 : t / ae;
}

void uniform_downsample(const std::vector<Eigen::Vector3d>& in_pts,
                        const std::vector<double>&          in_dop,
                        int                                 stride,
                        std::vector<Eigen::Vector3d>&       out_pts,
                        std::vector<double>&                out_dop)
{
    out_pts.clear();
    out_dop.clear();

    if (stride <= 1) {
        out_pts = in_pts;
        out_dop = in_dop;
        return;
    }

    out_pts.reserve(in_pts.size() / stride);
    out_dop.reserve(in_dop.size() / stride);

    for (std::size_t i = 0; i < in_pts.size(); i += stride) {
        out_pts.push_back(in_pts[i]);
        out_dop.push_back(in_dop[i]);
    }
}


} // namespace doppler::math
