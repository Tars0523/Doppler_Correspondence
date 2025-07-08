#include <Eigen/Dense>
#include <vector>

namespace doppler::math {
// compute norms
std::vector<double>
compute_norms(const std::vector<Eigen::Vector3d>& pts);

// compute Doppler feature 
std::vector<double>
doppler_range_src(const std::vector<double>& norms,
                  const std::vector<double>& dopplers,
                  double dt);
std::vector<double>
doppler_range_tgt(const std::vector<double>& norms,
                  const std::vector<double>& dopplers,
                  double dt);

// robust loss 
double huber_weight(double err, double thresh);

// uniform down-sampling
void uniform_downsample(const std::vector<Eigen::Vector3d>& in_pts,
                        const std::vector<double>&          in_dop,
                        int                                 stride,
                        std::vector<Eigen::Vector3d>&       out_pts,
                        std::vector<double>&                out_dop);
                        
                
} // namespace doppler::math
