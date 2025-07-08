#include <pipeline/correspondence.hpp>
#include <core/math_utils.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace doppler::corr {
using math::compute_norms;
using math::doppler_range_src;
using math::doppler_range_tgt;

void build_doppler_matches(
    const std::vector<Eigen::Vector3d>& src_pts,
    const std::vector<double>&          src_dop,
    const std::vector<Eigen::Vector3d>& tgt_pts,
    const std::vector<double>&          tgt_dop,
    double  period,
    double  range_tol,
    double  eucl_tol,
    std::vector<Eigen::Vector3d>& src_out,
    std::vector<Eigen::Vector3d>& tgt_out)
{
    src_out.clear(); tgt_out.clear();
    if (src_pts.empty() || tgt_pts.empty()) return;

    // Compute ranges for source and target points
    const auto src_norm = compute_norms(src_pts);
    const auto tgt_norm = compute_norms(tgt_pts);

    // Compute Doppler features
    const auto r_src = doppler_range_src(src_norm, src_dop, period);
    const auto r_tgt = doppler_range_tgt(tgt_norm, tgt_dop, period);

    // Compute Doppler Correspondence
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->resize(tgt_pts.size());
    for (std::size_t i=0;i<tgt_pts.size();++i)
        (*cloud)[i].x = static_cast<float>(r_tgt[i]);   
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    const float range_tol_f  = static_cast<float>(range_tol);
    const double eucl_tol_sq = eucl_tol * eucl_tol;

    std::vector<int>   idx(1);
    std::vector<float> dist2(1);

    for (std::size_t i=0;i<src_pts.size();++i)
    {
        pcl::PointXYZ query; query.x = static_cast<float>(r_src[i]);
        if (kdtree.nearestKSearch(query, 1, idx, dist2) == 0) continue;
        if (std::sqrt(dist2[0]) <= range_tol_f &&
            (src_pts[i] - tgt_pts[idx[0]]).squaredNorm() <= eucl_tol_sq)
        {
            {
                src_out.push_back(src_pts[i]);
                tgt_out.push_back(tgt_pts[idx[0]]);
            }
        }
    }
}
} // namespace doppler::corr


