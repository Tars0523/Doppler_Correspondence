#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>

namespace doppler::kdtree {

/// Convert Eigen point array → pcl::PointCloud<pcl::PointXYZ>
pcl::PointCloud<pcl::PointXYZ>::Ptr
make_cloud(const std::vector<Eigen::Vector3d>& pts);

/// Build KD‑tree 
pcl::KdTreeFLANN<pcl::PointXYZ>
build_tree(const std::vector<Eigen::Vector3d>& pts);

} // namespace doppler::kdtree
