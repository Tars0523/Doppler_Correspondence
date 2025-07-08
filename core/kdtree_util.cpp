#include <core/kdtree_util.hpp>


namespace doppler::kdtree {

pcl::PointCloud<pcl::PointXYZ>::Ptr
make_cloud(const std::vector<Eigen::Vector3d>& pts)
{
    auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->resize(pts.size());
    for (std::size_t i = 0; i < pts.size(); ++i) {
        (*cloud)[i].x = static_cast<float>(pts[i].x());
        (*cloud)[i].y = static_cast<float>(pts[i].y());
        (*cloud)[i].z = static_cast<float>(pts[i].z());
    }
    return cloud;
}

pcl::KdTreeFLANN<pcl::PointXYZ>
build_tree(const std::vector<Eigen::Vector3d>& pts)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(make_cloud(pts));
    return tree; 
}

} // namespace doppler::kdtree
