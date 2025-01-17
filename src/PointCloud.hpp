#pragma once

#include <pcl/common/common_headers.h>
#include <functional>

class PointCloud
{
public:
    using pc = pcl::PointCloud<pcl::PointXYZRGB>;
    using pc_ptr = pc::Ptr;
    PointCloud();
    PointCloud(pc);
    void save_to_pcd(const std::string &) const;
    pc_ptr get_cloud() const { return cloud; }
    void z_range_filter(int, int);
    void filter(std::function<void(pcl::PointXYZRGB &)>);
    void filter(std::function<pcl::PointXYZRGB &(const pcl::PointXYZRGB &)>);
    PointCloud extended(const PointCloud &);

private:
    pc_ptr cloud;
};