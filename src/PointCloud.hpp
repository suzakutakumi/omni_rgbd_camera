#pragma once

#include <pcl/common/common_headers.h>

class PointCloud
{
public:
    using pc = pcl::PointCloud<pcl::PointXYZRGB>;
    using pc_ptr = pc::Ptr;
    PointCloud();
    PointCloud(pc);
    void save_to_pcd(const std::string &) const;
    pc_ptr get_cloud() const { return cloud; }
    void z_range_filter(int,int);
    void filter(void (*func)(pcl::PointXYZRGB &));
    void filter(pcl::PointXYZRGB &(*func)(const pcl::PointXYZRGB &));
    PointCloud extended(const PointCloud &);

private:
    pc_ptr cloud;
};