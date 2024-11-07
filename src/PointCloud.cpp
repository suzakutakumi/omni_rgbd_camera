#include "PointCloud.hpp"

#include <iostream>
#include <algorithm>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <string>

// PCL Headers
#include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
// #include <pcl/filters/statistical_outlier_removal.h>

PointCloud::PointCloud()
{
    cloud = pc_ptr(new pc);
}
PointCloud::PointCloud(pc original_cloud)
{
    cloud = pc_ptr(new pc);
    *cloud = original_cloud;
}

void PointCloud::save_to_pcd(const std::string &n) const
{
    std::string name = n;

    if (name.size() <= 4 || name.substr(name.size() - 4) != ".pcd")
    {
        name = name + std::string(".pcd");
    }

    pcl::io::savePCDFileBinary(name, *cloud);
}

void PointCloud::z_range_filter(int limit_min, int limit_max)
{
    pcl::PassThrough<pcl::PointXYZRGB> Cloud_Filter;
    Cloud_Filter.setInputCloud(cloud);
    Cloud_Filter.setFilterFieldName("z");
    Cloud_Filter.setFilterLimits(limit_min, limit_max);
    Cloud_Filter.filter(*cloud);
}

void PointCloud::filter(void (*filter_func)(pcl::PointXYZRGB &))
{
    for (auto &p : cloud->points)
    {
        filter_func(p);
    }
}

void PointCloud::filter(
    pcl::PointXYZRGB &(*filter_func)(const pcl::PointXYZRGB &))
{
    for (auto &p : cloud->points)
    {
        p = filter_func(p);
    }
}

PointCloud PointCloud::extended(const PointCloud &other)
{
    auto &p1 = *cloud;
    auto p2 = other.get_cloud();
    p1.insert(p1.end(), p2->begin(), p2->end());
    return *this;
}