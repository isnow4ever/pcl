#pragma once
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <QMessageBox>

using namespace pcl;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::Normal> PointCloudN;

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<PointC> PointCloudC;

