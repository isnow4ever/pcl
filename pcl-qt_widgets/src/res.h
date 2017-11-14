#pragma once
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <Eigen/Geometry> 

#include <QMessageBox>
#include <iostream>
#include "time.h"

#define PI 3.1415926535897932

using namespace pcl;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::Normal> PointCloudN;

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<PointC> PointCloudC;

