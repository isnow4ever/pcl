#pragma once

#include <iostream>
#include <time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include "pcl/point_cloud.h" 
#include "pcl/kdtree/kdtree_flann.h" 
#include "pcl/filters/passthrough.h" 
#include "pcl/filters/voxel_grid.h" 
#include "pcl/features/fpfh.h" 

class Sac_IA
{
public:
	Sac_IA();
	~Sac_IA();

	//fliters
	void voxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out, float gridsize = 2.0f);
	void passFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, double filter_limit = 1000.0);

	//features
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr getFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, double feature_radius = 50.0);
	pcl::PointCloud<pcl::Normal>::Ptr getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr incloud, double normals_radius = 20.0);

	//sac-ia
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33>

		align(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2,
			pcl::PointCloud<pcl::FPFHSignature33>::Ptr features1, pcl::PointCloud<pcl::FPFHSignature33>::Ptr features2,
			int max_sacia_iterations = 2000, double min_correspondence_dist = 3, double max_correspondence_dist = 2000);

	//visualization
	void view(pcl::PointCloud<pcl::PointXYZ> &cloud);
	void viewPair(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2,
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1al, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2al);
};

