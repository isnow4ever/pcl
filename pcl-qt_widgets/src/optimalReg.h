#pragma once

#include "res.h"
#include <iostream>
#include <algorithm> 
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
#include "pcl/filters/voxel_grid.h" 
#include "pcl/features/fpfh.h" 
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

class OptimalRegistration
{
public:
	OptimalRegistration();
	~OptimalRegistration();

	bool computePointToPlaneDistance(const PointCloudT::Ptr cloud,
		const std::vector<double> &plane,
		std::vector<double> &distance);

	bool createSlicingPlanes(const std::vector<double> &datum_plane,
		const PointCloudT::Ptr cloud,
		std::vector< std::vector<double> > &slicing_planes,
		int number);

	bool estimatePointsBySlicing(const std::vector<double> &slicing_plane,
		const PointCloudT::Ptr cloud_model,
		const PointCloudT::Ptr cloud_data,
		const double epsilon,
		PointCloudT::Ptr &points_on_plane_m,
		PointCloudT::Ptr &points_on_plane_d);

	bool computeConvexHull(const PointCloudT::Ptr points_on_plane,
		PointCloud<PointXY>::Ptr &chull_points,
		std::vector<pcl::Vertices> &polygons);

	bool estimateInnerPoint(const pcl::PointXY point, const PointCloud<PointXY>::Ptr chull_points_outside);

	bool estimateEveloped(const double probability, std::vector<double> &dist);

	bool setModelCloud(PointCloudT::Ptr cloud);

	bool setDataCloud(PointCloudT::Ptr cloud);

	bool setDataNormalCloud(PointCloudPN::Ptr cloud);

	bool setSurfaceDataNormalCloud(PointCloudPN::Ptr cloud);

	double getEnvelopedRate();

	double computeFitness(Eigen::Matrix4d &transformation);

	void computeDatumCoefficients(PointCloud<PointNormal>::Ptr, PointCloud<PointXYZ>::Ptr, pcl::ModelCoefficients::Ptr);

	double computeDatumError(PointCloud<PointXYZ>::Ptr, PointCloud<PointXYZ>::Ptr, Normal&);

	double computeDatumAngle(pcl::ModelCoefficients::Ptr, pcl::ModelCoefficients::Ptr);

	double computeSurfaceVariance(std::vector<double> &);

private:
	PointCloudT::Ptr datum_data;
	PointCloudT::Ptr datum_model;

	PointCloudT::Ptr surface_model;
	PointCloudT::Ptr surface_data;
	PointCloudPN::Ptr model_with_normals;
	PointCloudPN::Ptr data_with_normals;
	PointCloudPN::Ptr surface_data_with_normals;

	double enveloped_rate;
};

