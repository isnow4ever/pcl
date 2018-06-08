#include "Sac_IA.h"

using namespace pcl;
using namespace std;
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;


Sac_IA::Sac_IA()
{
}


Sac_IA::~Sac_IA()
{
}


void
Sac_IA::voxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out, float gridsize) {
	VoxelGrid<PointXYZ> vox_grid;
	vox_grid.setLeafSize(gridsize, gridsize, gridsize);
	vox_grid.setInputCloud(cloud_in);
	vox_grid.filter(*cloud_out);
	return;
}

void
Sac_IA::passFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, double filter_limit) {
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(pc);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(0, filter_limit);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(0, filter_limit);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0, filter_limit);
	pass.filter(*pc);
	return;
}

PointCloud<FPFHSignature33>::Ptr
Sac_IA::getFeatures(PointCloud<PointXYZ>::Ptr cloud, PointCloud<Normal>::Ptr normals, double feature_radius) {

	PointCloud<FPFHSignature33>::Ptr features = PointCloud<FPFHSignature33>::Ptr(new PointCloud<FPFHSignature33>);
	search::KdTree<PointXYZ>::Ptr search_method_ptr = search::KdTree<PointXYZ>::Ptr(new search::KdTree<PointXYZ>);
	FPFHEstimationOMP<PointXYZ, Normal, FPFHSignature33> fpfh_est;
	fpfh_est.setInputCloud(cloud);
	fpfh_est.setInputNormals(normals);
	fpfh_est.setSearchMethod(search_method_ptr);
	fpfh_est.setRadiusSearch(feature_radius);
	fpfh_est.compute(*features);
	return features;
}

PointCloud<Normal>::Ptr
Sac_IA::getNormals(PointCloud<PointXYZ>::Ptr incloud, double normals_radius) {

	PointCloud<Normal>::Ptr normalsPtr = PointCloud<Normal>::Ptr(new PointCloud<Normal>);
	NormalEstimationOMP<PointXYZ, Normal> norm_est;
	norm_est.setInputCloud(incloud);
	norm_est.setRadiusSearch(normals_radius);
	norm_est.compute(*normalsPtr);
	return normalsPtr;
}

SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33>


Sac_IA::align(PointCloud<PointXYZ>::Ptr cloud1, PointCloud<PointXYZ>::Ptr cloud2,
	PointCloud<FPFHSignature33>::Ptr features1, PointCloud<FPFHSignature33>::Ptr features2,
	int max_sacia_iterations, double min_correspondence_dist, double max_correspondence_dist) {

	SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
	Eigen::Matrix4f final_transformation;
	sac_ia.setInputCloud(cloud2);
	sac_ia.setSourceFeatures(features2);
	sac_ia.setInputTarget(cloud1);
	sac_ia.setTargetFeatures(features1);
	sac_ia.setMaximumIterations(max_sacia_iterations);
	sac_ia.setMinSampleDistance(min_correspondence_dist);
	sac_ia.setMaxCorrespondenceDistance(max_correspondence_dist);
	PointCloud<PointXYZ> finalcloud;
	sac_ia.align(finalcloud);
	sac_ia.getCorrespondenceRandomness();
	return sac_ia;
}

void
Sac_IA::view(pcl::PointCloud<pcl::PointXYZ> &cloud)
{
	pcl::visualization::CloudViewer viewer1("Cloud Viewer");
	viewer1.showCloud(cloud.makeShared());
	while (!viewer1.wasStopped());
	return;
}

void
Sac_IA::viewPair(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1al, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2al) {

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->initCameraParameters();

	int v1(0), v2(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("Before Alignment", 10, 10, "v1 text", v1);
	PointCloudColorHandlerCustom<PointXYZ> green(cloud1, 0, 255, 0);
	PointCloudColorHandlerCustom<PointXYZ> red(cloud2, 255, 0, 0);
	viewer->addPointCloud(cloud1, green, "v1_target", v1);
	viewer->addPointCloud(cloud2, red, "v1_sourse", v1);

	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0, 0, 0, v2);
	viewer->addText("After Alignment", 10, 10, "v2 text", v2);
	PointCloudColorHandlerCustom<PointXYZ> green2(cloud1al, 0, 255, 0);
	PointCloudColorHandlerCustom<PointXYZ> red2(cloud2al, 255, 0, 0);
	viewer->addPointCloud(cloud1al, green2, "v2_target", v2);
	viewer->addPointCloud(cloud2al, red2, "v2_sourse", v2);
	viewer->spin();

	return;
}

