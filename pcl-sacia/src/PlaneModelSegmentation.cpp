#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ>);


	// check arguments
	if(argc != 2) {
		std::cout << "ERROR: the number of arguments is illegal!" << std::endl;
		return -1;
	}

	// load pcd file
	if(pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud_source)==-1) {
		std::cout << "load source failed!" << std::endl;
		return -1;
	}

	std::cout << "source loaded!" << std::endl;


	// pass through filter to get the certain field
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pt;
	pt.setInputCloud(cloud_source);
	pt.setFilterFieldName("y");
	pt.setFilterLimits(-0.1, 0.6);
	pt.filter(*cloud_source_filtered);
	
	// segmentation
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

	pcl::SACSegmentation<pcl::PointXYZ> sac;
	sac.setInputCloud(cloud_source_filtered);    // cloud_source_filtered 为提取桌子表面 cloud_source 为提取地面
	sac.setMethodType(pcl::SAC_RANSAC);
	sac.setModelType(pcl::SACMODEL_PLANE);
	sac.setDistanceThreshold(0.01);
	sac.setMaxIterations(100);
	sac.setProbability(0.95);

	sac.segment(*inliers, *coefficients);
	
	// extract the certain field
	pcl::ExtractIndices<pcl::PointXYZ> ei;
	ei.setIndices(inliers);
	ei.setInputCloud(cloud_source_filtered);    // cloud_source_filtered 为提取桌子表面 cloud_source 为提取地面
	ei.filter(*cloud_target);

	std::cout << *coefficients << std::endl;

	// display
	pcl::visualization::PCLVisualizer p;
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h (cloud_target, 255, 0, 0);
	p.addPointCloud(cloud_target, tgt_h, "target");
	p.spinOnce();
	
	pcl::visualization::PCLVisualizer p2;
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h (cloud_source, 0, 255, 0);
	p2.addPointCloud(cloud_source, src_h, "source");
	p2.spin();

	

	return 0;
}
