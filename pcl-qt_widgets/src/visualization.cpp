#include "visualization.h"
#include "EGIReg.h"
#include "Sac_IA.h"
#include "optimalReg.h"
#include <Eigen/Geometry> 
#include <Eigen/src/Core/IO.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/registration/ndt.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

#define ICO_X .525731112119133606
#define ICO_Z .850650808352039932

using namespace pcl::visualization;

Visualization::Visualization(QString filename, QString filename_data)
	:filename(filename), filename_data(filename_data)
{
	record = new Record();
	original_model.reset(new PointCloudT);
	original_data.reset(new PointCloudT);
	final_data.reset(new PointCloudT);
	cloud.reset(new PointCloudT);
	cloud_out.reset(new PointCloudT);
	cloud_normals.reset(new PointCloudN);
	cloud_data.reset(new PointCloudT);
	viewer.reset(new pcl::visualization::PCLVisualizer("preview", false));

	//kdtreeFlag = false;
	//centroidFlag = false;

	feature_id = 0;

	search_radius = 0.3;
	normal_level = 100;
	normal_scale = 0.0;
	rotation = { 0.0, 0.0, 0.0 };

	data_with_normals.reset(new PointCloudPN);
	model_with_normals.reset(new PointCloudPN);
	datum_data_with_normals.reset(new PointCloudPN);
	datum_model_with_normals.reset(new PointCloudPN);
	surface_data_with_normals.reset(new PointCloudPN);
	surface_model_with_normals.reset(new PointCloudPN);

	datum_data.reset(new PointCloudT);
	datum_model.reset(new PointCloudT);
	surface_data.reset(new PointCloudT);
	surface_model.reset(new PointCloudT);
	normals_model.reset(new PointCloudN);
	normals_data.reset(new PointCloudN);
	//normals_datum_model.reset(new PointCloudN);
	//normals_datum_data.reset(new PointCloudN);
	normals_surface_model.reset(new PointCloudN);
	normals_surface_data.reset(new PointCloudN);
}


Visualization::~Visualization()
{
}

void
Visualization::preview(QString filename, QString filename_data)
{
	std::string file_name = filename.toStdString();
	std::string file_name_data = filename_data.toStdString();

	pcl_time.tic();
	record->statusUpdate("Loading model...");

	if (filename.endsWith(".pcd"))
	{
		if (!pcl::io::loadPCDFile(file_name, *cloud))
		{
			record->statusUpdate("Cannot open pcd file!");
		}
	}
	else if (filename.endsWith(".ply"))
	{
		if (!pcl::io::loadPLYFile(file_name, *cloud))
		{
			record->statusUpdate("Cannot open ply file!");
		}
	}
	//VoxelGrid Filter Downsampling
	pcl::VoxelGrid<PointT> grid;
	const float leaf = 0.1f;
	grid.setLeafSize(leaf, leaf, leaf);
	grid.setInputCloud(cloud);
	grid.filter(*original_model);
	//*original_model = *cloud;
	if (filename_data.endsWith(".pcd"))
	{
		if (!pcl::io::loadPCDFile(file_name_data, *cloud_data))
		{
			record->statusUpdate("Cannot open pcd file!");
		}
	}
	else if (filename_data.endsWith(".ply"))
	{
		if (!pcl::io::loadPLYFile(file_name_data, *cloud_data))
		{
			record->statusUpdate("Cannot open ply file!");
		}
	}
	grid.setInputCloud(cloud_data);
	grid.filter(*original_data);
	//*original_data = *cloud_data;

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1ds(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2ds(new pcl::PointCloud<pcl::PointXYZ>);

	//pcl::ConvexHull<pcl::PointXYZ> chull;
	//chull.setInputCloud(original_model);
	//chull.reconstruct(*cloud1ds);
	//chull.setInputCloud(original_data);
	//chull.reconstruct(*cloud2ds);
	//original_model = cloud1ds;
	//original_data = cloud2ds;

	//display number of points
	record->info = QString("model size: "
		+ QString::number(cloud->points.size())
		+ "data size: "
		+ QString::number(cloud_data->points.size())
		+ "; duration: "
		+ QString::number(pcl_time.toc())
		+ "ms;");
	record->infoRec(record->info);
	record->statusUpdate(record->info);

	viewer.reset(new pcl::visualization::PCLVisualizer("preview", true));
	// Create two verticaly separated viewports
	int v1(1);
	int v2(2);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);

	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> green(original_model, 20, 180, 20);
	viewer->addPointCloud(original_model, green, "model cloud", v1);

	pcl::visualization::PointCloudColorHandlerCustom<PointT> red(original_data, 180, 20, 20);
	viewer->addPointCloud(original_data, red, "data cloud", v2);

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "model cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "data cloud");
	viewer->addCoordinateSystem(10.0);
	viewer->initCameraParameters();


	while (!viewer->wasStopped())
	{
		switch (feature_id)
		{
		case 1:
			computeKdtree();
			break;
		case 2:
			computeCentroid();
			break;
		case 3:
			computeNormals();
			break;
		case 4:
			computeFPFH();
			break;
		case 5:
			filter_n_downsampling();
			break;
		case 6:
			computeEGI();
			break;
		case 7:
			registration();
			break;
		case 8:
			initialAlignment();
			break;
		case 9:
			sacSegment();
		case 10:
			optimalReg();
		default:
			break;
		}
		feature_id = 0;

		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

void
Visualization::OnStarted()
{
	this->preview(filename, filename_data);
	emit finished();
}

void
Visualization::feature_id_slot(int feature)
{
	feature_id = feature;
}

bool
Visualization::filter_n_downsampling()
{
	////VoxelGrid Filter Downsampling
	//record->statusUpdate("Downsampling...");
	//pcl::VoxelGrid<PointT> grid;
	//const float leaf = 0.005f;
	//grid.setLeafSize(leaf, leaf, leaf);
	//grid.setInputCloud(cloud);
	//grid.filter(*cloud);

	//StatisticalOutlierRemoval Filter
	record->statusUpdate("Remove Statistical Outlier...");
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(normal_level);
	sor.setStddevMulThresh(search_radius);
	sor.filter(*cloud);
	record->statusUpdate("Done");

	////RadiusOutlierRemoval Filter
	//record->statusUpdate("Remove Radius Outlier...");
	//pcl::RadiusOutlierRemoval<PointT> ror;
	//ror.setInputCloud(cloud);
	//ror.setRadiusSearch(search_radius);
	//ror.setMinNeighborsInRadius(normal_level);
	//ror.filter(*cloud);
	//record->statusUpdate("Done");

	pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb(cloud, 20, 180, 20);
	viewer->updatePointCloud(cloud, rgb, "model cloud");

	return true;
}

bool
Visualization::computeKdtree()
{
	record->statusUpdate("compute Kdtree...");
	pcl_time.tic();
	//kd-tree tutorials
	srand(time(NULL));
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(cloud);
	int size = cloud->points.size();
	PointT searchPoint;
	std::vector<int> indices;
	std::vector<float> meanDistance;
	//searchPoint = cloud->points.at(cloud->points.size() * rand() / (RAND_MAX + 1));

	//viewer->addSphere(searchPoint, 1, 1, 0.2, 0.2, "search Point");

	int k = normal_level;
	std::vector<int> pointIdxNKNSearch(k);
	std::vector<float> pointNKNSquaredDistance(k);
	//std::stringstream ss;
	//ss  << "k nearest neighbor search at (" << searchPoint.x
	//	<< " " << searchPoint.y
	//	<< " " << searchPoint.z
	//	<< ") with K = " << k << std::endl;

	//record->infoRec(QString::fromStdString(ss.str()));

	//if (kdtree.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
	//{
	//	for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
	//	{
	//		std::stringstream ss;

	//		ss  << " " << cloud->points[pointIdxNKNSearch[i]].x
	//			<< " " << cloud->points[pointIdxNKNSearch[i]].y
	//			<< " " << cloud->points[pointIdxNKNSearch[i]].z
	//			<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
	//		record->infoRec(QString::fromStdString(ss.str()));
	//	}
	//}
	record->progressBarUpdate(0);
	for (int i = 0; i < size; i++)
	{
		searchPoint = cloud->points.at(i);
		if (kdtree.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			float distance = 0.0;
			for (int j = 0; j < k; j++)
			{
				distance = distance + pointNKNSquaredDistance.at(j);
			}
			meanDistance.push_back(distance / k);
			indices.push_back(i);
			record->progressBarUpdate(int(100 * i / size));
		}
	}
	QFile file("kdtree_data.txt");
	if (!file.open(QIODevice::Truncate | QFile::ReadWrite | QFile::Text))
		emit record->statusUpdate("can't open kdtree_data.txt");
	QTextStream in(&file);
	for (int i = 0; i < indices.size(); i++)
	{
		in << indices.at(i) << " " << meanDistance.at(i) << endl;
	}
	file.close();
	record->progressBarUpdate(100);
	record->statusUpdate("completed in " + QString::number(pcl_time.toc()) + "ms;");

	return true;
}

bool
Visualization::computeCentroid()
{
	record->statusUpdate("compute Centroid...");
	pcl_time.tic();
	Eigen::Vector4f xyz_centroid;
	pcl::compute3DCentroid(*cloud, xyz_centroid);
	//std::stringstream ss;
	//for (size_t i = 0; i < 4; i++)
	//{
	//	ss << " " << xyz_centroid(i);
	//}
	//ss << std::endl;
	//record->infoRec(QString::fromStdString(ss.str()));

	//transform point cloud
	//Eigen::Transform<float, 3, Eigen::Affine> transformation_matrix;
	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
	transformation_matrix(0, 0) = 1;
	transformation_matrix(1, 1) = 1;
	transformation_matrix(2, 2) = 1;
	transformation_matrix(0, 3) = -xyz_centroid(0);
	transformation_matrix(1, 3) = -xyz_centroid(1);
	transformation_matrix(2, 3) = -xyz_centroid(2);
	transformation_matrix(3, 3) = 1;
	pcl::transformPointCloud(*cloud, *cloud_out, transformation_matrix, true);
	*cloud = *cloud_out;

	pcl::visualization::PointCloudColorHandlerCustom<PointT> green(cloud_out, 20, 180, 20);
	viewer->updatePointCloud(cloud_out, green, "model cloud");

	pcl::compute3DCentroid(*cloud_data, xyz_centroid);

	record->statusUpdate("completed in " + QString::number(pcl_time.toc()) + "ms;");

	//transform point cloud
	transformation_matrix(0, 3) = -xyz_centroid(0);
	transformation_matrix(1, 3) = -xyz_centroid(1);
	transformation_matrix(2, 3) = -xyz_centroid(2);
	pcl::transformPointCloud(*cloud_data, *cloud_out, transformation_matrix, true);
	*cloud_data = *cloud_out;

	pcl::visualization::PointCloudColorHandlerCustom<PointT> red(cloud_out, 180, 20, 20);
	viewer->updatePointCloud(cloud_out, red, "data cloud");

	viewer->setCameraPosition(-50, 5, 360, 0, 0, 0);

	//pcl::visualization::Camera cam;
	//viewer->getCameraParameters(cam);

	//std::stringstream ss;
	//ss << cam.pos[0] << " "
	//	<< cam.pos[1] << " "
	//	<< cam.pos[2] << " "
	//	<< cam.view[0] << " "
	//	<< cam.view[1] << " "
	//	<< cam.view[2];
	//record->statusUpdate(QString::fromStdString(ss.str()));

	return true;
}

bool
Visualization::computeNormals()
{
	record->statusUpdate("compute Normals...");
	pcl_time.tic();
	// Create the normal estimation class, and pass the input dataset to it
	//pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);

	// Output datasets
	//PointCloudN::Ptr cloud_normals(new PointCloudN);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch(search_radius);
	//ne.setViewPoint(0, 0, 0);
	// Compute the features
	ne.compute(*cloud_normals);

	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals, normal_level, normal_scale, "model normals", 1);
	// Compute the 3x3 covariance matrix
	//Eigen::Matrix3f covariance_matrix;
	//pcl::computeCovarianceMatrix(cloud, xyz_centroid, covariance_matrix);

	PointCloudN::Ptr cloud_data_normals(new PointCloudN);
	ne.setInputCloud(cloud_data);
	ne.compute(*cloud_data_normals);
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_data, cloud_data_normals, normal_level, normal_scale, "data normals", 2);

	//QFile file("normals_data.txt");
	//if (!file.open(QIODevice::Truncate | QFile::ReadWrite | QFile::Text))
	//	emit record->statusUpdate("can't open normals_data.txt");
	//QTextStream in(&file);
	//int size = cloud_normals->size();
	//for (int i = 0; i < size; i++)
	//{
	//	in << cloud_normals->at(i).normal_x << " "
	//		<< cloud_normals->at(i).normal_y << " "
	//		<< cloud_normals->at(i).normal_z << " "
	//		<< cloud_normals->at(i).curvature << endl;
	//	record->progressBarUpdate(int(100 * i / size));
	//}
	//file.close();
	//record->progressBarUpdate(100);
	record->statusUpdate("completed in " + QString::number(pcl_time.toc()) + "ms;");

	return true;
}

bool
Visualization::computeFPFH()
{
	record->statusUpdate("compute FPFH...");
	pcl_time.tic();
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>fpfh;

	fpfh.setInputCloud(cloud);
	fpfh.setInputNormals(cloud_normals);

	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	fpfh.setSearchMethod(tree);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());

	fpfh.setRadiusSearch(search_radius);
	fpfh.compute(*fpfhs);
	record->statusUpdate("completed in " + QString::number(pcl_time.toc()) + "ms;");
	//pcl::visualization::PCLHistogramVisualizer hist_viewer;
	//hist_viewer.addFeatureHistogram(*fpfhs, 33);

	// Plotter object.
	pcl::visualization::PCLPlotter plotter;
	// We need to set the size of the descriptor beforehand.
	plotter.addFeatureHistogram(*fpfhs, 33);

	plotter.plot();

	return true;
}

bool
Visualization::computeEGI()
{
	/********************************* Visualization *************************************/
	viewer->removeAllPointClouds();
	int v1(1);
	int v2(2);
	int v3(3);
	int v4(4);
	viewer->createViewPort(0.0, 0.0, 0.5, 0.5, v3);
	viewer->createViewPort(0.5, 0.0, 1.0, 0.5, v4);
	viewer->createViewPort(0.0, 0.5, 0.5, 1.0, v1);
	viewer->createViewPort(0.5, 0.5, 1.0, 1.0, v2);

	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
	viewer->setBackgroundColor(0.1, 0.1, 0.1, v3);
	viewer->setBackgroundColor(0, 0, 0, v4);

	pcl::visualization::PointCloudColorHandlerCustom<PointT> green(original_model, 20, 180, 20);
	viewer->addPointCloud(original_model, green, "model cloud", v1);

	pcl::visualization::PointCloudColorHandlerCustom<PointT> red(original_data, 180, 20, 20);
	viewer->addPointCloud(original_data, red, "data cloud", v2);

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "model cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "data cloud");
	viewer->addCoordinateSystem(10.0, v1);
	viewer->addCoordinateSystem(10.0, v2);
	viewer->initCameraParameters();
	computeCentroid();

	/********************************* Preprogress *************************************/
	Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();;
	EGIReg *nsReg = new EGIReg();
	nsReg->setModel(original_model);
	nsReg->setData(original_data);
	nsReg->ns_visualization();
	nsReg->translationEstimate();

	viewer->addPointCloud(nsReg->model_normal_sphere, green, "model", v3);
	viewer->addPointCloud(nsReg->data_normal_sphere, red, "data", v4);

	nsReg->search(tf);
	//double corr = nsReg->computeCorrelation(0.5, 0.5, 0.5);

	//viewer->addText(QString::number(corr, 'g', 6).toStdString(), 0.5, 0.5, "debug", 0);
	pcl::transformPointCloud(*cloud_data, *cloud_out, tf, true);
	*cloud_data = *cloud_out;
	viewer->updatePointCloud(cloud_data, red, "data cloud");

	pcl::transformPointCloud(*nsReg->data_normal_sphere, *cloud_out, tf, true);
	*nsReg->data_normal_sphere = *cloud_out;
	viewer->updatePointCloud(nsReg->data_normal_sphere, red, "data");

	qDebug("Rotation matrix :\n");
	qDebug("    | %6.3f %6.3f %6.3f | \n", tf(0, 0), tf(0, 1), tf(0, 2));
	qDebug("R = | %6.3f %6.3f %6.3f | \n", tf(1, 0), tf(1, 1), tf(1, 2));
	qDebug("    | %6.3f %6.3f %6.3f | \n", tf(2, 0), tf(2, 1), tf(2, 2));
	qDebug("Translation vector :\n");
	qDebug("t = < %6.3f, %6.3f, %6.3f >\n\n", tf(0, 3), tf(1, 3), tf(2, 3));


	//if (cloud_normals->size() == 0)
	//{
	//	record->statusUpdate("no cloud normals!");
	//	return false;
	//}
	//int size = cloud_normals->size();

	//PointCloudT::Ptr cloud_EGI;
	//PointCloudN::Ptr normals_EGI;

	//cloud_EGI.reset(new PointCloudT);
	//normals_EGI.reset(new PointCloudN);

	//boost::shared_ptr<pcl::visualization::PCLVisualizer> EGI_viewer;
	//EGI_viewer.reset(new pcl::visualization::PCLVisualizer("EGI", true));

	//cloud_EGI->resize(size);
	//record->statusUpdate(QString::number(size));
	//for (int i = 0; i < size; ++i)
	//{
	//	cloud_EGI->points[i].x = cloud_normals->at(i).normal_x;
	//	cloud_EGI->points[i].y = cloud_normals->at(i).normal_y;
	//	cloud_EGI->points[i].z = cloud_normals->at(i).normal_z;
	//	record->progressBarUpdate(int(100 * i / size));
	//}



	///*************************Create Icosahedron****************************/
	//static float vdata[12][3] = {
	//	{ -ICO_X, 0.0, ICO_Z},{ ICO_X, 0.0, ICO_Z },{ -ICO_X, 0.0, -ICO_Z },{ ICO_X, 0.0, -ICO_Z },
	//	{ 0.0, ICO_Z, ICO_X},{ 0.0, ICO_Z, -ICO_X },{ 0.0, -ICO_Z, ICO_X },{ 0.0, -ICO_Z, -ICO_X },
	//	{ ICO_Z, ICO_X, 0.0},{ -ICO_Z, ICO_X, 0.0 },{ ICO_Z, -ICO_X, 0.0 },{ -ICO_Z, -ICO_X, 0.0 }
	//};

	//static unsigned int tindices[20][3] = {
	//	{ 1,4,0 },{ 4,9,0 },{ 4,5,9 },{ 8,5,4 },{ 1,8,4 },
	//	{ 1,10,8 },{ 10,3,8 },{ 8,3,5 },{ 3,2,5 },{ 3,7,2 },
	//	{ 3,10,7 },{ 10,6,7 },{ 6,11,7 },{ 6,0,11 },{ 6,1,0 },
	//	{ 10,1,6 },{ 11,0,9 },{ 2,11,9 },{ 5,2,9 },{ 11,2,7 }
	//};

	////Mapping normals to Icosahedron
	//QFile file("EGI_intensity_data.txt");
	//if (!file.open(QIODevice::Truncate | QFile::ReadWrite | QFile::Text))
	//	emit record->statusUpdate("can't open EGI_intensity_data.txt");
	//QTextStream in(&file);

	//unsigned int intensity_EGI_ICO[20] = { 0 };

	//for (int i = 0; i < 20; ++i)
	//{
	//	Eigen::Vector3f lamda;
	//	Eigen::Vector3f normal_vector;
	//	Eigen::Matrix3f indice_matrix;
	//	indice_matrix << vdata[tindices[i][0]][0], vdata[tindices[i][1]][0], vdata[tindices[i][2]][0],
	//		vdata[tindices[i][0]][1], vdata[tindices[i][1]][1], vdata[tindices[i][2]][1],
	//		vdata[tindices[i][0]][2], vdata[tindices[i][1]][2], vdata[tindices[i][2]][2];
	//	for (int j = 0; j < size; ++j)
	//	{
	//		normal_vector << cloud_EGI->points[j].x, cloud_EGI->points[j].y, cloud_EGI->points[j].z;
	//		lamda = indice_matrix.inverse() * normal_vector;
	//		if ((lamda[0] > 0) && (lamda[1] > 0) && (lamda[2] > 0))
	//		{
	//			intensity_EGI_ICO[i]++;

	//			//in << lamda[0] << " " << lamda[1] << " " << lamda[2] << endl;
	//			record->progressBarUpdate(int(100 * (i * size + j) / (20 * size)));
	//		}
	//	}
	//	in << intensity_EGI_ICO[i] << endl;
	//}
	//file.close();

	//record->progressBarUpdate(100);

	//Extend Icosahedron




	//pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	//ne.setInputCloud(cloud_EGI);

	//// Create an empty kdtree representation, and pass it to the normal estimation object.
	//// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	//ne.setSearchMethod(tree);

	//// Output datasets
	////PointCloudN::Ptr cloud_normals(new PointCloudN);

	//// Use all neighbors in a sphere of radius 3cm
	//ne.setRadiusSearch(0.03);
	//ne.setViewPoint(3, 3, 3);
	//// Compute the features
	//ne.compute(*normals_EGI);

	//EGI_viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_EGI, normals_EGI, 100, 0.2);

	//EGI_viewer->setBackgroundColor(0, 0, 0);
	//pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb(cloud_EGI, 220, 220, 220);
	//EGI_viewer->addPointCloud(cloud_EGI, rgb, "EGI cloud");
	//EGI_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "EGI cloud");
	//EGI_viewer->addCoordinateSystem(1.0);
	//EGI_viewer->initCameraParameters();

	//while (!EGI_viewer->wasStopped())
	//{
	//	EGI_viewer->spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//}
	//record->progressBarUpdate(100);

	return true;
}

bool
Visualization::registration()
{
	viewer->removeAllPointClouds();
	int v1(1);
	int v2(2);
	int v3(3);
	int v4(4);
	viewer->createViewPort(0.0, 0.0, 0.5, 0.5, v3);
	viewer->createViewPort(0.5, 0.0, 1.0, 0.5, v4);
	viewer->createViewPort(0.0, 0.5, 0.5, 1.0, v1);
	viewer->createViewPort(0.5, 0.5, 1.0, 1.0, v2);

	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
	viewer->setBackgroundColor(0.1, 0.1, 0.1, v3);
	viewer->setBackgroundColor(0, 0, 0, v4);

	pcl::visualization::PointCloudColorHandlerCustom<PointT> green(original_model, 20, 180, 20);
	viewer->addPointCloud(original_model, green, "model cloud", v1);

	pcl::visualization::PointCloudColorHandlerCustom<PointT> red(original_data, 180, 20, 20);
	viewer->addPointCloud(original_data, red, "data cloud", v2);

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "model cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "data cloud");
	viewer->addCoordinateSystem(10.0, v1);
	viewer->addCoordinateSystem(10.0, v2);
	viewer->initCameraParameters();
	computeCentroid();

	EGIReg *nsReg = new EGIReg();
	nsReg->setModel(original_model);
	nsReg->setData(original_data);
	nsReg->ns_visualization();
	nsReg->translationEstimate();

	Eigen::AngleAxisd rollAngle(rotation.at(0), Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd yawAngle(rotation.at(1), Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd pitchAngle(rotation.at(2), Eigen::Vector3d::UnitX());
	Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
	Eigen::Matrix3d rotationMatrix = q.matrix();

	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
	Eigen::Affine3d r(rotationMatrix);
	Eigen::Affine3d t(Eigen::Translation3d(0.0, 0.0, 0.0));
	transformation_matrix = (t * r).matrix();

	viewer->addPointCloud(nsReg->model_normal_sphere, green, "model", v3);
	viewer->addPointCloud(nsReg->data_normal_sphere, red, "data", v4);

	pcl::transformPointCloud(*cloud_data, *cloud_out, transformation_matrix, true);
	*cloud_data = *cloud_out;
	viewer->updatePointCloud(cloud_data, red, "data cloud");

	pcl::transformPointCloud(*nsReg->data_normal_sphere, *cloud_out, transformation_matrix, true);
	*nsReg->data_normal_sphere = *cloud_out;
	viewer->updatePointCloud(nsReg->data_normal_sphere, red, "data");

	return true;
}

bool
Visualization::initialAlignment()
{
	//===================Sac-IA==========================//
	/*
	const double FILTER_LIMIT = 1000.0;
	const int MAX_SACIA_ITERATIONS = 2000;

	const float VOXEL_GRID_SIZE = 3;
	const double NORMALS_RADIUS = 20;
	const double FEATURES_RADIUS = 50;
	const double SAC_MAX_CORRESPONDENCE_DIST = 2000;
	const double SAC_MIN_CORRESPONDENCE_DIST = 3;
	Sac_IA *ia = new Sac_IA();

	pcl_time.tic();
	// downsample the clouds
	record->statusUpdate("downsample the clouds...");
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1ds(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2ds(new pcl::PointCloud<pcl::PointXYZ>);
	
	ia->voxelFilter(original_model, cloud1ds, VOXEL_GRID_SIZE);
	ia->voxelFilter(original_data, cloud2ds, VOXEL_GRID_SIZE);
		
	record->progressBarUpdate(10);
		
	// compute normals
	record->statusUpdate("compute normals...");
	pcl::PointCloud<pcl::Normal>::Ptr normals1 = ia->getNormals(cloud1ds, NORMALS_RADIUS);
	pcl::PointCloud<pcl::Normal>::Ptr normals2 = ia->getNormals(cloud2ds, NORMALS_RADIUS);

	record->progressBarUpdate(25);

	// compute local features
	record->statusUpdate("compute local features...");
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr features1 = ia->getFeatures(cloud1ds, normals1, FEATURES_RADIUS);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr features2 = ia->getFeatures(cloud2ds, normals2, FEATURES_RADIUS);

	record->progressBarUpdate(50);
	
	// initial alignment
	record->statusUpdate("alignment...");
	auto sac_ia = ia->align(cloud1ds, cloud2ds, features1, features2,
		MAX_SACIA_ITERATIONS, SAC_MIN_CORRESPONDENCE_DIST, SAC_MAX_CORRESPONDENCE_DIST);

	record->progressBarUpdate(90);	
	init_transform = sac_ia.getFinalTransformation();
	*/
	Eigen::Matrix4f transform;
	//transform <<
	//	 0.91302,  0.081678,  0.399655, -77.4954,
	//	 0.387609, 0.131557, -0.912388, 193.777,
	//	-0.127099, 0.987938,  0.0884551, 79.7795,
	//	 0,		   0,		  0,		  1; 
	transform <<
		-0.858067, -0.507153, 0.080725, 109.273,
		-0.511004, 0.858815, -0.0362297, 13.2583,
		-0.0509539, -0.0723383, -0.996078, 396.565,
		0, 0, 0, 1;
	init_transform = transform;

	PointCloudT::Ptr trans_data(new PointCloudT);
	pcl::transformPointCloud(*original_data, *trans_data, init_transform);
	
	record->progressBarUpdate(100);
	//===================RecordData==========================//
	/*
	record->statusUpdate("completed in " + QString::number(pcl_time.toc()) + "ms;");
	std::stringstream trans;
	Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
	trans << init_transform.format(OctaveFmt);
	record->infoRec(QString::fromStdString(trans.str()));
	*/
	//===================Visualization==========================//
	int v1(0), v2(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0.0, 0.0, 0.0, v1);
	viewer->addText("Before Alignment", 10, 10, "v1 text", v1);
	PointCloudColorHandlerCustom<PointXYZ> green(original_model, 0, 255, 0);
	PointCloudColorHandlerCustom<PointXYZ> red(original_data, 255, 0, 0);
	viewer->addPointCloud(original_model, green, "v1_target", v1);
	viewer->addPointCloud(original_data, red, "v1_sourse", v1);

	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.0, 0.0, 0.0, v2);
	viewer->addText("After Alignment", 10, 10, "v2 text", v2);
	PointCloudColorHandlerCustom<PointXYZ> green2(original_model, 0, 255, 0);
	PointCloudColorHandlerCustom<PointXYZ> red2(trans_data, 255, 0, 0);
	viewer->addPointCloud(original_model, green2, "v2_target", v2);
	viewer->addPointCloud(trans_data, red2, "v2_sourse", v2);

	viewer->spinOnce();

	return true;
}

void
Visualization::computeDatumCoefficients(PointCloud<PointNormal>::Ptr cloud, PointCloud<PointXYZ>::Ptr datum_plane, pcl::ModelCoefficients::Ptr coefficients)
{
	PointCloud<PointNormal>::Ptr filtered_cloud(new PointCloud<PointNormal>);
	// Create the filtering object
	pcl::PassThrough<pcl::PointNormal> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-10, 10);
	
	pass.filter(*filtered_cloud);
	//pass.setFilterLimitsNegative (true);


	//================segmentation=========================//
	// segmentation
	//pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	PointCloudN::Ptr filtered_normals(new PointCloudN);
	PointCloudT::Ptr filtered_points(new PointCloudT);
	for (size_t i = 0; i < filtered_cloud->points.size(); ++i)
	{
		const pcl::PointNormal &mls_pt = filtered_cloud->points[i];
		pcl::PointXYZ pt(mls_pt.x, mls_pt.y, mls_pt.z);
		pcl::Normal pn(mls_pt.normal_x, mls_pt.normal_y, mls_pt.normal_z);
		filtered_points->push_back(pt);
		filtered_normals->push_back(pn);
	}
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> sac;
	sac.setInputCloud(filtered_points);
	sac.setInputNormals(filtered_normals);
	sac.setMethodType(pcl::SAC_RANSAC);
	sac.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
	//sac.setNormalDistanceWeight(0.1);
	sac.setAxis(Eigen::Vector3f(0, 0, 1));
	sac.setEpsAngle(0.1);
	sac.setDistanceThreshold(0.1);
	sac.setMaxIterations(100);
	sac.setProbability(0.95);

	sac.segment(*inliers, *coefficients);

	// extract the certain field	 
	pcl::ExtractIndices<pcl::PointXYZ> ei;
	ei.setIndices(inliers);
	ei.setInputCloud(filtered_points);
	ei.setNegative(true);
	ei.filter(*datum_plane);
	//pcl::ExtractIndices<pcl::PointXYZ> ei2;
	//ei2.setIndices(inliers);
	//ei2.setInputCloud(cloud);
	//ei2.setNegative(false);
	//ei2.filter(*surface);


	return;
};

double
Visualization::computeDatumError(PointCloud<PointXYZ>::Ptr datum_model, PointCloud<PointXYZ>::Ptr datum_data, Normal &normal)
{
	double datum_error = 0.0;
	float resolution = 128.0f;
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
	octree.setInputCloud(datum_data);
	octree.addPointsFromInputCloud();
	int K = 2;
	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;
	int scale = datum_model->size();
	for (int i = 0; i < scale; i++)
	{
		octree.nearestKSearch(datum_model->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance);
		Eigen::Vector3d yi(datum_data->points[pointIdxNKNSearch[0]].x, datum_data->points[pointIdxNKNSearch[0]].y, datum_data->points[pointIdxNKNSearch[0]].z);
		Eigen::Vector3d xi(datum_model ->points[i].x, datum_model->points[i].y, datum_model->points[i].z);
		Eigen::Vector3d w(normal.normal_x, normal.normal_y, normal.normal_z);
		Eigen::Vector3d v = yi - xi;
		double e = v.dot(w);
		datum_error = datum_error + e*e;
	}
	return datum_error;
};

double
Visualization::computeDatumAngle(pcl::ModelCoefficients::Ptr coeff_model, pcl::ModelCoefficients::Ptr coeff_data)
{
	return 0.0;
};

double 
Visualization::enveloped(PointCloudT::Ptr surface_model, PointCloudN::Ptr normals, PointCloudT::Ptr surface_data, std::vector<double> dist)
{
	int enveloped_counts = 0;

	float resolution = 128.0f;
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
	octree.setInputCloud(surface_data);
	octree.addPointsFromInputCloud();
	int K = 1;
	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;
	int scale = surface_model->size();
	for (int i = 0; i < scale; i++)
	{
		octree.nearestKSearch(surface_model->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance);
		Eigen::Vector3d yi(surface_data->points[pointIdxNKNSearch[0]].x, surface_data->points[pointIdxNKNSearch[0]].y, surface_data->points[pointIdxNKNSearch[0]].z);
		Eigen::Vector3d xi(surface_model->points[i].x, surface_model->points[i].y, surface_model->points[i].z);
		Eigen::Vector3d w(normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z);
		Eigen::Vector3d v = yi - xi;
		//qDebug("v & w: %f, %f, %f, %f, %f, %f, %f;", v[0], v[1], v[2], w[0], w[1], w[2], v.dot(w));
		dist.push_back(v.dot(w));
		if (v.dot(w) > 0)
			enveloped_counts++;		
	}
	double enveloped_rate = (double)enveloped_counts / (double)scale;
	qDebug("enveloped rate: %f", enveloped_rate);
	return enveloped_rate;
	//if (enveloped_rate > 0.75)
	//	return true;
	//else
	//	return false;
};

double
Visualization::computeSurfaceVariance(std::vector<double> dist)
{
	double var = 0.0;
	double sum = 0.0;
	for (int i = 0; i < dist.size(); i++)
	{
		sum = sum + dist.at(i);
	}

	for (int i = 0; i < dist.size(); i++)
	{
		var = var + sqrt(dist.at(i) - sum / dist.size());
	}
	var = var / dist.size();
	return var;
};

double
Visualization::computeFitness(Eigen::Matrix4d &transformation)
{
	PointCloud<PointNormal>::Ptr transformed_data(new PointCloud<PointNormal>);
	PointCloud<PointXYZ>::Ptr transformed_surface(new PointCloud<PointXYZ>);
	PointCloud<PointNormal>::Ptr transformed_surface_with_normals(new PointCloud<PointNormal>);
	pcl::transformPointCloudWithNormals(*data_with_normals, *transformed_data, transformation);
	pcl::transformPointCloudWithNormals(*surface_data_with_normals, *transformed_surface_with_normals, transformation);
	for (size_t i = 0; i < transformed_surface_with_normals->points.size(); ++i)
	{
		const pcl::PointNormal &mls_pt = transformed_surface_with_normals->points[i];
		pcl::PointXYZ pt(mls_pt.x, mls_pt.y, mls_pt.z);
		transformed_surface->push_back(pt);
	}

	pcl::ModelCoefficients::Ptr coeff_data(new pcl::ModelCoefficients);
	pcl::ModelCoefficients::Ptr coeff_model(new pcl::ModelCoefficients);
		
	//qDebug("datum plane of data: %f, %f, %f, %f\n", coeff_data->values[0], coeff_data->values[1], coeff_data->values[2], coeff_data->values[3]);
	//qDebug("datum plane of model: %f, %f, %f, %f\n", coeff_model->values[0], coeff_model->values[1], coeff_model->values[2], coeff_model->values[3]);

	double alpha, beta;
	double datum_error, dist_variance, enveloped_rate;
	double fitness;
	std::vector<double> dist;

	alpha = 0.2;
	beta = 0.8;

	OptimalRegistration optReg;
	optReg.setModelCloud(surface_model);
	optReg.setDataCloud(transformed_surface);
	bool _enveloped = optReg.estimateEveloped(0.7);
	qDebug("enveloped_rate: %f.", optReg.getEnvelopedRate());

	//enveloped_rate = enveloped(surface_model, normals_surface, transformed_surface, dist);
	//
	//return enveloped_rate;
	
	if (_enveloped)
	{
		//===============NormalEstimation========================//
		//pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
		//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		//ne.setSearchMethod(tree);
		//ne.setRadiusSearch(30);
		//ne.setInputCloud(transformed_data);
		//ne.compute(*normals_data);
		//===============DatumEstimation========================//
		computeDatumCoefficients(model_with_normals, datum_model, coeff_model);
		computeDatumCoefficients(transformed_data, datum_data, coeff_data);
		//===============ComputefFitness========================//
		pcl::Normal datum_normal;
		double mA, mB, mC;
		mA = coeff_model->values[0];
		mB = coeff_model->values[1];
		mC = coeff_model->values[2];
		datum_normal.normal_x = mA / sqrt(mA*mA + mB*mB + mC*mC);
		datum_normal.normal_y = mB / sqrt(mA*mA + mB*mB + mC*mC);
		datum_normal.normal_z = mC / sqrt(mA*mA + mB*mB + mC*mC);		
		datum_error = computeDatumError(datum_model, datum_data, datum_normal);
		dist_variance = computeSurfaceVariance(dist);

		fitness = 1 / (1 + alpha * exp(datum_error) + beta * exp(dist_variance));
		qDebug("Daturm Error: %f. Distance Var: %f. Fitness: %f.", datum_error, dist_variance, fitness);
		return fitness;
	}
	else
	{
		return 0.0;
	}	
};

//GA part
double
Visualization::Curve(Chromo2 input)
{
	double omega, fai, kappa;
	double a, b, c;
	omega = input.vecGenome[0];
	fai = input.vecGenome[1];
	kappa = input.vecGenome[2];
	a = input.vecGenome[3];
	b = input.vecGenome[4];
	c = input.vecGenome[5];

	Eigen::AngleAxisd rollAngle(omega, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd yawAngle(fai, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd pitchAngle(kappa, Eigen::Vector3d::UnitX());

	Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

	Eigen::Matrix3d rotationMatrix = q.matrix();

	Eigen::Affine3d r(rotationMatrix);
	Eigen::Affine3d t(Eigen::Translation3d(a , b , c ));
	Eigen::Matrix4d transformation = (t * r).matrix();

	double y = computeFitness(transformation);
	//qDebug("fitness: %f", y);
	return y;
}

double
Visualization::Random()
{
	double random;
	random = (1.0*rand()) / (RAND_MAX + 1);
	return random;
}

void
Visualization::Reset()
{
	generationCount = 0;
	//popOperation.vecPop.clear();
}

void
Visualization::Init(int popsize, double mutationrate, double crossoverrate, int generationmax,
	double maxstep, double leftmax, double rightmax)
{
	popOperation.popSize = popsize;
	popOperation.mutationRate = mutationrate;
	popOperation.crossoverRate = crossoverrate;
	popOperation.generationMax = generationmax;
	popOperation.maxStep = maxstep;
	popOperation.leftMax = leftmax;
	popOperation.rightMax = rightmax;
	double k = 0.0;
	//初始化种群中染色体的基因--随机方式
	for (int i = 0; i < popOperation.popSize; i++)
	{
		for (unsigned int j = 0; j < popOperation.vecPop[i].chromoLength; j++)
		{
			k = Random();
			popOperation.vecPop[i].vecGenome[j] = k*(rightmax - leftmax) + leftmax;
		}
	}
}

void
Visualization::CalculateRate()
{
	double Ydata = 0.0, AllRate = 0.0, Yrate = 0.0;
	double maxRate = 0.0, minRate = 0.0, maxData = 0.0;

	Ydata = Curve(popOperation.vecPop[0]);
	Yrate = Ydata;
	popOperation.vecPop[0].fitness = Yrate;
	maxRate = Yrate;
	minRate = Yrate;
	AllRate = Yrate;
	maxData = Ydata;
	for (int i = 1; i < popOperation.popSize; i++)
	{
		Ydata = Curve(popOperation.vecPop[i]);
		Yrate = Ydata;
		popOperation.vecPop[i].fitness = Yrate;
		AllRate = AllRate + Yrate;
		if (maxRate < Yrate)
		{
			maxRate = Yrate;
			maxData = Ydata;
			popOperation.fitnessChromo = popOperation.vecPop[i];
		}
		if (minRate > Yrate)
			minRate = Yrate;
	}
#if false
	double maxY = 0.0, absMax = 0.0, Max = 0.0, AllMax = 0.0, test = 0.0;
	double maxRate = 0.0, minRate = 0.0;
	for (int i = 0; i < popOperation.popSize; i++)//解码
	{
		test = Curve(popOperation.vecPop[i]);
		Max = Max + test;
		if (test < 0) test = -test;
		absMax = absMax + test;
	}
	AllMax = Max + popOperation.popSize * absMax;
	test = Curve(popOperation.vecPop[0]);
	popOperation.vecPop[0].fitness = (absMax + test) / (AllMax);
	maxRate = popOperation.vecPop[0].fitness;
	minRate = popOperation.vecPop[0].fitness;
	for (int i = 1; i < popOperation.popSize; i++)
	{
		test = Curve(popOperation.vecPop[i]);
		popOperation.vecPop[i].fitness = (absMax + test) / (AllMax);
		if (maxRate < popOperation.vecPop[i].fitness)
		{
			maxRate = popOperation.vecPop[i].fitness;
			maxY = Curve(popOperation.vecPop[i]);
		}
		if (minRate > popOperation.vecPop[i].fitness)
			minRate = popOperation.vecPop[i].fitness;
	}
#endif
	popOperation.bestFitness = maxRate;
	popOperation.worstFitness = minRate;
	popOperation.MaxY = maxData;
	popOperation.totalFitness = AllRate;
	Report();
}

Chromo2
Visualization::GenomeRoulette()
{
	double chooseRate = 0.0;
	Chromo2 chooseChromo;
	double randomLimit = 0.0;
	randomLimit = Random()*popOperation.totalFitness;
	for (int i = 0; i < popOperation.popSize; i++)
	{
		chooseRate = chooseRate + popOperation.vecPop[i].fitness;
		if (chooseRate > randomLimit)
		{
			chooseChromo = popOperation.vecPop[i];
			break;
		}
	}
	return chooseChromo;
}

void
Visualization::Mutate(Chromo2 genome)
{
	double mutateData = 0.0;
	for (int i = 0; i < popOperation.popSize; i++)
	{
		for (unsigned int j = 0; j < popOperation.vecPop[i].chromoLength; j++)
		{
			mutateData = popOperation.vecPop[i].vecGenome[j];
			if (Random() < popOperation.mutationRate)
				popOperation.vecPop[i].vecGenome[j] = (Random() - 0.5)*popOperation.maxStep + mutateData;
			if (popOperation.vecPop[i].vecGenome[j] > popOperation.rightMax)
				popOperation.vecPop[i].vecGenome[j] = popOperation.rightMax;
			if (popOperation.vecPop[i].vecGenome[j] < popOperation.leftMax)
				popOperation.vecPop[i].vecGenome[j] = popOperation.leftMax;
		}
	}
}

void
Visualization::Epoch(Population2& newgeneration)
{
	double x = 0, y = 0;
	std::vector<Chromo2> tempGeneration;
	CalculateRate();
	tempGeneration = popOperation.vecPop;
	Chromo2 MotherChromo, FatherChromo, GirlChromo, BoyChromo;
	for (int i = 0; i < popOperation.popSize; i += 2)
	{
		MotherChromo = GenomeRoulette();
		FatherChromo = GenomeRoulette();
		//进行交叉变异
		double crossover = MotherChromo.vecGenome[1];
		MotherChromo.vecGenome[1] = FatherChromo.vecGenome[1];
		FatherChromo.vecGenome[1] = crossover;
		GirlChromo = MotherChromo;
		BoyChromo = FatherChromo;
		//进行基因突变了
		Mutate(GirlChromo);
		Mutate(BoyChromo);
		popOperation.vecPop[i].vecGenome = GirlChromo.vecGenome;
		popOperation.vecPop[i + 1].vecGenome = BoyChromo.vecGenome;
	}
	//newgeneration.vecPop = tempGeneration;
}

void
Visualization::ImplementGa()
{
	for (int i = 0; i < popOperation.generationMax + 1; i++)
	{

		Epoch(popOperation);
		generationCount++;
	}
}

void
Visualization::Report()
{
	qDebug("GenerationCount: %d.", generationCount);
	//qDebug("bestFitness: %f.", popOperation.bestFitness);
	//qDebug("worstFitness: %f.", popOperation.worstFitness);
	qDebug("Max Fitness: %f.", popOperation.MaxY);
}

bool
Visualization::sacSegment()
{
	qDebug("s1");
	pcl::PointCloud<pcl::PointXYZ>::Ptr datum_plane(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudN::Ptr normals(new PointCloudN);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(original_data);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);

	// Output datasets
	//PointCloudN::Ptr cloud_normals(new PointCloudN);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch(1);
	//ne.setViewPoint(0, 0, 0);
	// Compute the features
	ne.compute(*normals);
	qDebug("s2");
	// segmentation
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> sac;
	sac.setInputCloud(original_data);    // cloud_source_filtered 为提取桌子表面 cloud_source 为提取地面
	sac.setInputNormals(normals);
	sac.setMethodType(pcl::SAC_RANSAC);
	sac.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
	//sac.setNormalDistanceWeight(0.1);
	sac.setAxis(Eigen::Vector3f(0, 1, 0));
	sac.setEpsAngle(0.1);
	sac.setDistanceThreshold(0.1);
	sac.setMaxIterations(100);
	sac.setProbability(0.95);

	sac.segment(*inliers, *coefficients);

	// extract the certain field
	pcl::ExtractIndices<pcl::PointXYZ> ei;
	ei.setIndices(inliers);
	ei.setInputCloud(original_data);    // cloud_source_filtered 为提取桌子表面 cloud_source 为提取地面
	ei.filter(*cloud_target);

	// display
	pcl::visualization::PCLVisualizer p;
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(cloud_target, 255, 0, 0);
	p.addPointCloud(cloud_target, tgt_h, "target");
	p.spinOnce();

	pcl::visualization::PCLVisualizer p2;
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(original_data, 0, 255, 0);
	p2.addPointCloud(original_data, src_h, "source");
	p2.spin();


	return true;
}

bool
Visualization::optimalReg()
{
	qDebug("Start Optimizing!");
	PointCloudT::Ptr trans_data(new PointCloudT);
	pcl::transformPointCloud(*original_data, *trans_data, init_transform);
	//===============PassthroughFilter========================//
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(original_model);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-10, 10);
	pass.setFilterLimitsNegative(true);
	pass.filter(*surface_model);
	pass.setInputCloud(trans_data);
	pass.filter(*surface_data);

	//===============NormalEstimation========================//
	qDebug("Start Normal Estimating!");
	computePointNormal(original_model, model_with_normals);
	computePointNormal(original_data, data_with_normals);
	computePointNormal(surface_model, surface_model_with_normals);
	computePointNormal(surface_data, surface_data_with_normals);

	/*pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setNumberOfThreads(3);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(30);

	ne.setInputCloud(original_model);
	ne.compute(*normals_model);
	for (size_t i = 0; i < original_model->points.size(); ++i)
	{
		const pcl::PointXYZ &pt = original_model->points[i];
		const pcl::Normal &pn = normals_model->points[i];
		pcl::PointNormal points;
		points.x = pt.x;
		points.y = pt.y;
		points.z = pt.z;
		points.normal_x = pn.normal_x;
		points.normal_y = pn.normal_y;
		points.normal_z = pn.normal_z;
		model_with_normals->push_back(points);
	}

	ne.setInputCloud(original_data);
	ne.compute(*normals_data);
	for (size_t i = 0; i < original_data->points.size(); ++i)
	{
		const pcl::PointXYZ &pt = original_data->points[i];
		const pcl::Normal &pn = normals_data->points[i];
		pcl::PointNormal points;
		points.x = pt.x;
		points.y = pt.y;
		points.z = pt.z;
		points.normal_x = pn.normal_x;
		points.normal_y = pn.normal_y;
		points.normal_z = pn.normal_z;
		data_with_normals->push_back(points);
	}
	ne.setInputCloud(surface_model);
	ne.compute(*normals_surface_model);
	for (size_t i = 0; i < surface_model->points.size(); ++i)
	{
		const pcl::PointXYZ &pt = surface_model->points[i];
		const pcl::Normal &pn = normals_surface_model->points[i];
		pcl::PointNormal points;
		points.x = pt.x;
		points.y = pt.y;
		points.z = pt.z;
		points.normal_x = pn.normal_x;
		points.normal_y = pn.normal_y;
		points.normal_z = pn.normal_z;
		surface_model_with_normals->push_back(points);
	}
	ne.setInputCloud(surface_data);
	ne.compute(*normals_surface_data);
	for (size_t i = 0; i < surface_data->points.size(); ++i)
	{
		const pcl::PointXYZ &pt = surface_data->points[i];
		const pcl::Normal &pn = normals_surface_data->points[i];
		pcl::PointNormal points;
		points.x = pt.x;
		points.y = pt.y;
		points.z = pt.z;
		points.normal_x = pn.normal_x;
		points.normal_y = pn.normal_y;
		points.normal_z = pn.normal_z;
		surface_data_with_normals->push_back(points);
	}*/
	

	//=================GA Implementation=========================//
	qDebug("Start GA!");
	int popsize = 20;
	double mutationrate = 0.8;
	double crossoverrate = 1;
	int generationmax = 10;
	double maxstep = 0.001;
	double leftmax = -PI / 9;
	double rightmax = PI / 9;

	Reset();
	Init(popsize, mutationrate, crossoverrate, generationmax,
		maxstep, leftmax, rightmax);
	ImplementGa();

	double omega, fai, kappa;
	double a, b, c;
	omega = popOperation.fitnessChromo.vecGenome[0];
	fai = popOperation.fitnessChromo.vecGenome[1];
	kappa = popOperation.fitnessChromo.vecGenome[2];
	a = popOperation.fitnessChromo.vecGenome[3];
	b = popOperation.fitnessChromo.vecGenome[4];
	c = popOperation.fitnessChromo.vecGenome[5];

	Eigen::AngleAxisd rollAngle(omega, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd yawAngle(fai, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd pitchAngle(kappa, Eigen::Vector3d::UnitX());

	Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

	Eigen::Matrix3d rotationMatrix = q.matrix();

	Eigen::Affine3d r(rotationMatrix);
	Eigen::Affine3d t(Eigen::Translation3d(a, b, c));
	Eigen::Matrix4d transformation = (t * r).matrix();

	pcl::transformPointCloud(*trans_data, *final_data, init_transform);

	//===================Visualization==========================//
	int v1(0), v2(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0.0, 0.0, 0.0, v1);
	viewer->addText("Before Optimal Registration", 10, 10, "v1 text", v1);
	PointCloudColorHandlerCustom<PointXYZ> green(original_model, 0, 255, 0);
	PointCloudColorHandlerCustom<PointXYZ> red(trans_data, 255, 0, 0);
	viewer->addPointCloud(original_model, green, "v1_target", v1);
	viewer->addPointCloud(trans_data, red, "v1_sourse", v1);


	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.0, 0.0, 0.0, v2);
	viewer->addText("After Optimal Registration", 10, 10, "v2 text", v2);
	PointCloudColorHandlerCustom<PointXYZ> green2(original_model, 0, 255, 0);
	PointCloudColorHandlerCustom<PointXYZ> red2(final_data, 255, 0, 0);
	viewer->addPointCloud(original_model, green2, "v2_target", v2);
	viewer->addPointCloud(final_data, red2, "v2_sourse", v2);
	//viewer->addPointCloud(datum_model, green2, "v2_target", v2);
	//viewer->addPointCloud(datum_data, red2, "v2_sourse", v2);

	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(datum_data, 255, 0, 255);
	//viewer->addPointCloud(datum_data, tgt_h, "target", v2);

	//viewer->spin();

	//ia->viewPair(cloud1ds, cloud2ds, cloud1, cloud2);
	//pcl::io::savePCDFile("result.pcd", final);
	//view(final);

	//======================ICP==========================//
	/*
	qDebug("icp started...");
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setMaxCorrespondenceDistance(0.1);
	icp.setTransformationEpsilon(1e-10);
	icp.setEuclideanFitnessEpsilon(0.01);
	icp.setMaximumIterations(100);
	icp.setInputSource(cloud2ds);
	icp.setInputTarget(cloud1ds);
	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	icp.align(*output);
	double fitness;
	fitness = icp.getFitnessScore();
	PointCloudColorHandlerCustom<PointXYZ> blue(output, 0, 0, 255);
	Eigen::Matrix4d icp_transform = icp.getFinalTransformation().cast<double>();
	viewer->addPointCloud(output, blue, "v2_output", v2);
	//pcl::transformPointCloud(*cloud2ds, *cloud2ds, icp_transform);
	qDebug("icp finished...fitness score:%f",fitness);
	qDebug("Rotation matrix :\n");
	qDebug("    | %6.3f %6.3f %6.3f | \n", icp_transform(0, 0), icp_transform(0, 1), icp_transform(0, 2));
	qDebug("R = | %6.3f %6.3f %6.3f | \n", icp_transform(1, 0), icp_transform(1, 1), icp_transform(1, 2));
	qDebug("    | %6.3f %6.3f %6.3f | \n", icp_transform(2, 0), icp_transform(2, 1), icp_transform(2, 2));
	qDebug("Translation vector :\n");
	qDebug("t = < %6.3f, %6.3f, %6.3f >\n\n", icp_transform(0, 3), icp_transform(1, 3), icp_transform(2, 3));
	*/
	viewer->spinOnce();
	return true;
}

bool 
Visualization::computePointNormal(const PointCloud<PointXYZ>::Ptr cloud, PointCloud<PointNormal>::Ptr &cloud_with_normals)
{
	PointCloud<Normal>::Ptr normals(new PointCloudN);
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setNumberOfThreads(3);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(30);

	ne.setInputCloud(cloud);
	ne.compute(*normals);

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		const pcl::PointXYZ &pt = cloud->points[i];
		const pcl::Normal &pn = normals->points[i];
		pcl::PointNormal points;
		points.x = pt.x;
		points.y = pt.y;
		points.z = pt.z;
		points.normal_x = pn.normal_x;
		points.normal_y = pn.normal_y;
		points.normal_z = pn.normal_z;
		cloud_with_normals->push_back(points);
	}
	return true;
}