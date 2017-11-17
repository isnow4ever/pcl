#include "visualization.h"
#include "EGIReg.h"
#include <Eigen/Geometry> 

#define ICO_X .525731112119133606
#define ICO_Z .850650808352039932

Visualization::Visualization(QString filename, QString filename_data)
	:filename(filename), filename_data(filename_data)
{
	record = new Record();
	original_model.reset(new PointCloudT);
	original_data.reset(new PointCloudT);
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
	*original_model = *cloud;
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
	*original_data = *cloud_data;

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
	pcl::visualization::PointCloudColorHandlerCustom<PointT> green(cloud, 20, 180, 20);
	viewer->addPointCloud(cloud, green, "model cloud",v1);
	
	pcl::visualization::PointCloudColorHandlerCustom<PointT> red(cloud_data, 180, 20, 20);
	viewer->addPointCloud(cloud_data, red, "data cloud", v2);

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
			record->progressBarUpdate(int(100*i/size));
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
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_data, cloud_data_normals, normal_level, normal_scale,"data normals", 2);

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
	//nsReg->translationEstimate();

	viewer->addPointCloud(nsReg->model_normal_sphere, green, "model", v3);
	viewer->addPointCloud(nsReg->data_normal_sphere, red, "data", v4);
	
	nsReg->search(tf);
	//double corr = nsReg->computeCorrelation(0.5, 0.5, 0.5);

	//viewer->addText(QString::number(corr, 'g', 6).toStdString(), 0.5, 0.5, "debug", 0);




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


	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", tf(0, 0), tf(0, 1), tf(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", tf(1, 0), tf(1, 1), tf(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", tf(2, 0), tf(2, 1), tf(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", tf(0, 3), tf(1, 3), tf(2, 3));

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