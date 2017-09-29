#include "visualization.h"



Visualization::Visualization(QString filename)
	:filename(filename)
{
	record = new Record();
	cloud.reset(new PointCloudT);
	cloud_normals.reset(new PointCloudN);
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
Visualization::preview(QString filename)
{
	viewer.reset(new pcl::visualization::PCLVisualizer("preview", true));

	std::string file_name = filename.toStdString();
	if (filename.endsWith(".pcd"))
	{
		if (!pcl::io::loadPCDFile(file_name, *cloud))
		{
			//QMessageBox msgBox;
			//msgBox.setText("Couldn't read file.");
			//int ret = msgBox.exec();
		}
			

	}		
			
	else if (filename.endsWith(".ply"))
	{
		if (!pcl::io::loadPLYFile(file_name, *cloud))
		{
			//QMessageBox msgBox;
			//msgBox.setText("Couldn't read file.");
			//int ret = msgBox.exec();
		}
	}


	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb(cloud, 20, 180, 20);
	//pcl::visualization::PointCloudColorHandlerRGBField<PointC> rgb(cloud);
	viewer->addPointCloud(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem(1.0);
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
	this->preview(filename);
	emit finished();
}

void Visualization::feature_id_slot(int feature)
{
	feature_id = feature;
}

void
Visualization::computeKdtree()
{
	//kd-tree tutorials
	srand(time(NULL));
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(cloud);
	PointT searchPoint;
	searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

	int k = 10;
	std::vector<int> pointIdxNKNSearch(k);
	std::vector<float> pointNKNSquaredDistance(k);
	std::stringstream ss;
	ss  << "k nearest neighbor search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with K = " << k << std::endl;

	record->infoRec(QString::fromStdString(ss.str()));

	if (kdtree.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
		{
			std::stringstream ss;

			ss  << " " << cloud->points[pointIdxNKNSearch[i]].x
				<< " " << cloud->points[pointIdxNKNSearch[i]].y
				<< " " << cloud->points[pointIdxNKNSearch[i]].z
				<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
			record->infoRec(QString::fromStdString(ss.str()));
		}
	}
}

void
Visualization::computeCentroid()
{
	Eigen::Vector4f xyz_centroid;
	pcl::compute3DCentroid(*cloud, xyz_centroid);
	std::stringstream ss;
	for (size_t i = 0; i < 4; i++)
	{
		ss << " " << xyz_centroid(i);
	}
	ss << std::endl;
	record->infoRec(QString::fromStdString(ss.str()));

	PointT center;
	center.x = xyz_centroid(0);
	center.y = xyz_centroid(1);
	center.z = xyz_centroid(2);

	viewer->addSphere(center, 0.1, 1, 0.2, 0.2, "centroid");
}

void Visualization::computeNormals()
{
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
	ne.setViewPoint(0, 0, 0);
	// Compute the features
	ne.compute(*cloud_normals);

	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals, normal_level, normal_scale);
	// Compute the 3x3 covariance matrix
	//Eigen::Matrix3f covariance_matrix;
	//pcl::computeCovarianceMatrix(cloud, xyz_centroid, covariance_matrix);
}

void Visualization::computeFPFH()
{
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>fpfh;

	fpfh.setInputCloud(cloud);
	fpfh.setInputNormals(cloud_normals);

	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	fpfh.setSearchMethod(tree);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());

	fpfh.setRadiusSearch(search_radius);
	fpfh.compute(*fpfhs);

	//pcl::visualization::PCLHistogramVisualizer hist_viewer;
	//hist_viewer.addFeatureHistogram(*fpfhs, 33);

	// Plotter object.
	pcl::visualization::PCLPlotter plotter;
	// We need to set the size of the descriptor beforehand.
	plotter.addFeatureHistogram(*fpfhs, 33);

	plotter.plot();


}