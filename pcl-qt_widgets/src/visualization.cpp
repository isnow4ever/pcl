#include "visualization.h"



Visualization::Visualization(QString filename)
	:filename(filename)
{
	record = new Record();
	cloud.reset(new PointCloudT);
	viewer.reset(new pcl::visualization::PCLVisualizer("preview", false));

	kdtreeFlag = false;
	centroidFlag = false;
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
		if (kdtreeFlag)
		{
			computeKdtree();
			kdtreeFlag = false;
		}

		if (centroidFlag)
		{
			computeCentroid();
			centroidFlag = false;
		}
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

void
Visualization::kdtreeFlagToggle()
{
	kdtreeFlag = true;
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
Visualization::centroidFlagToggle()
{
	centroidFlag = true;
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