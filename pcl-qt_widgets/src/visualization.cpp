#include "visualization.h"



Visualization::Visualization(QString filename)
	:filename(filename)
{
	cloud.reset(new PointCloudT);
	viewer.reset(new pcl::visualization::PCLVisualizer("preview", false));
}


Visualization::~Visualization()
{
}

void
Visualization::preview(QString filename)
{
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


	viewer.reset(new pcl::visualization::PCLVisualizer("preview", true));

	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb(cloud, 20, 180, 20);
	//pcl::visualization::PointCloudColorHandlerRGBField<PointC> rgb(cloud);
	viewer->addPointCloud(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

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
	//std::cout << "k nearest neighbor search at (" << searchPoint.x
	//	<< " " << searchPoint.y
	//	<< " " << searchPoint.z
	//	<< ") with K = " << k << std::endl;

	qDebug("k nearest neighbor search at (%f %f %f) with K = %d", searchPoint.x, searchPoint.y, searchPoint.z, k);
	 
	if (kdtree.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
		{
			//std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x
			//	<< " " << cloud->points[pointIdxNKNSearch[i]].y
			//	<< " " << cloud->points[pointIdxNKNSearch[i]].z
			//	<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
			qDebug("  %f %f %f (squared distance: %f)",
				cloud->points[pointIdxNKNSearch[i]].x,
				cloud->points[pointIdxNKNSearch[i]].y,
				cloud->points[pointIdxNKNSearch[i]].z,
				pointNKNSquaredDistance[i]);
		}
	}

	while (!viewer->wasStopped())
	{
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