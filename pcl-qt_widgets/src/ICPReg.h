#pragma once
#include "res.h"
#include "record.h"
#include <QObject>

#include <iostream>
#include <string>


#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>



class ICPReg : public QObject
{
	Q_OBJECT

public:
	ICPReg(QString, QString, int);
	~ICPReg();

	Record *record;

	QString filename_model;
	QString filename_data;

	int iterations;


	PointCloudT::Ptr cloud_in;  // Original point cloud
	PointCloudT::Ptr cloud_tr;  // Transformed point cloud
	PointCloudT::Ptr cloud_icp;  // ICP output point cloud

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;


	void print4x4Matrix(const Eigen::Matrix4d & matrix);
	void proceed(QString filename_model, QString filename_data, int iterations);

	void loadPointCloud(QString filename, pcl::PointCloud<PointT> &cloud);
	void downSampling(PointCloudT::Ptr cloud);
	void outliersRemover(PointCloudT::Ptr cloud);
	void infoCollect();

signals:
	void finished();
	//void progressBarUpdate(int);
	//void infoRec(QString);

public slots:
	void OnStarted();
	//void OnFinished();
};

