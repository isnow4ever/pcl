#pragma once

#include "res.h"
#include "record.h"
#include <QObject>
#include <QThread>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/pfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh.h>
#include <pcl/visualization/histogram_visualizer.h>
#include<pcl/visualization/pcl_plotter.h>

#include <iostream>
#include <string>

class Visualization : public QObject
{
	Q_OBJECT

public:
	Visualization(QString);
	virtual ~Visualization();

	Record *record;

	QString filename;

	PointCloudT::Ptr cloud;
	PointCloudT::Ptr cloud_out;
	PointCloudN::Ptr cloud_normals;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	pcl::console::TicToc pcl_time;

	void preview(QString filename);

	int feature_id;

	bool filter_n_downsampling();

	//bool kdtreeFlag;
	bool computeKdtree();

	//bool centroidFlag;
	bool computeCentroid();

	double search_radius;
	int normal_level;
	double normal_scale;
	bool computeNormals();

	bool computeFPFH();

	bool computeEGI();


signals:
	void finished();

public slots:
	void OnStarted();
	//void kdtreeFlagToggle();
	//void centroidFlagToggle();
	void feature_id_slot(int);

private:
	
//
//protected:
//	void run();
};

