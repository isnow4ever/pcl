#pragma once

#include "res.h"
#include <QObject>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <string>

class Visualization : public QObject
{
	Q_OBJECT

public:
	Visualization(QString);
	virtual ~Visualization();

	QString filename;

	PointCloudT::Ptr cloud;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;


	void preview(QString filename);

signals:
	void finished();

public slots:
	void OnStarted();
};

