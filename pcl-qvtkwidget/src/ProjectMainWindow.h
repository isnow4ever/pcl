#ifndef Project_MainWindow_H
#define Project_MainWindow_H

#include <QMainWindow>
#include "ui_ProjectMainWindow.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/shared_ptr.hpp>


class ProjectMainWindow : public QMainWindow, public Ui::ProjectMainWindow
{
	Q_OBJECT

public:
	ProjectMainWindow();
	~ProjectMainWindow();

private:
	//点云数据存储
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	//初始化vtk部件
	void initialVtkWidget();

private slots:
	//响应打开图像文件的槽函数
	void onOpenSlot();
};

#endif
