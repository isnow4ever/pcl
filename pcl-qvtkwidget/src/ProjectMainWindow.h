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
	//�������ݴ洢
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	//��ʼ��vtk����
	void initialVtkWidget();

private slots:
	//��Ӧ��ͼ���ļ��Ĳۺ���
	void onOpenSlot();
};

#endif
