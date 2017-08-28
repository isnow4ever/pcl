#ifndef Project_MainWindow_H
#define Project_MainWindow_H

#include <QMainWindow>
#include "ui_ProjectMainWindow.h"

#include <vtkSmartPointer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

class vtkImageViewer2;
class vtkRenderer;
class vtkEventQtSlotConnect;
class vtkObject;
class vtkCommand;

class ProjectMainWindow : public QMainWindow, public Ui::ProjectMainWindow
{
	Q_OBJECT

public:
	ProjectMainWindow();
	~ProjectMainWindow();

private:
	//Ui::PCLVisualizer ui;
	//点云数据存储
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	//初始化vtk部件
	void initialVtkWidget();

private slots:
	//响应打开图像文件的槽函数
	void onOpenSlot();

	//响应鼠标移动的消息，实时输出鼠标的当前位置
	//void updateCoords(vtkObject* obj);

private:
	//vtkSmartPointer< vtkImageViewer2 >           m_pImageViewer;
	//vtkSmartPointer< vtkRenderer >                   m_pRenderder;

	//vtkEventQtSlotConnect* m_Connections;
};

#endif
