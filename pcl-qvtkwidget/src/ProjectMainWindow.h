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
	//�������ݴ洢
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	//��ʼ��vtk����
	void initialVtkWidget();

private slots:
	//��Ӧ��ͼ���ļ��Ĳۺ���
	void onOpenSlot();

	//��Ӧ����ƶ�����Ϣ��ʵʱ������ĵ�ǰλ��
	//void updateCoords(vtkObject* obj);

private:
	//vtkSmartPointer< vtkImageViewer2 >           m_pImageViewer;
	//vtkSmartPointer< vtkRenderer >                   m_pRenderder;

	//vtkEventQtSlotConnect* m_Connections;
};

#endif
