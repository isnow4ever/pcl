#include "ProjectMainWindow.h"

#include <iostream>
#include <pcl/io/ply_io.h>

#include <QFileDialog>
#include <QDir>

#include <vtkRenderWindow.h>
//#include <vtkRenderer.h>
//#include <vtkImageViewer2.h>
//#include <QVTKWidget.h>
//
//#include <vtkJPEGReader.h>
//#include <vtkImageActor.h>
//
//#include <vtkEventQtSlotConnect.h>
//#include <vtkCommand.h>

ProjectMainWindow::ProjectMainWindow()
{
	setupUi(this);

	initialVtkWidget();

	//m_pImageViewer  = vtkSmartPointer< vtkImageViewer2 >::New();
	//m_pRenderder      = vtkSmartPointer< vtkRenderer >::New();

	// 设置m_QVTKWidget的渲染器
	//m_QVTKWidget->GetRenderWindow()->AddRenderer(m_pRenderder);

	//连接打开的信号与相应的槽
	connect( m_OpenAction, SIGNAL( triggered() ), this, SLOT( onOpenSlot() ) ); 

	//m_Connections = vtkEventQtSlotConnect::New();
	/*m_Connections->Connect(m_QVTKWidget->GetRenderWindow()->GetInteractor(),
		vtkCommand::MouseMoveEvent,
		this,
		SLOT(updateCoords(vtkObject*)));*/
}

ProjectMainWindow::~ProjectMainWindow()
{
}

void ProjectMainWindow::initialVtkWidget()
{
	//cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	viewer->addPointCloud(cloud, "cloud");

	m_QVTKWidget->SetRenderWindow(viewer->getRenderWindow());
	//viewer->setupInteractor(m_QVTKWidget->GetInteractor(), m_QVTKWidget->GetRenderWindow());
	m_QVTKWidget->update();
}

void ProjectMainWindow::onOpenSlot()
{

	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open PointCloud"), ".",
		tr("Open PLY files(*.ply)"));

	if (!fileName.isEmpty())
	{
		std::string file_name = fileName.toStdString();
		//sensor_msgs::PointCloud2 cloud2;
		pcl::PCLPointCloud2 cloud2;

		pcl::io::loadPLYFile (file_name, *cloud);
		////pcl::PointCloud<Eigen::MatrixXf> cloud2;
		//Eigen::Vector4f origin;
		//Eigen::Quaternionf orientation;
		//int pcd_version;
		//int data_type;
		//unsigned int data_idx;
		//int offset = 0;
		//pcl::PCDReader rd;
		//rd.readHeader(file_name, cloud2, origin, orientation, pcd_version, data_type, data_idx);

		//if (data_type == 0)
		//{
		//	pcl::io::loadPCDFile(fileName.toStdString(), *cloud);
		//}
		//else if (data_type == 2)
		//{
		//	pcl::PCDReader reader;
		//	reader.read<pcl::PointXYZ>(fileName.toStdString(), *cloud);
		//}
		//viewer->addPointCloud(cloud, "cloud");
		//viewer->updatePointCloud(cloud, "cloud");
		//viewer->resetCamera();
		m_QVTKWidget->update();
	}

	//m_QVTKWidget->GetRenderWindow()->Render();
}

//void ProjectMainWindow::updateCoords(vtkObject* obj)
//{
//	// 获取交互器
//	vtkRenderWindowInteractor* iren = vtkRenderWindowInteractor::SafeDownCast(obj);
//
//	// 获取鼠标的当前位置
//	int event_pos[2];
//	iren->GetEventPosition(event_pos);
//
//	QString str;
//	str.sprintf("x=%d : y=%d", event_pos[0], event_pos[1]);
//	m_StatusBar->showMessage(str);
//}