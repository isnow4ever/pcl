#include "ProjectMainWindow.h"

#include <iostream>
#include <pcl/io/ply_io.h>

#include <QFileDialog>
#include <QDir>

#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <QVTKWidget.h>


ProjectMainWindow::ProjectMainWindow()
{
	setupUi(this);

	initialVtkWidget();

	//连接打开的信号与相应的槽
	connect( m_OpenAction, SIGNAL( triggered() ), this, SLOT( onOpenSlot() ) ); 
}

ProjectMainWindow::~ProjectMainWindow()
{
}

void ProjectMainWindow::initialVtkWidget()
{
	cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
		;

	//m_QVTKWidget->SetRenderWindow(viewer->getRenderWindow());
	//viewer->setupInteractor(m_QVTKWidget->GetInteractor(), m_QVTKWidget->GetRenderWindow());
	//m_QVTKWidget->update();
}

void ProjectMainWindow::onOpenSlot()
{

	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open PointCloud"), ".",
		tr("Open PLY files(*.ply)"));

	if (!fileName.isEmpty())
	{
		std::string file_name = fileName.toStdString();

		pcl::io::loadPLYFile (file_name, *cloud);

		//viewer->updatePointCloud(cloud, "cloud");
		//viewer->resetCamera();
		m_QVTKWidget->update();
	}

}
