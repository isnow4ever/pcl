#include "mainwindow.h"
#include "ui_mainwindow.h"


#include <QFileDialog>
#include <QDir>
#include <QThread>

//bool next_iteration = false;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

	//next_iteration = false;


	
	connect(ui->pushButton, SIGNAL(clicked()), this, SLOT(onModelFileOpenSlot()));
	connect(ui->pushButton_2, SIGNAL(clicked()), this, SLOT(onDataFileOpenSlot()));
	connect(ui->pushButton_3, SIGNAL(clicked()), this, SLOT(onVisualizerOpenSlot()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void
MainWindow::icp_proceed(QString filename_model, QString filename_data, int iterations)
{
	icpreg = new ICPReg(filename_model, filename_data, iterations);
	QThread * th = new QThread();
	icpreg->moveToThread(th);

	connect(th, SIGNAL(started()), icpreg, SLOT(OnStarted()));
	connect(icpreg, SIGNAL(finished()), th, SLOT(terminate()));

	th->start();
}

void
MainWindow::onModelFileOpenSlot()
{
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open PointCloud"), ".",
		tr("Open PLY files(*.ply)"));
	QFileInfo temDir(fileName);
	if (!fileName.isEmpty())
	{
		ui->textBrowser->setText(temDir.fileName());
		fileName_Model = fileName;
	}
}

void
MainWindow::onDataFileOpenSlot()
{
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open PointCloud"), ".",
		tr("Open PLY files(*.ply)"));
	QFileInfo temDir(fileName);
	if (!fileName.isEmpty())
	{
		ui->textBrowser_2->setText(temDir.fileName());
		fileName_Data = fileName;
	}
}

void
MainWindow::onVisualizerOpenSlot()
{
	if ((!fileName_Model.isEmpty()) && (!fileName_Model.isEmpty()))
	{
		ui->pushButton_3->setEnabled(false);
		iterations = ui->spinBox->value();
		icp_proceed(fileName_Model, fileName_Data, iterations);
	}
	
	//pcl::console::TicToc time;
	//time.tic();
	//if (!fileName_Model.isEmpty())
	//{
	//	ui->pushButton_3->setEnabled(false);

	//	std::string file_name = filename_model.toStdString();

	//	pcl::io::loadPLYFile(file_name, *cloud_in);
	//}
	//else
	//	return;

	//int iterations = 1;  // Default number of ICP iterations

	////Downsampling
	//pcl::console::print_highlight("Downsampling...\n");
	//pcl::VoxelGrid<pcl::PointXYZ> grid;
	//const float leaf = 0.005f;
	//grid.setLeafSize(leaf, leaf, leaf);
	//grid.setInputCloud(cloud_in);
	//grid.filter(*cloud_in);


	//// Defining a rotation matrix and translation vector
	//Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

	//// A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
	//double theta = M_PI / 8;  // The angle of rotation in radians
	//transformation_matrix(0, 0) = cos(theta);
	//transformation_matrix(0, 1) = -sin(theta);
	//transformation_matrix(1, 0) = sin(theta);
	//transformation_matrix(1, 1) = cos(theta);

	//// A translation on Z axis (0.4 meters)
	//transformation_matrix(2, 3) = 0.4;

	//// Display in terminal the transformation matrix
	//std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
	//print4x4Matrix(transformation_matrix);

	//// Executing the transformation
	//pcl::transformPointCloud(*cloud_in, *cloud_icp, transformation_matrix);
	//*cloud_tr = *cloud_icp;  // We backup cloud_icp into cloud_tr for later use

	//						 // The Iterative Closest Point algorithm
	//time.tic();
	//pcl::IterativeClosestPoint<PointT, PointT> icp;
	//icp.setMaximumIterations(iterations);
	//icp.setInputSource(cloud_icp);
	//icp.setInputTarget(cloud_in);
	//icp.align(*cloud_icp);
	//icp.setMaximumIterations(1);  // We set this variable to 1 for the next time we will call .align () function
	//std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc() << " ms" << std::endl;

	//if (icp.hasConverged())
	//{
	//	std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
	//	std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
	//	transformation_matrix = icp.getFinalTransformation().cast<double>();
	//	print4x4Matrix(transformation_matrix);
	//}
	//else
	//{
	//	PCL_ERROR("\nICP has not converged.\n");
	//	return ;
	//}

	//viewer.reset(new pcl::visualization::PCLVisualizer("icp_demo", true));
	//// Create two verticaly separated viewports
	//int v1(0);
	//int v2(1);
	//viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	//viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);

	//// The color we will be using
	//float bckgr_gray_level = 0.0;  // Black
	//float txt_gray_lvl = 1.0 - bckgr_gray_level;

	//// Original point cloud is white
	//pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cloud_in, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
	//	(int)255 * txt_gray_lvl);
	//viewer->addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
	//viewer->addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v2", v2);

	//// Transformed point cloud is green
	//pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(cloud_tr, 20, 180, 20);
	//viewer->addPointCloud(cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

	//// ICP aligned point cloud is red
	//pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h(cloud_icp, 180, 20, 20);
	//viewer->addPointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

	//// Adding text descriptions in each viewport
	//viewer->addText("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
	//viewer->addText("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

	//std::stringstream ss;
	//ss << iterations;
	//std::string iterations_cnt = "ICP iterations = " + ss.str();
	//viewer->addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

	//// Set background color
	//viewer->setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
	//viewer->setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

	//// Set camera position and orientation
	//viewer->setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	//viewer->setSize(1280, 1024);  // Visualiser window size

	//							 // Register keyboard callback :
	//viewer->registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);

	//// Display the visualiser
	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce();

	//	// The user pressed "space" :
	//	if (next_iteration)
	//	{
	//		// The Iterative Closest Point algorithm
	//		time.tic();
	//		icp.align(*cloud_icp);
	//		std::cout << "Applied 1 ICP iteration in " << time.toc() << " ms" << std::endl;

	//		if (icp.hasConverged())
	//		{
	//			printf("\033[11A");  // Go up 11 lines in terminal output.
	//			printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
	//			std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
	//			transformation_matrix *= icp.getFinalTransformation().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
	//			print4x4Matrix(transformation_matrix);  // Print the transformation between original pose and current pose

	//			ss.str("");
	//			ss << iterations;
	//			std::string iterations_cnt = "ICP iterations = " + ss.str();
	//			viewer->updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
	//			viewer->updatePointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
	//		}
	//		else
	//		{
	//			PCL_ERROR("\nICP has not converged.\n");
	//			return;
	//		}
	//	}
	//	next_iteration = false;
	//}

	////while (!viewer->wasStopped())
	////{
	////	viewer->spinOnce(100);
	////	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	////}
	//ui->pushButton_3->setEnabled(true);
}