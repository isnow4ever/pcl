#include "ICPReg.h"
#include <Eigen/src/Core/IO.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool next_iteration = false;

void
keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,
	void* nothing)
{
	if (event.getKeySym() == "space" && event.keyDown())
		next_iteration = true;
}

ICPReg::ICPReg(QString filename_model, QString filename_data, int iterations)
	: filename_model(filename_model),filename_data(filename_data),iterations(iterations)
{

	// The point clouds we will be using
	cloud_in.reset(new PointCloudT);  // Original point cloud
	cloud_tr.reset(new PointCloudT);  // Transformed point cloud
	cloud_icp.reset(new PointCloudT);  // ICP output point cloud
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
}


ICPReg::~ICPReg()
{
}

void
ICPReg::print4x4Matrix(const Eigen::Matrix4d & matrix)
{
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void
ICPReg::proceed(QString filename_model, QString filename_data, int iterations)
{
	pcl::console::TicToc time;
	time.tic();
	std::string file_name = filename_model.toStdString();

	emit infoRec("Loading model...");
	emit progressBarUpdate(10);

	pcl::io::loadPLYFile(file_name, *cloud_in);

	emit infoRec("Loading model finished.");
	emit progressBarUpdate(20);
	
						 //Downsampling
	pcl::console::print_highlight("Downsampling...\n");
	pcl::VoxelGrid<pcl::PointXYZ> grid;
	const float leaf = 0.005f;
	grid.setLeafSize(leaf, leaf, leaf);
	grid.setInputCloud(cloud_in);
	grid.filter(*cloud_in);

	emit infoRec("Downsampling finished.");
	emit progressBarUpdate(30);

	// Defining a rotation matrix and translation vector
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

	// A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
	double theta = M_PI / 8;  // The angle of rotation in radians
	transformation_matrix(0, 0) = cos(theta);
	transformation_matrix(0, 1) = -sin(theta);
	transformation_matrix(1, 0) = sin(theta);
	transformation_matrix(1, 1) = cos(theta);

	// A translation on Z axis (0.4 meters)
	transformation_matrix(2, 3) = 0.4;

	// Display in terminal the transformation matrix
	std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
	print4x4Matrix(transformation_matrix);

	// Executing the transformation
	pcl::transformPointCloud(*cloud_in, *cloud_icp, transformation_matrix);
	*cloud_tr = *cloud_icp;  // We backup cloud_icp into cloud_tr for later use

	emit infoRec("Transform finished.");
	emit progressBarUpdate(40);
	QString matrix;
	Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
	std::stringstream trans;
	trans << transformation_matrix.format(OctaveFmt);
	emit infoRec(QString::fromStdString(trans.str()));

							 // The Iterative Closest Point algorithm
	time.tic();
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setMaximumIterations(iterations);
	icp.setInputSource(cloud_icp);
	icp.setInputTarget(cloud_in);
	icp.align(*cloud_icp);
	icp.setMaximumIterations(1);  // We set this variable to 1 for the next time we will call .align () function
	std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc() << " ms" << std::endl;

	if (icp.hasConverged())
	{
		std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
		std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
		transformation_matrix = icp.getFinalTransformation().cast<double>();
		print4x4Matrix(transformation_matrix);

		trans.str("");
		trans << transformation_matrix.format(OctaveFmt);
		emit infoRec(QString::fromStdString(trans.str()));
	}
	else
	{
		PCL_ERROR("\nICP has not converged.\n");
		return;
	}

	viewer.reset(new pcl::visualization::PCLVisualizer("icp_demo", true));
	// Create two verticaly separated viewports
	int v1(0);
	int v2(1);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);

	// The color we will be using
	float bckgr_gray_level = 0.0;  // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;

	// Original point cloud is white
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cloud_in, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
		(int)255 * txt_gray_lvl);
	viewer->addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
	viewer->addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v2", v2);

	// Transformed point cloud is green
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(cloud_tr, 20, 180, 20);
	viewer->addPointCloud(cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

	// ICP aligned point cloud is red
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h(cloud_icp, 180, 20, 20);
	viewer->addPointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

	// Adding text descriptions in each viewport
	viewer->addText("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
	viewer->addText("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

	std::stringstream ss;
	ss << iterations;
	std::string iterations_cnt = "ICP iterations = " + ss.str();
	viewer->addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

	// Set background color
	viewer->setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
	viewer->setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

	// Set camera position and orientation
	viewer->setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer->setSize(640, 512);  // Visualiser window size

								  // Register keyboard callback :
	viewer->registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);

	// Display the visualiser
	while (!viewer->wasStopped())
	{
		viewer->spinOnce();

		// The user pressed "space" :
		if (next_iteration)
		{
			// The Iterative Closest Point algorithm
			time.tic();
			icp.align(*cloud_icp);
			std::cout << "Applied 1 ICP iteration in " << time.toc() << " ms" << std::endl;

			std::stringstream info_ss;
			info_ss << "Applied 1 ICP iteration in " << time.toc() << " ms";
			emit infoRec(QString::fromStdString(info_ss.str()));
			emit progressBarUpdate(50);

			if (icp.hasConverged())
			{
				printf("\033[11A");  // Go up 11 lines in terminal output.
				printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
				std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
				transformation_matrix *= icp.getFinalTransformation().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
				print4x4Matrix(transformation_matrix);  // Print the transformation between original pose and current pose

				trans.str("");
				trans << transformation_matrix.format(OctaveFmt);
				emit infoRec(QString::fromStdString(trans.str()));

				info_ss.str("");
				info_ss << "ICP has converged, score is " << icp.getFitnessScore();
				emit infoRec(QString::fromStdString(info_ss.str()));
				emit progressBarUpdate(100);

				ss.str("");
				ss << iterations;
				std::string iterations_cnt = "ICP iterations = " + ss.str();
				viewer->updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
				viewer->updatePointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
			}
			else
			{
				PCL_ERROR("\nICP has not converged.\n");
				return;
			}
		}
		next_iteration = false;

		//emit progressBarUpdate(100);
	}
	qDebug("123");
}

void
ICPReg::OnStarted()
{
	this->proceed(filename_model, filename_data, iterations);
	emit finished();
}

//void
//ICPReg::OnFinished()
//{
//	
//}
