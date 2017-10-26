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
#include <pcl/visualization/pcl_plotter.h>

#include <iostream>
#include <string>

class EGIReg : public QObject
{
	Q_OBJECT

public:
	EGIReg(PointCloudT::Ptr cloud_model, PointCloudT::Ptr cloud_data);
	virtual ~EGIReg();

	//functions
	void params_initial();

	Eigen::Vector3f translationEstimate();
	Eigen::Matrix3f rotationEstimate();
	
	void normalSphereCompute(PointCloudT::Ptr cloud_in, PointCloudT::Ptr cloud_out);
	void mapToIcosahedron(PointCloudT::Ptr normal_sphere, std::vector<double> intensity, int EGI_level);

	void search(Eigen::Matrix4f transformation);

	//Records
	void recordToFile(QString filename, QString content);

	//input & output
	void setNormalSearchRadius(double);
	void setNormalLevel(int);
	void setNormalScale(double);

	Eigen::Vector3f getTranslation();
	Eigen::Matrix3f getRotation();
	Eigen::Matrix4f getTransformation();

	int getProgress();

private:
	PointCloudT::Ptr model;
	PointCloudT::Ptr data;

	PointCloudT::Ptr model_normal_sphere;
	PointCloudT::Ptr data_normals_sphere;

	std::vector< std::vector<int> > face_indices;
	std::vector< std::vector<double> > vertex_indices;

	//normal estimate params
	double search_radius;
	int normal_level;
	double normal_scale;

	//rotation angle search params
	int EGI_level;
	double step_size;
	std::vector<double> intensity;

	//GUI
	int progress;

	//rotation angles
	Eigen::Vector3f rot_angles;
	double alpha;
	double beta;
	double gama;

	Eigen::Vector3f translation;
	Eigen::Matrix3f rotation;
	Eigen::Matrix4f transformation;

};

