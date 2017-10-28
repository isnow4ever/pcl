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

	Eigen::Vector4f translationEstimate();
	Eigen::Matrix3f rotationEstimate();
	
	void normalSphereCompute(PointCloudT::Ptr cloud_in, PointCloudT::Ptr cloud_out);
	void mapToIcosahedron(PointCloudT::Ptr normal_sphere, std::vector<double> intensity, int EGI_level);
	void subdivide(Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d v3, long depth);

	void search(Eigen::Matrix4f &transformation);

	//Records
	Record *record;
	void recordToFile(QString filename, QString content);
	pcl::console::TicToc pcl_time;

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

	PointCloudT::Ptr model_trans;
	PointCloudT::Ptr data_trans;

	PointCloudT::Ptr model_normal_sphere;
	PointCloudT::Ptr data_normal_sphere;

	std::vector< std::vector<Eigen::Vector3d> > triangle_vectors;
	std::vector<Eigen::Vector3i> face_indices;
	std::vector<Eigen::Vector3d> vertex_indices;

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

	Eigen::Vector4f translation;
	Eigen::Matrix3f rotation;
	Eigen::Matrix4f transformation;

};

