#pragma once

#include "res.h"
#include "record.h"
#include <QObject>
#include <QThread>

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

//---------- define a Chromo class-----------//
class Chromo2//定义一个染色体
{
public:
	friend class Visualization;
	friend class Population2;
	Chromo2() :fitness(0), chromoLength(6)
	{
		vecGenome.resize(6);
	};
	Chromo2::Chromo2(std::vector<double> &vec, double &fit)
	{
		fitness = 0;
		vecGenome = vec;
		fitness = fit;
		chromoLength = 6;
	}
	~Chromo2() {}
private:
	std::vector<double> vecGenome;//染色体包含的基因组
	double fitness;//染色体适应度
	int chromoLength;//染色体长度----本文染色体统一长度为6，因为就一个基因“位置”
};


//---------- define a Population class-------//
class Population2
{
public:
	std::vector<Chromo2> vecPop;//定义一个种群
	friend class Visualization;
	Population2() : totalFitness(0), bestFitness(0), averageFitness(0), worstFitness(0),
		mutationRate(0), crossoverRate(0), leftMax(0), rightMax(0), MaxY(0),
		generationMax(0), maxStep(0) {
		vecPop.resize(500); srand((unsigned)time(NULL));
	}//初始化随机种子
	~Population2() {}
private:
	int popSize;//种群里人口数量
				//一些适应率参数
	double totalFitness;
	double bestFitness;
	double averageFitness;
	double worstFitness;
	//最适合的个体
	Chromo2 fitnessChromo;
	//突变概率
	double mutationRate;
	//交叉概率
	double crossoverRate;
	//繁衍的代数
	int generationMax;
	//最大变异的步长
	double maxStep;
	//边界值
	double leftMax;
	double rightMax;
	//函数最大值
	double MaxY;
};

//class GA : public QObject
//{
//public:
//	//GA
//	void Reset();//重置参数
//				 //外界输入初始化参数、染色体初始化
//	void Init(int popsize, double mutationrate, double crossoverrate, int generationmax,
//		double maxstep, double leftmax, double rightmax);
//	void ImplementGa();//执行遗传算法
//
//
//	Eigen::Vector3d translation;
//	Eigen::Matrix3d rotation;
//	Eigen::Matrix4d transformation;
//
//private:
//	//GA
//	double Curve(Chromo2 input);//解码---x->F(x)
//	double Random();//制造随机0-1的输
//	Population2 popOperation;//定义一个可操作的种群-populationOperation	
//	int generationCount;//记录杂交的代数
//	void CalculateRate();//初始化软件计算的参数
//	Chromo2 GenomeRoulette();//轮盘赌选择函数
//	void Mutate(Chromo2 genome);//基因突变函数
//	void Epoch(Population2& newgeneration);//产生新的一代
//	void Report();//输出到终端的信息
//};


class Visualization : public QObject
{
	Q_OBJECT

public:
	Visualization(QString, QString);
	virtual ~Visualization();

	Record *record;

	QString filename;
	QString filename_data;

	PointCloudT::Ptr original_model;
	PointCloudT::Ptr original_data;
	PointCloudT::Ptr final_data;
	PointCloudT::Ptr cloud;
	PointCloudT::Ptr cloud_out;
	PointCloudN::Ptr cloud_normals;
	PointCloudT::Ptr cloud_data;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	pcl::console::TicToc pcl_time;

	void preview(QString filename, QString filename_data);

	int feature_id;

	bool filter_n_downsampling();

	//bool kdtreeFlag;
	bool computeKdtree();

	//bool centroidFlag;
	bool computeCentroid();

	double search_radius;
	int normal_level;
	double normal_scale;
	bool computeNormals();

	bool computeFPFH();

	bool computeEGI();

	std::vector<double> rotation;
	bool registration();

	bool initialAlignment();

	void computeDatumCoefficients(PointCloudT::Ptr, PointCloudN::Ptr, PointCloudT::Ptr, pcl::ModelCoefficients::Ptr);

	double computeDatumError(PointCloudT::Ptr, PointCloudN::Ptr, PointCloudT::Ptr);

	double computeDatumAngle(pcl::ModelCoefficients::Ptr, pcl::ModelCoefficients::Ptr);

	double enveloped(PointCloudT::Ptr, PointCloudN::Ptr, PointCloudT::Ptr, std::vector<double>);

	double computeSurfaceVariance(std::vector<double>);

	double computeFitness(Eigen::Matrix4d &transformation);

	void search(Eigen::Matrix4d &transformation);

	bool sacSegment();

	bool optimalReg();
signals:
	void finished();

	public slots:
	void OnStarted();
	//void kdtreeFlagToggle();
	//void centroidFlagToggle();
	void feature_id_slot(int);

private:

	//
	//protected:
	//	void run();


public:
	//GA
	void Reset();//重置参数
				 //外界输入初始化参数、染色体初始化
	void Init(int popsize, double mutationrate, double crossoverrate, int generationmax,
		double maxstep, double leftmax, double rightmax);
	void ImplementGa();//执行遗传算法


	//Eigen::Vector3f translation;
	//Eigen::Matrix3f rotation;
	Eigen::Matrix4f init_transform;

private:
	//GA
	double Curve(Chromo2 input);//解码---x->F(x)
	double Random();//制造随机0-1的输
	Population2 popOperation;//定义一个可操作的种群-populationOperation	
	int generationCount;//记录杂交的代数
	void CalculateRate();//初始化软件计算的参数
	Chromo2 GenomeRoulette();//轮盘赌选择函数
	void Mutate(Chromo2 genome);//基因突变函数
	void Epoch(Population2& newgeneration);//产生新的一代
	void Report();//输出到终端的信息

	PointCloudT::Ptr datum_data;
	PointCloudT::Ptr datum_model;
	PointCloudT::Ptr surface_data;
	PointCloudT::Ptr surface_model;

	PointCloudN::Ptr normals_model;
	PointCloudN::Ptr normals_data;
	PointCloudN::Ptr normals_datum_model;
	PointCloudN::Ptr normals_datum_data;
	PointCloudN::Ptr normals_surface;
};
