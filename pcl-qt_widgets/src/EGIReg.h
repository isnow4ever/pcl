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

#include <string>
#include <vector>

//---------- define a Chromo class-----------//
class Chromo//定义一个染色体
{
public:
	friend class EGIReg;
	friend class Population;
	Chromo() :fitness(0), chromoLength(3) 
	{
		vecGenome.resize(3);
	};
	Chromo::Chromo(std::vector<double> &vec, double &fit)
	{
		fitness = 0;
		vecGenome = vec;
		fitness = fit;
		chromoLength = 3;
	}
	~Chromo() {}
private:
	std::vector<double> vecGenome;//染色体包含的基因组
	double fitness;//染色体适应度
	int chromoLength;//染色体长度----本文染色体统一长度为1，因为就一个基因“位置”
};


//---------- define a Population class-------//
class Population
{
public:
	std::vector<Chromo> vecPop;//定义一个种群
	friend class EGIReg;
	Population() : totalFitness(0), bestFitness(0), averageFitness(0), worstFitness(0),
		mutationRate(0), crossoverRate(0), leftMax(0), rightMax(0), MaxY(0),
		generationMax(0), maxStep(0) {
		vecPop.resize(500); srand((unsigned)time(NULL));
	}//初始化随机种子
	~Population() {}
private:
	int popSize;//种群里人口数量
				//一些适应率参数
	double totalFitness;
	double bestFitness;
	double averageFitness;
	double worstFitness;
	//最适合的个体
	Chromo fitnessChromo;
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



class EGIReg : public QObject
{
	Q_OBJECT

public:
	EGIReg();
	virtual ~EGIReg();

	//functions
	void params_initial();

	void translationEstimate();
	//Eigen::Matrix3d rotationEstimate();
	
	void computeCentroid(const PointCloudT::ConstPtr &cloud_in, PointCloudT::Ptr &cloud_out);
	void normalSphereCompute(const PointCloudT::ConstPtr &cloud_in, PointCloudT::Ptr &cloud_out);
	void mapToIcosahedron(PointCloudT::Ptr &normal_sphere, std::vector<int> *intensity, int EGI_level);
	void subdivide(Eigen::Vector3d v1,
		Eigen::Vector3d v2,
		Eigen::Vector3d v3, 
		long depth,
		std::vector< std::vector<Eigen::Vector3d> > triangle_vectors,
		std::vector<Eigen::Vector3d> vertex_indices);

	double computeCorrelation(float omega, float fai, float kappa);
	double correlation(std::vector<int> sphere_1, std::vector<int> sphere_2);

	void ns_visualization();
	void search(Eigen::Matrix4d &transformation);

	//Records
	Record *record;
	void recordToFile(QString filename, QString content);
	pcl::console::TicToc pcl_time;

	//input & output
	void setNormalSearchRadius(double);
	void setNormalLevel(int);
	void setNormalScale(double);

	void setModel(PointCloudT::Ptr &cloud_in);
	void setData(PointCloudT::Ptr &cloud_in);

	Eigen::Vector3d getTranslation();
	Eigen::Matrix3d getRotation();
	Eigen::Matrix4d getTransformation();

	PointCloudT::Ptr getModelNormalSphere();
	PointCloudT::Ptr getDataNormalSphere();

	PointCloudT::Ptr model;
	PointCloudT::Ptr data;

	PointCloudT::Ptr model_trans;
	PointCloudT::Ptr data_trans;

	PointCloudT::Ptr model_normal_sphere;
	PointCloudT::Ptr data_normal_sphere;

	int getProgress();


	//GA
	void Reset();//重置参数
				 //外界输入初始化参数、染色体初始化
	void Init(int popsize, double mutationrate, double crossoverrate, int generationmax,
		double maxstep, double leftmax, double rightmax);
	void ImplementGa();//执行遗传算法

private:
	bool preprogress;//判断平移点云和NS电云是否已计算



	//std::vector< std::vector<Eigen::Vector3d> > triangle_vectors;
	//std::vector<Eigen::Vector3i> face_indices;
	//std::vector<Eigen::Vector3d> vertex_indices;

	//normal estimate params
	double search_radius;
	int normal_level;
	double normal_scale;

	//rotation angle search params
	int EGI_level;
	//double step_size;
	//std::vector<double> intensity;

	//GUI
	int progress;

	////rotation angles
	//Eigen::Vector3f rot_angles;
	//double alpha;
	//double beta;
	//double gama;

	Eigen::Vector3d translation;
	Eigen::Matrix3d rotation;
	Eigen::Matrix4d transformation;

	//GA
	double Curve(Chromo input);//解码---x->F(x)
	double Random();//制造随机0-1的输
	Population popOperation;//定义一个可操作的种群-populationOperation	
	int generationCount;//记录杂交的代数
	void CalculateRate();//初始化软件计算的参数
	Chromo GenomeRoulette();//轮盘赌选择函数
	void Mutate(Chromo genome);//基因突变函数
	void Epoch(Population& newgeneration);//产生新的一代
	void Report();//输出到终端的信息

};
