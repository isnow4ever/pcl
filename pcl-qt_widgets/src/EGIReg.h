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
class Chromo//����һ��Ⱦɫ��
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
	std::vector<double> vecGenome;//Ⱦɫ������Ļ�����
	double fitness;//Ⱦɫ����Ӧ��
	int chromoLength;//Ⱦɫ�峤��----����Ⱦɫ��ͳһ����Ϊ1����Ϊ��һ������λ�á�
};


//---------- define a Population class-------//
class Population
{
public:
	std::vector<Chromo> vecPop;//����һ����Ⱥ
	friend class EGIReg;
	Population() : totalFitness(0), bestFitness(0), averageFitness(0), worstFitness(0),
		mutationRate(0), crossoverRate(0), leftMax(0), rightMax(0), MaxY(0),
		generationMax(0), maxStep(0) {
		vecPop.resize(500); srand((unsigned)time(NULL));
	}//��ʼ���������
	~Population() {}
private:
	int popSize;//��Ⱥ���˿�����
				//һЩ��Ӧ�ʲ���
	double totalFitness;
	double bestFitness;
	double averageFitness;
	double worstFitness;
	//���ʺϵĸ���
	Chromo fitnessChromo;
	//ͻ�����
	double mutationRate;
	//�������
	double crossoverRate;
	//���ܵĴ���
	int generationMax;
	//������Ĳ���
	double maxStep;
	//�߽�ֵ
	double leftMax;
	double rightMax;
	//�������ֵ
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
	void Reset();//���ò���
				 //��������ʼ��������Ⱦɫ���ʼ��
	void Init(int popsize, double mutationrate, double crossoverrate, int generationmax,
		double maxstep, double leftmax, double rightmax);
	void ImplementGa();//ִ���Ŵ��㷨

private:
	bool preprogress;//�ж�ƽ�Ƶ��ƺ�NS�����Ƿ��Ѽ���



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
	double Curve(Chromo input);//����---x->F(x)
	double Random();//�������0-1����
	Population popOperation;//����һ���ɲ�������Ⱥ-populationOperation	
	int generationCount;//��¼�ӽ��Ĵ���
	void CalculateRate();//��ʼ���������Ĳ���
	Chromo GenomeRoulette();//���̶�ѡ����
	void Mutate(Chromo genome);//����ͻ�亯��
	void Epoch(Population& newgeneration);//�����µ�һ��
	void Report();//������ն˵���Ϣ

};
