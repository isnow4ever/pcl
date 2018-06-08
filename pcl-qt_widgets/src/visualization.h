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
class Chromo2//����һ��Ⱦɫ��
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
	std::vector<double> vecGenome;//Ⱦɫ������Ļ�����
	double fitness;//Ⱦɫ����Ӧ��
	int chromoLength;//Ⱦɫ�峤��----����Ⱦɫ��ͳһ����Ϊ6����Ϊ��һ������λ�á�
};


//---------- define a Population class-------//
class Population2
{
public:
	std::vector<Chromo2> vecPop;//����һ����Ⱥ
	friend class Visualization;
	Population2() : totalFitness(0), bestFitness(0), averageFitness(0), worstFitness(0),
		mutationRate(0), crossoverRate(0), leftMax(0), rightMax(0), MaxY(0),
		generationMax(0), maxStep(0) {
		vecPop.resize(500); srand((unsigned)time(NULL));
	}//��ʼ���������
	~Population2() {}
private:
	int popSize;//��Ⱥ���˿�����
				//һЩ��Ӧ�ʲ���
	double totalFitness;
	double bestFitness;
	double averageFitness;
	double worstFitness;
	//���ʺϵĸ���
	Chromo2 fitnessChromo;
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

//class GA : public QObject
//{
//public:
//	//GA
//	void Reset();//���ò���
//				 //��������ʼ��������Ⱦɫ���ʼ��
//	void Init(int popsize, double mutationrate, double crossoverrate, int generationmax,
//		double maxstep, double leftmax, double rightmax);
//	void ImplementGa();//ִ���Ŵ��㷨
//
//
//	Eigen::Vector3d translation;
//	Eigen::Matrix3d rotation;
//	Eigen::Matrix4d transformation;
//
//private:
//	//GA
//	double Curve(Chromo2 input);//����---x->F(x)
//	double Random();//�������0-1����
//	Population2 popOperation;//����һ���ɲ�������Ⱥ-populationOperation	
//	int generationCount;//��¼�ӽ��Ĵ���
//	void CalculateRate();//��ʼ���������Ĳ���
//	Chromo2 GenomeRoulette();//���̶�ѡ����
//	void Mutate(Chromo2 genome);//����ͻ�亯��
//	void Epoch(Population2& newgeneration);//�����µ�һ��
//	void Report();//������ն˵���Ϣ
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
	void Reset();//���ò���
				 //��������ʼ��������Ⱦɫ���ʼ��
	void Init(int popsize, double mutationrate, double crossoverrate, int generationmax,
		double maxstep, double leftmax, double rightmax);
	void ImplementGa();//ִ���Ŵ��㷨


	//Eigen::Vector3f translation;
	//Eigen::Matrix3f rotation;
	Eigen::Matrix4f init_transform;

private:
	//GA
	double Curve(Chromo2 input);//����---x->F(x)
	double Random();//�������0-1����
	Population2 popOperation;//����һ���ɲ�������Ⱥ-populationOperation	
	int generationCount;//��¼�ӽ��Ĵ���
	void CalculateRate();//��ʼ���������Ĳ���
	Chromo2 GenomeRoulette();//���̶�ѡ����
	void Mutate(Chromo2 genome);//����ͻ�亯��
	void Epoch(Population2& newgeneration);//�����µ�һ��
	void Report();//������ն˵���Ϣ

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
