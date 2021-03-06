#include "EGIReg.h"

#define ICO_X .525731112119133606
#define ICO_Z .850650808352039932

EGIReg::EGIReg()
{
	Record *record = new Record();
	preprogress = false;

	model.reset(new PointCloudT);
	data.reset(new PointCloudT);

	model_trans.reset(new PointCloudT);
	data_trans.reset(new PointCloudT);

	model_normal_sphere.reset(new PointCloudT);
	data_normal_sphere.reset(new PointCloudT);

	search_radius = 20.0;
	normal_level = 100;
	normal_scale = 20.0;
}


EGIReg::~EGIReg()
{
}

void
EGIReg::params_initial()
{
	preprogress = false;

	//model.reset(new PointCloudT);
	//data.reset(new PointCloudT);

	model_trans.reset(new PointCloudT);
	data_trans.reset(new PointCloudT);

	model_normal_sphere.reset(new PointCloudT);
	data_normal_sphere.reset(new PointCloudT);

	search_radius = 20.0;
	normal_level = 100;
	normal_scale = 20.0;
}

void
EGIReg::translationEstimate()
{
	Eigen::Vector4d centroid_model, centroid_data;
	pcl::compute3DCentroid(*model, centroid_model);
	pcl::compute3DCentroid(*data, centroid_data);

	//transform point cloud
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
	transformation_matrix(0, 0) = 1;
	transformation_matrix(1, 1) = 1;
	transformation_matrix(2, 2) = 1;
	transformation_matrix(0, 3) = -centroid_model(0);
	transformation_matrix(1, 3) = -centroid_model(1);
	transformation_matrix(2, 3) = -centroid_model(2);
	transformation_matrix(3, 3) = 1;
	pcl::transformPointCloud(*model, *model_trans, transformation_matrix, true);

	transformation_matrix(0, 3) = -centroid_data(0);
	transformation_matrix(1, 3) = -centroid_data(1);
	transformation_matrix(2, 3) = -centroid_data(2);
	pcl::transformPointCloud(*data, *data_trans, transformation_matrix, true);

	translation = centroid_model.head(3) - centroid_data.head(3);

	return;
}

void
EGIReg::computeCentroid(const PointCloudT::ConstPtr &cloud_in, PointCloudT::Ptr &cloud_out)
{
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloud_in, centroid);

	//transform point cloud
	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
	transformation_matrix(0, 0) = 1;
	transformation_matrix(1, 1) = 1;
	transformation_matrix(2, 2) = 1;
	transformation_matrix(0, 3) = -centroid(0);
	transformation_matrix(1, 3) = -centroid(1);
	transformation_matrix(2, 3) = -centroid(2);
	transformation_matrix(3, 3) = 1;
	pcl::transformPointCloud(*cloud_in, *cloud_out, transformation_matrix, true);

	return;
}

void
EGIReg::normalSphereCompute(const PointCloudT::ConstPtr &cloud_in, PointCloudT::Ptr &cloud_out)
{
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud_in);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);

    PointCloudN::Ptr cloud_normals(new PointCloudN);

	ne.setRadiusSearch(search_radius);

	ne.compute(*cloud_normals);

	if (cloud_normals->size() == 0)
	{
		return;
	}
	int size = cloud_normals->size();

	cloud_out->resize(size);

	for (int i = 0; i < size; ++i)
	{
		cloud_out->points[i].x = cloud_normals->at(i).normal_x * 100;
		cloud_out->points[i].y = cloud_normals->at(i).normal_y * 100;
		cloud_out->points[i].z = cloud_normals->at(i).normal_z * 100;
	}
}

void
EGIReg::mapToIcosahedron(PointCloudT::Ptr &normal_sphere, std::vector<int> *EGI_intensity, int EGI_level)
{
	std::vector< std::vector<Eigen::Vector3d> > triangle_vectors;
	std::vector<Eigen::Vector3i> face_indices;
	std::vector<Eigen::Vector3d> vertex_indices;
	/*************************Create Icosahedron****************************/
	vertex_indices.resize(12);
	vertex_indices = {
		{ -ICO_X, 0.0, ICO_Z },{ ICO_X, 0.0, ICO_Z },{ -ICO_X, 0.0, -ICO_Z },{ ICO_X, 0.0, -ICO_Z },
		{ 0.0, ICO_Z, ICO_X },{ 0.0, ICO_Z, -ICO_X },{ 0.0, -ICO_Z, ICO_X },{ 0.0, -ICO_Z, -ICO_X },
		{ ICO_Z, ICO_X, 0.0 },{ -ICO_Z, ICO_X, 0.0 },{ ICO_Z, -ICO_X, 0.0 },{ -ICO_Z, -ICO_X, 0.0 }
	};

	face_indices.resize(20);
	face_indices = {
		{ 1,4,0 },{ 4,9,0 },{ 4,5,9 },{ 8,5,4 },{ 1,8,4 },
		{ 1,10,8 },{ 10,3,8 },{ 8,3,5 },{ 3,2,5 },{ 3,7,2 },
		{ 3,10,7 },{ 10,6,7 },{ 6,11,7 },{ 6,0,11 },{ 6,1,0 },
		{ 10,1,6 },{ 11,0,9 },{ 2,11,9 },{ 5,2,9 },{ 11,2,7 }
	};

	//Mapping normals to Icosahedron
	int size = normal_sphere->size();
	//qDebug("size: %d", size);

	triangle_vectors.resize(20 * pow(4, EGI_level - 1));
	for (int k = 0; k < 20; k++)
	{
		//subdivide(vertex_indices.at(face_indices.at(k)[0]),
		//	vertex_indices.at(face_indices.at(k)[1]),
		//	vertex_indices.at(face_indices.at(k)[2]),
		//	EGI_level,
		//	triangle_vectors,
		//	vertex_indices);

		std::vector<Eigen::Vector3d> tri_vertex;
		tri_vertex.push_back(vertex_indices.at(face_indices.at(k)[0]));
		tri_vertex.push_back(vertex_indices.at(face_indices.at(k)[1]));
		tri_vertex.push_back(vertex_indices.at(face_indices.at(k)[2]));
		triangle_vectors[k] = tri_vertex;
	}
	EGI_intensity->resize(20 * pow(4, EGI_level - 1));

	for (int i = 0; i < EGI_intensity->size(); ++i)
	{
		Eigen::Vector3f lamda;
		Eigen::Vector3f normal_vector;
		Eigen::Matrix3f indice_matrix;
		indice_matrix << triangle_vectors.at(i).at(0)[0], triangle_vectors.at(i).at(1)[0], triangle_vectors.at(i).at(2)[0],
			triangle_vectors.at(i).at(0)[1], triangle_vectors.at(i).at(1)[1], triangle_vectors.at(i).at(2)[1],
			triangle_vectors.at(i).at(0)[2], triangle_vectors.at(i).at(1)[2], triangle_vectors.at(i).at(2)[2];
		EGI_intensity->at(i) = 0;
		for (int j = 0; j < size; ++j)
		{
			normal_vector << normal_sphere->points[j].x, normal_sphere->points[j].y, normal_sphere->points[j].z;
			lamda = indice_matrix.inverse() * normal_vector;
			if ((lamda[0] > 0) && (lamda[1] > 0) && (lamda[2] > 0))
			{
				EGI_intensity->at(i)++;
			}
		}
	}
	
}

void
EGIReg::subdivide(Eigen::Vector3d v1,
	Eigen::Vector3d v2,
	Eigen::Vector3d v3,
	long depth,
	std::vector< std::vector<Eigen::Vector3d> > triangle_vectors,
	std::vector<Eigen::Vector3d> vertex_indices)
{
	Eigen::Vector3d v12, v23, v31;
	std::vector<Eigen::Vector3d> tri_vertex;

	if (depth == 1)
	{
		tri_vertex.push_back(v1);
		tri_vertex.push_back(v2);
		tri_vertex.push_back(v3);
		triangle_vectors.push_back(tri_vertex);
		return;
	}

	for (int i = 0; i < 3; i++)
	{
		v12[i] = (v1[i] + v2[i]) / 2.0;
		vertex_indices.push_back(v12);

		v23[i] = (v2[i] + v3[i]) / 2.0;
		vertex_indices.push_back(v23);

		v31[i] = (v3[i] + v1[i]) / 2.0;
		vertex_indices.push_back(v31);
	}


	subdivide(v1, v12, v31, depth - 1, triangle_vectors, vertex_indices);
	subdivide(v2, v23, v12, depth - 1, triangle_vectors, vertex_indices);
	subdivide(v3, v31, v23, depth - 1, triangle_vectors, vertex_indices);
	subdivide(v12, v23, v31, depth - 1, triangle_vectors, vertex_indices);
}

double
EGIReg::correlation(std::vector<int> sphere_1, std::vector<int> sphere_2)
{
	Eigen::VectorXi s1 = Eigen::Map<Eigen::VectorXi, Eigen::Unaligned>(sphere_1.data(), sphere_1.size());
	Eigen::VectorXi s2 = Eigen::Map<Eigen::VectorXi, Eigen::Unaligned>(sphere_2.data(), sphere_2.size());
	int inner_product = s1.transpose()*s2;
	double corr;
	corr = inner_product / (s1.cast<double>().norm() * s2.cast<double>().norm() + 1e-6);
	//qDebug("inner product: %d", inner_product);
	//qDebug("s1.norm(): %f", s1.cast<double>().norm());
	return corr;
}

double 
EGIReg::computeCorrelation(float omega, float fai, float kappa)
{
	Eigen::AngleAxisd rollAngle(omega, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd yawAngle(fai, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd pitchAngle(kappa, Eigen::Vector3d::UnitX());

	Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

	Eigen::Matrix3d rotationMatrix = q.matrix();

	Eigen::Affine3d r(rotationMatrix);
	Eigen::Affine3d t(Eigen::Translation3d(0.0, 0.0, 0.0));
	Eigen::Matrix4d matrix = (t * r).matrix();
	
	PointCloudT::Ptr data_normal_sphere_trans(new PointCloudT);
	pcl::transformPointCloud(*data_normal_sphere, *data_normal_sphere_trans, matrix, true);

	std::vector<int> intensity_model, intensity_data;
	mapToIcosahedron(model_normal_sphere, &intensity_model, 1);
	mapToIcosahedron(data_normal_sphere_trans, &intensity_data, 1);
	//for (int k = 0; k < 20; k++)
	//{
	//	qDebug("sphere: %d, %d", intensity_model[k], intensity_data[k]);
	//}

	double corr = correlation(intensity_model, intensity_data);

	return corr;
}

void
EGIReg::ns_visualization()
{
	computeCentroid(model, model_trans);
	computeCentroid(data, data_trans);

	//translationEstimate();
	normalSphereCompute(model_trans, model_normal_sphere);
	normalSphereCompute(data_trans, data_normal_sphere);

	QFile file1("normals_model.txt");
	if (!file1.open(QIODevice::Truncate | QFile::ReadWrite | QFile::Text))
		emit record->statusUpdate("can't open normals_model.txt");
	QTextStream in1(&file1);
	int size1 = model_normal_sphere->size();
	for (int i = 0; i < size1; i++)
	{
		in1 << model_normal_sphere->points[i].x / 100 << " "
			<< model_normal_sphere->points[i].y / 100 << " "
			<< model_normal_sphere->points[i].z / 100 << endl;
	}
	file1.close();

	QFile file2("normals_data.txt");
	if (!file2.open(QIODevice::Truncate | QFile::ReadWrite | QFile::Text))
		emit record->statusUpdate("can't open normals_data.txt");
	QTextStream in2(&file2);
	int size2 = data_normal_sphere->size();
	for (int i = 0; i < size2; i++)
	{
		in2 << data_normal_sphere->points[i].x / 100 << " "
			<< data_normal_sphere->points[i].y / 100 << " "
			<< data_normal_sphere->points[i].z / 100 << endl;
	}
	file2.close();
	//preprogress = true;
}

void
EGIReg::search(Eigen::Matrix4d &transformation)
{
	Init(5, 0.8, 1, 50, 0.5, -PI, PI);
	//ImplementGa();

	double omega, fai, kappa;
	omega = popOperation.fitnessChromo.vecGenome[0];
	fai = popOperation.fitnessChromo.vecGenome[1];
	kappa = popOperation.fitnessChromo.vecGenome[2];

	Eigen::AngleAxisd rollAngle(omega, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd yawAngle(fai, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd pitchAngle(kappa, Eigen::Vector3d::UnitX());

	Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

	Eigen::Matrix3d rotationMatrix = q.matrix();

	Eigen::Affine3d r(rotationMatrix);
	Eigen::Affine3d t(Eigen::Translation3d(0.0, 0.0, 0.0));
	transformation = (t * r).matrix();
	
	return;
}


void
EGIReg::recordToFile(QString filename, QString content)
{
	QFile file(filename);
	if (!file.open(QIODevice::Truncate | QFile::ReadWrite | QFile::Text))
		emit qDebug("can't open EGI_intensity_data.txt");
	QTextStream in(&file);

	in << content << endl;

	file.close();
}


void
EGIReg::setNormalSearchRadius(double sr)
{
	search_radius = sr;
}

void
EGIReg::setNormalLevel(int nl)
{
	normal_level = nl;
}

void
EGIReg::setNormalScale(double ns)
{
	normal_scale = ns;
}

void
EGIReg::setModel(PointCloudT::Ptr &cloud_in)
{
	*model = *cloud_in;
	return;
}

void
EGIReg::setData(PointCloudT::Ptr &cloud_in)
{
	*data = *cloud_in;
	return;
}

Eigen::Vector3d
EGIReg::getTranslation()
{
	return translation;
}

Eigen::Matrix3d
EGIReg::getRotation()
{
	return rotation;
}

Eigen::Matrix4d
EGIReg::getTransformation()
{
	return transformation;
}

PointCloudT::Ptr 
EGIReg::getModelNormalSphere()
{
	if (!preprogress)
	{
		qDebug("PointCloud not available.");
	}
	return model_normal_sphere;
}

PointCloudT::Ptr
EGIReg::getDataNormalSphere()
{
	if (!preprogress)
	{
		qDebug("PointCloud not available.");
	}
	return data_normal_sphere;
}
int
EGIReg::getProgress()
{
	return progress;
}


//GA part

double
EGIReg::Curve(Chromo input)
{
	double omega, fai, kappa;
	omega = input.vecGenome[0];
	fai = input.vecGenome[1];
	kappa = input.vecGenome[2];

	double y = computeCorrelation(omega, fai, kappa);
	return y;
}

double 
EGIReg::Random()
{
	double random;
	random = (1.0*rand()) / (RAND_MAX + 1);
	return random;
}

void
EGIReg::Reset()
{
	generationCount = 0;
	popOperation.vecPop.clear();
}

void 
EGIReg::Init(int popsize, double mutationrate, double crossoverrate, int generationmax,
	double maxstep, double leftmax, double rightmax)
{
	popOperation.popSize = popsize;
	popOperation.mutationRate = mutationrate;
	popOperation.crossoverRate = crossoverrate;
	popOperation.generationMax = generationmax;
	popOperation.maxStep = maxstep;
	popOperation.leftMax = leftmax;
	popOperation.rightMax = rightmax;
	double k = 0.0;
	//初始化种群中染色体的基因--随机方式
	for (int i = 0; i < popOperation.popSize; i++)
	{
		k = Random();
		
		for (unsigned int j = 0; j < popOperation.vecPop[i].chromoLength; j++)
		{
			popOperation.vecPop[i].vecGenome[j] = k*(rightmax - leftmax) + leftmax;
		}
	}
}

void
EGIReg::CalculateRate()
{
	double Ydata = 0.0, AllRate = 0.0, Yrate = 0.0;
	double maxRate = 0.0, minRate = 0.0, maxData = 0.0;

	Ydata = Curve(popOperation.vecPop[0]);
	Yrate = Ydata;
	popOperation.vecPop[0].fitness = Yrate;
	maxRate = Yrate;
	minRate = Yrate;
	AllRate = Yrate;
	maxData = Ydata;
	for (int i = 1; i < popOperation.popSize; i++)
	{
		Ydata = Curve(popOperation.vecPop[i]);
		Yrate = Ydata;
		popOperation.vecPop[i].fitness = Yrate;
		AllRate = AllRate + Yrate;
		if (maxRate < Yrate)
		{
			maxRate = Yrate;
			maxData = Ydata;
			popOperation.fitnessChromo = popOperation.vecPop[i];
		}
		if (minRate > Yrate)
			minRate = Yrate;
	}
#if false
	double maxY = 0.0, absMax = 0.0, Max = 0.0, AllMax = 0.0, test = 0.0;
	double maxRate = 0.0, minRate = 0.0;
	for (int i = 0; i < popOperation.popSize; i++)//解码
	{
		test = Curve(popOperation.vecPop[i]);
		Max = Max + test;
		if (test < 0) test = -test;
		absMax = absMax + test;
	}
	AllMax = Max + popOperation.popSize * absMax;
	test = Curve(popOperation.vecPop[0]);
	popOperation.vecPop[0].fitness = (absMax + test) / (AllMax);
	maxRate = popOperation.vecPop[0].fitness;
	minRate = popOperation.vecPop[0].fitness;
	for (int i = 1; i < popOperation.popSize; i++)
	{
		test = Curve(popOperation.vecPop[i]);
		popOperation.vecPop[i].fitness = (absMax + test) / (AllMax);
		if (maxRate < popOperation.vecPop[i].fitness)
		{
			maxRate = popOperation.vecPop[i].fitness;
			maxY = Curve(popOperation.vecPop[i]);
		}
		if (minRate > popOperation.vecPop[i].fitness)
			minRate = popOperation.vecPop[i].fitness;
	}
#endif
	popOperation.bestFitness = maxRate;
	popOperation.worstFitness = minRate;
	popOperation.MaxY = maxData;
	popOperation.totalFitness = AllRate;
	Report();
}

Chromo
EGIReg::GenomeRoulette()
{
	double chooseRate = 0.0;
	Chromo chooseChromo;
	double randomLimit = 0.0;
	randomLimit = Random()*popOperation.totalFitness;
	for (int i = 0; i < popOperation.popSize; i++)
	{
		chooseRate = chooseRate + popOperation.vecPop[i].fitness;
		if (chooseRate > randomLimit)
		{
			chooseChromo = popOperation.vecPop[i];
			break;
		}
	}
	return chooseChromo;
}

void
EGIReg::Mutate(Chromo genome)
{
	double mutateData = 0.0;
	for (int i = 0; i < popOperation.popSize; i++)
	{
		for (unsigned int j = 0; j < popOperation.vecPop[i].chromoLength; j++)
		{
			mutateData = popOperation.vecPop[i].vecGenome[j];
			if (Random() < popOperation.mutationRate)
				popOperation.vecPop[i].vecGenome[j] = (Random() - 0.5)*popOperation.maxStep + mutateData;
			if (popOperation.vecPop[i].vecGenome[j] > popOperation.rightMax)
				popOperation.vecPop[i].vecGenome[j] = popOperation.rightMax;
			if (popOperation.vecPop[i].vecGenome[j] < popOperation.leftMax)
				popOperation.vecPop[i].vecGenome[j] = popOperation.leftMax;
		}
	}
}

void 
EGIReg::Epoch(Population& newgeneration)
{
	double x = 0, y = 0;
	std::vector<Chromo> tempGeneration;
	CalculateRate();
	tempGeneration = popOperation.vecPop;
	Chromo MotherChromo, FatherChromo, GirlChromo, BoyChromo;
	for (int i = 0; i < popOperation.popSize; i += 2)
	{
		MotherChromo = GenomeRoulette();
		FatherChromo = GenomeRoulette();
		//进行交叉变异
		double crossover = MotherChromo.vecGenome[1];
		MotherChromo.vecGenome[1] = FatherChromo.vecGenome[1];
		FatherChromo.vecGenome[1] = crossover;
		GirlChromo = MotherChromo;
		BoyChromo = FatherChromo;
		//进行基因突变了
		Mutate(GirlChromo);
		Mutate(BoyChromo);
		popOperation.vecPop[i].vecGenome = GirlChromo.vecGenome;
		popOperation.vecPop[i + 1].vecGenome = BoyChromo.vecGenome;
	}
	//newgeneration.vecPop = tempGeneration;
}

void
EGIReg::ImplementGa()
{
	for (int i = 0; i < popOperation.generationMax + 1; i++)
	{

		Epoch(popOperation);
		generationCount++;
	}
}

void
EGIReg::Report()
{
	qDebug("GenerationCount: %d.", generationCount);
	//qDebug("bestFitness: %f.", popOperation.bestFitness);
	//qDebug("worstFitness: %f.", popOperation.worstFitness);
	qDebug("Max Correlation: %f.", popOperation.MaxY);
}