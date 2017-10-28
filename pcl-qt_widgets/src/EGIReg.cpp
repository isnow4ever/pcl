#include "EGIReg.h"

#define ICO_X .525731112119133606
#define ICO_Z .850650808352039932

EGIReg::EGIReg(PointCloudT::Ptr cloud_model, PointCloudT::Ptr cloud_data)
	:model(cloud_model), data(cloud_data)
{
	Record *record = new Record();
}


EGIReg::~EGIReg()
{
}

void
EGIReg::params_initial()
{
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

Eigen::Vector4f
EGIReg::translationEstimate()
{
	record->statusUpdate("Translation Estimate...");
	pcl_time.tic();
	Eigen::Vector4f centroid_model, centroid_data;
	pcl::compute3DCentroid(*model, centroid_model);
	pcl::compute3DCentroid(*data, centroid_data);

	//transform point cloud
	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
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

	translation = centroid_model - centroid_data;

	record->statusUpdate("Translation Estimate completed in " + QString::number(pcl_time.toc()) + "ms;");

	return translation;
}

Eigen::Matrix3f
EGIReg::rotationEstimate()
{

}

void
EGIReg::normalSphereCompute(PointCloudT::Ptr cloud_in, PointCloudT::Ptr cloud_out)
{
	record->statusUpdate("Compute Normal Sphere...");
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud_in);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);

    PointCloudN::Ptr cloud_normals(new PointCloudN);

	ne.setRadiusSearch(search_radius);

	ne.compute(*cloud_normals);

	if (cloud_normals->size() == 0)
	{
		record->statusUpdate("no cloud normals!");
		return;
	}
	int size = cloud_normals->size();

	cloud_out->resize(size);

	for (int i = 0; i < size; ++i)
	{
		cloud_out->points[i].x = cloud_normals->at(i).normal_x;
		cloud_out->points[i].y = cloud_normals->at(i).normal_y;
		cloud_out->points[i].z = cloud_normals->at(i).normal_z;
		record->progressBarUpdate(int(100 * i / size));
	}
}

void
EGIReg::mapToIcosahedron(PointCloudT::Ptr normal_sphere, std::vector<double> EGI_intensity, int EGI_level)
{
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
	triangle_vectors.empty();
	for (int k = 0; k < 20; k++)
	{
		subdivide(vertex_indices.at(face_indices.at(k)[0]), vertex_indices.at(face_indices.at(k)[1]), vertex_indices.at(face_indices.at(k)[2]), EGI_level);
	}
	EGI_intensity.resize(20 * 4^(EGI_level - 1));
	EGI_intensity = { 0.0 };

	for (int i = 0; i < EGI_intensity.size(); ++i)
	{
		Eigen::Vector3f lamda;
		Eigen::Vector3f normal_vector;
		Eigen::Matrix3f indice_matrix;
		indice_matrix << triangle_vectors.at(i).at(0)[0], triangle_vectors.at(i).at(1)[0], triangle_vectors.at(i).at(2)[0],
			triangle_vectors.at(i).at(0)[1], triangle_vectors.at(i).at(1)[1], triangle_vectors.at(i).at(2)[1],
			triangle_vectors.at(i).at(0)[2], triangle_vectors.at(i).at(1)[2], triangle_vectors.at(i).at(2)[2];

		for (int j = 0; j < size; ++j)
		{
			normal_vector << normal_sphere->points[j].x, normal_sphere->points[j].y, normal_sphere->points[j].z;
			lamda = indice_matrix.inverse() * normal_vector;
			if ((lamda[0] > 0) && (lamda[1] > 0) && (lamda[2] > 0))
			{
				EGI_intensity[i]++;

				record->progressBarUpdate(int(100 * (i * size + j) / (20 * size)));
			}
		}
	}
	
}

void
EGIReg::subdivide(Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d v3, long depth)
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


	subdivide(v1, v12, v31, depth - 1);
	subdivide(v2, v23, v12, depth - 1);
	subdivide(v3, v31, v23, depth - 1);
	subdivide(v12, v23, v31, depth - 1);
}

void
EGIReg::search(Eigen::Matrix4f &transformation)
{
	return;
}


void
EGIReg::recordToFile(QString filename, QString content)
{
	QFile file(filename);
	if (!file.open(QIODevice::Truncate | QFile::ReadWrite | QFile::Text))
		emit record->statusUpdate("can't open EGI_intensity_data.txt");
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


Eigen::Vector3f
EGIReg::getTranslation()
{
	return translation;
}

Eigen::Matrix3f
EGIReg::getRotation()
{
	return rotation;
}

Eigen::Matrix4f
EGIReg::getTransformation()
{
	return transformation;
}


int
EGIReg::getProgress()
{
	return progress;
}
