#include "optimalReg.h"


using namespace pcl;
using namespace std;
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;


OptimalRegistration::OptimalRegistration()
{
	surface_model.reset(new PointCloudT);
	surface_data.reset(new PointCloudT);
	
	datum_data.reset(new PointCloudT);
	datum_model.reset(new PointCloudT);

	model_with_normals.reset(new PointCloudPN);
	data_with_normals.reset(new PointCloudPN);
	surface_data_with_normals.reset(new PointCloudPN);

	enveloped_rate = 0.0;
}


OptimalRegistration::~OptimalRegistration()
{
}

bool
OptimalRegistration::computePointToPlaneDistance(const PointCloudT::Ptr cloud,
	const std::vector<double> &plane,
	std::vector<double> &distance)
{
	if (cloud->size() == 0)
	{
		qDebug("Invalid cloud input!");
		return false;
	}
		

	double dt = 0.0;
	double mA, mB, mC, mD, mX, mY, mZ;
	//distance.resize(surface_model->size());
	mA = plane[0];
	mB = plane[1];
	mC = plane[2];
	mD = plane[3];
	//qDebug("plane: %f, %f, %f, %f", plane[0], plane[1], plane[2], plane[3]);
	for (int i = 0; i < cloud->points.size(); i++)
	{
		mX = cloud->points[i].x;
		mY = cloud->points[i].y;
		mZ = cloud->points[i].z;

		if (mA*mA + mB*mB + mC*mC)
		{
			dt = abs(mA*mX + mB*mY + mC*mZ + mD) / sqrt(mA*mA + mB*mB + mC*mC);
			distance.push_back(dt);
		}
		else
			return false;
	}
	//qDebug("distance calculated!");
	return true;
}

bool
OptimalRegistration::createSlicingPlanes(const std::vector<double> &datum_plane,
	const PointCloudT::Ptr cloud,
	std::vector< std::vector<double> > &slicing_planes,
	int number)
{	
	std::vector<double> points_distance;
	computePointToPlaneDistance(cloud, datum_plane, points_distance);

	if (points_distance.size() == 0)
	{
		qDebug("invalid points_distance!");
		return false;
	}		

	int max_dist_index;
	std::vector<double>::iterator biggest = std::max_element(std::begin(points_distance), std::end(points_distance));
	max_dist_index = std::distance(std::begin(points_distance), biggest);
	double mA, mB, mC, mD;
	mA = datum_plane[0];
	mB = datum_plane[1];
	mC = datum_plane[2];
	mD = datum_plane[3];
	std::vector< std::vector<double> > planes;

	double step = (*biggest / (number + 1)) * sqrt(mA*mA + mB*mB + mC*mC);
	
	for (int i = 1; i <= number; i++)
	{
		std::vector<double> plane = {mA, mB, mC, mD - i*step};
		planes.push_back(plane);
		//qDebug("planes: %f, %f, %f, %f.", mA, mB, mC, mD - i*step);
	}
	slicing_planes = planes;

	return true;
}

bool
OptimalRegistration::estimatePointsBySlicing(const std::vector<double> &slicing_plane,
	const PointCloudT::Ptr cloud_model,
	const PointCloudT::Ptr cloud_data,
	const double epsilon,
	PointCloudT::Ptr &points_on_plane_m,
	PointCloudT::Ptr &points_on_plane_d) 
{
	points_on_plane_m.reset(new PointCloudT);
	points_on_plane_d.reset(new PointCloudT);

	std::vector<double> model_dist;
	std::vector<double> data_dist;
	PointCloudT::Ptr filtered_model(new PointCloudT);
	PointCloudT::Ptr filtered_data(new PointCloudT);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = slicing_plane[0];
	coefficients->values[1] = slicing_plane[1];
	coefficients->values[2] = slicing_plane[2];
	coefficients->values[3] = slicing_plane[3];

	//qDebug("coefficients: %f, %f, %f, %f", coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);

	computePointToPlaneDistance(cloud_model, slicing_plane, model_dist);
	computePointToPlaneDistance(cloud_data, slicing_plane, data_dist);

	// Create the filtering object
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setModelCoefficients(coefficients);
	PointCloudT::Ptr projected_points(new PointCloudT);
	int counts = 0;
	//qDebug("hello counts: %d", counts);
	projected_points.reset(new PointCloudT);
	for (int j = 0; j < cloud_model->points.size(); j++)
	{		
		if (model_dist[j] < epsilon)
		{
			filtered_model->push_back(cloud_model->points[j]);
			//qDebug("counts: %d", counts++);
		}		
	}
	proj.setInputCloud(filtered_model);
	proj.filter(*projected_points);
	points_on_plane_m = projected_points;
	
	projected_points.reset(new PointCloudT);
	for (int k = 0; k < cloud_data->points.size(); k++)
	{
		if (data_dist[k] < epsilon)
		{
			filtered_data->push_back(cloud_data->points[k]);
		}
	}
	proj.setInputCloud(filtered_data);
	proj.filter(*projected_points);
	points_on_plane_d = projected_points;

	//qDebug("Estimate Sliced Points!");
	return true;
}

bool
OptimalRegistration::computeConvexHull(const PointCloudT::Ptr points_on_plane,
	PointCloud<PointXY>::Ptr &chull_points,
	std::vector<pcl::Vertices> &polygons)
{
	PointCloud<PointXY>::Ptr points2D(new PointCloud<PointXY>);
	for (int i = 0; i < points_on_plane->points.size(); i++)
	{
		PointXY point;
		point.x = points_on_plane->points[i].x;
		point.y = points_on_plane->points[i].y;
		points2D->push_back(point);
	}
	// Create a Convex Hull representation of the projected inliers
	chull_points.reset(new PointCloud<PointXY>);
	PointCloud<PointXYZ>::Ptr chull_points3D(new PointCloud<PointXYZ>);
	pcl::ConvexHull<pcl::PointXYZ> chull;
	chull.setInputCloud(points_on_plane);
	//chull.setDimension(3);
	chull.reconstruct(*chull_points3D, polygons);
	chull_points3D = points_on_plane;//不求凸包
	for (int j = 0; j < chull_points3D->points.size(); j++)
	{
		pcl::PointXY p;
		p.x = chull_points3D->points[j].x;
		p.y = chull_points3D->points[j].y;
		chull_points->push_back(p);
	}
	return true;
}

bool
OptimalRegistration::estimateInnerPoint(const pcl::PointXY point, const PointCloud<PointXY>::Ptr chull_points_outside)
{
	double px, py, sx, sy, tx, ty;
	double sum = 0.0;
	px = point.x;
	py = point.y;

	for (int m = 0, l = chull_points_outside->points.size(), n = l - 1; m < l; n = m, m++)
	{
		sx = chull_points_outside->points[m].x;
		sy = chull_points_outside->points[m].y;
		tx = chull_points_outside->points[n].x;
		ty = chull_points_outside->points[n].y;

		// 点与多边形顶点重合或在多边形的边上
		if ((sx - px) * (px - tx) >= 0 && (sy - py) * (py - ty) >= 0 && (px - sx) * (ty - sy) == (py - sy) * (tx - sx))
		{
			return true;
		}
		// 点与相邻顶点连线的夹角
		double angle = atan2(sy - py, sx - px) - atan2(ty - py, tx - px);

		// 确保夹角不超出取值范围（-π 到 π）
		if (angle >= PI)
			angle = angle - PI * 2;
		else if (angle <= -PI)
			angle = angle + PI * 2;

		sum += angle;
	}

		if (round(sum / PI) == 0)
			return false;
		else
			return true;
}

bool
OptimalRegistration::estimateEveloped(const double probability, std::vector<double> &dist)
{
	if ((surface_model->points.size() == 0) || (surface_data->points.size() == 0))
	{
		qDebug("Invalid cloud input!");
		return false;
	}
	
	std::vector<double> datum_plane = { 0.0, 0.0, 1.0, 0.0 };
	std::vector< std::vector<double> > slicing_planes;
	int layers = 10;
	createSlicingPlanes(datum_plane, surface_model, slicing_planes, layers);
	//for (int i = 0; i < slicing_planes.size(); i++)
	//{
	//	qDebug("slicing_planes: %f, %f, %f, %f", slicing_planes[i][0], slicing_planes[i][1], slicing_planes[i][2], slicing_planes[i][3]);
	//}
	
	PointCloud<PointXY>::Ptr chull_points_inside;
	PointCloud<PointXY>::Ptr chull_points_outside;
	std::vector<pcl::Vertices> polygons_inside;
	std::vector<pcl::Vertices> polygons_outside;
	int _enveloped_count = 0;
	int total_count = 0;
	double _enveloped_rate;

	for (int i = 0; i < layers; i++)
	{
		PointCloudT::Ptr points_on_planes_m(new PointCloudT);
		PointCloudT::Ptr points_on_planes_d(new PointCloudT);
		estimatePointsBySlicing(slicing_planes[i], surface_model, surface_data, 1.0, points_on_planes_m, points_on_planes_d);
		computeConvexHull(points_on_planes_m, chull_points_inside, polygons_inside);
		computeConvexHull(points_on_planes_d, chull_points_outside, polygons_outside);
		points_on_planes_m.reset(new PointCloudT);
		for (int j = 0; j < chull_points_inside->points.size(); j++)
		{
			bool _enveloped_point = estimateInnerPoint(chull_points_inside->points[j], chull_points_outside);
			if (_enveloped_point)
				_enveloped_count++;
			PointXYZ pt(chull_points_inside->points[j].x, chull_points_inside->points[j].x, 0.0);
			points_on_planes_m->push_back(pt);
		}
		total_count += chull_points_inside->points.size();

		//PointCloud<Normal>::Ptr normals(new PointCloudN);
		//pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
		//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		//ne.setNumberOfThreads(4);
		//ne.setSearchMethod(tree);
		//ne.setRadiusSearch(10);
		//ne.setInputCloud(points_on_planes_m);
		//ne.compute(*normals);

		points_on_planes_d.reset(new PointCloudT);
		for (int l = 0; l < chull_points_outside->points.size(); l++)
		{
			PointXYZ pt(chull_points_outside->points[l].x, chull_points_outside->points[l].x, 0.0);
			points_on_planes_d->push_back(pt);
		}
		float resolution = 128.0f;
		pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
		octree.setInputCloud(points_on_planes_d);
		octree.addPointsFromInputCloud();
		int K = 1;
		std::vector<int> pointIdxNKNSearch;
		std::vector<float> pointNKNSquaredDistance;
		for (int k = 0; k < points_on_planes_m->points.size(); k++)
		{
			octree.nearestKSearch(points_on_planes_m->points[k], K, pointIdxNKNSearch, pointNKNSquaredDistance);
			//Eigen::Vector2d yi(chull_points_outside->points[pointIdxNKNSearch[0]].x, chull_points_outside->points[pointIdxNKNSearch[0]].y);
			//Eigen::Vector2d xi(chull_points_inside->points[i].x, chull_points_inside->points[i].y);
			//Eigen::Vector2d w(normals->points[i].normal_x, normals->points[i].normal_y);
			//Eigen::Vector2d v = yi - xi;
			//double e = v.dot(w);
			dist.push_back(sqrt(pointNKNSquaredDistance[0]));
		}
	}
	_enveloped_rate = (double)_enveloped_count / total_count;
	enveloped_rate = _enveloped_rate;
	if (_enveloped_rate < probability)
		return false;
	else
		return true;

}

bool
OptimalRegistration::setModelCloud(PointCloudT::Ptr cloud)
{
	surface_model.reset(new PointCloudT);
	surface_model = cloud;
	return true;
}

bool
OptimalRegistration::setDataCloud(PointCloudT::Ptr cloud)
{
	surface_data.reset(new PointCloudT);
	surface_data = cloud;
	return true;
}

bool
OptimalRegistration::setDataNormalCloud(PointCloudPN::Ptr cloud)
{
	data_with_normals.reset(new PointCloudPN);
	data_with_normals = cloud;
	return true;
}

bool
OptimalRegistration::setSurfaceDataNormalCloud(PointCloudPN::Ptr cloud)
{
	surface_data_with_normals.reset(new PointCloudPN);
	surface_data_with_normals = cloud;
	return true;
}

double 
OptimalRegistration::getEnvelopedRate()
{
	return enveloped_rate;
}

double
OptimalRegistration::computeFitness(Eigen::Matrix4d &transformation)
{
	PointCloud<PointNormal>::Ptr transformed_data(new PointCloud<PointNormal>);
	PointCloud<PointXYZ>::Ptr transformed_surface(new PointCloud<PointXYZ>);
	PointCloud<PointNormal>::Ptr transformed_surface_with_normals(new PointCloud<PointNormal>);
	pcl::transformPointCloudWithNormals(*data_with_normals, *transformed_data, transformation);
	pcl::transformPointCloudWithNormals(*surface_data_with_normals, *transformed_surface_with_normals, transformation);
	for (size_t i = 0; i < transformed_surface_with_normals->points.size(); ++i)
	{
		const pcl::PointNormal &mls_pt = transformed_surface_with_normals->points[i];
		pcl::PointXYZ pt(mls_pt.x, mls_pt.y, mls_pt.z);
		transformed_surface->push_back(pt);
	}

	pcl::ModelCoefficients::Ptr coeff_data(new pcl::ModelCoefficients);
	pcl::ModelCoefficients::Ptr coeff_model(new pcl::ModelCoefficients);

	//qDebug("datum plane of data: %f, %f, %f, %f\n", coeff_data->values[0], coeff_data->values[1], coeff_data->values[2], coeff_data->values[3]);
	//qDebug("datum plane of model: %f, %f, %f, %f\n", coeff_model->values[0], coeff_model->values[1], coeff_model->values[2], coeff_model->values[3]);

	double alpha, beta;
	double datum_error, dist_variance, enveloped_rate;
	double fitness;
	std::vector<double> dist;

	alpha = 0.2;
	beta = 0.8;

	OptimalRegistration optReg;
	optReg.setModelCloud(surface_model);
	optReg.setDataCloud(transformed_surface);
	bool _enveloped = optReg.estimateEveloped(0.7, dist);
	qDebug("enveloped_rate: %f.", optReg.getEnvelopedRate());

	//enveloped_rate = enveloped(surface_model, normals_surface, transformed_surface, dist);
	//
	//return enveloped_rate;

	if (_enveloped)
	{
		//===============NormalEstimation========================//
		//pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
		//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		//ne.setSearchMethod(tree);
		//ne.setRadiusSearch(30);
		//ne.setInputCloud(transformed_data);
		//ne.compute(*normals_data);
		//===============DatumEstimation========================//

		computeDatumCoefficients(model_with_normals, datum_model, coeff_model);
		computeDatumCoefficients(transformed_data, datum_data, coeff_data);
		//===============ComputefFitness========================//
		pcl::Normal datum_normal;
		double mA, mB, mC;
		mA = coeff_model->values[0];
		mB = coeff_model->values[1];
		mC = coeff_model->values[2];
		datum_normal.normal_x = mA / sqrt(mA*mA + mB*mB + mC*mC);
		datum_normal.normal_y = mB / sqrt(mA*mA + mB*mB + mC*mC);
		datum_normal.normal_z = mC / sqrt(mA*mA + mB*mB + mC*mC);
		datum_error = computeDatumError(datum_model, datum_data, datum_normal);
		dist_variance = computeSurfaceVariance(dist);
		double sigmoid_e = 1.f / (1.f + sqrt(0.01f * datum_error));
		double sigmoid_v = 1.f / (1.f + sqrt(dist_variance));
		fitness = alpha * sigmoid_e + beta * sigmoid_v;
		qDebug("Daturm Error: %f. Distance Var: %f. Fitness: %f.", datum_error, dist_variance, fitness);
		return fitness;
	}
	else
	{
		return 0.0;
	}
}

void
OptimalRegistration::computeDatumCoefficients(PointCloud<PointNormal>::Ptr cloud, PointCloud<PointXYZ>::Ptr datum_plane, pcl::ModelCoefficients::Ptr coefficients)
{
	PointCloud<PointNormal>::Ptr filtered_cloud(new PointCloud<PointNormal>);
	// Create the filtering object
	pcl::PassThrough<pcl::PointNormal> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-10, 10);

	pass.filter(*filtered_cloud);
	//pass.setFilterLimitsNegative (true);


	//================segmentation=========================//
	// segmentation
	//pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	PointCloudN::Ptr filtered_normals(new PointCloudN);
	PointCloudT::Ptr filtered_points(new PointCloudT);
	for (size_t i = 0; i < filtered_cloud->points.size(); ++i)
	{
		const pcl::PointNormal &mls_pt = filtered_cloud->points[i];
		pcl::PointXYZ pt(mls_pt.x, mls_pt.y, mls_pt.z);
		pcl::Normal pn(mls_pt.normal_x, mls_pt.normal_y, mls_pt.normal_z);
		filtered_points->push_back(pt);
		filtered_normals->push_back(pn);
	}
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> sac;
	sac.setInputCloud(filtered_points);
	sac.setInputNormals(filtered_normals);
	sac.setMethodType(pcl::SAC_RANSAC);
	sac.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
	//sac.setNormalDistanceWeight(0.1);
	sac.setAxis(Eigen::Vector3f(0, 0, 1));
	sac.setEpsAngle(0.1);
	sac.setDistanceThreshold(0.1);
	sac.setMaxIterations(100);
	sac.setProbability(0.95);

	sac.segment(*inliers, *coefficients);

	// extract the certain field	 
	pcl::ExtractIndices<pcl::PointXYZ> ei;
	ei.setIndices(inliers);
	ei.setInputCloud(filtered_points);
	ei.setNegative(false);
	ei.filter(*datum_plane);
	//pcl::ExtractIndices<pcl::PointXYZ> ei2;
	//ei2.setIndices(inliers);
	//ei2.setInputCloud(cloud);
	//ei2.setNegative(false);
	//ei2.filter(*surface);


	return;
};

double
OptimalRegistration::computeDatumError(PointCloud<PointXYZ>::Ptr datum_model, PointCloud<PointXYZ>::Ptr datum_data, Normal &normal)
{
	double datum_error = 0.0;
	float resolution = 128.0f;
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
	octree.setInputCloud(datum_data);
	octree.addPointsFromInputCloud();
	int K = 2;
	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;
	int scale = datum_model->size();
	for (int i = 0; i < scale; i++)
	{
		octree.nearestKSearch(datum_model->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance);
		Eigen::Vector3d yi(datum_data->points[pointIdxNKNSearch[0]].x, datum_data->points[pointIdxNKNSearch[0]].y, datum_data->points[pointIdxNKNSearch[0]].z);
		Eigen::Vector3d xi(datum_model->points[i].x, datum_model->points[i].y, datum_model->points[i].z);
		Eigen::Vector3d w(normal.normal_x, normal.normal_y, normal.normal_z);
		Eigen::Vector3d v = yi - xi;
		double e = v.dot(w);
		datum_error = datum_error + e*e;
	}
	return datum_error;
};

double
OptimalRegistration::computeDatumAngle(pcl::ModelCoefficients::Ptr coeff_model, pcl::ModelCoefficients::Ptr coeff_data)
{
	return 0.0;
};

double
OptimalRegistration::computeSurfaceVariance(std::vector<double> &dist)
{
	double var = 0.0;
	double sum = 0.0;
	for (int i = 0; i < dist.size(); i++)
	{
		sum = sum + dist.at(i);
	}

	for (int j = 0; j < dist.size(); j++)
	{
		double d = dist.at(j) - sum / dist.size();
		var = var + d*d;
	}
	var = var / dist.size();
	return var;
};