#include "optimalReg.h"


using namespace pcl;
using namespace std;
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;


OptimalRegistration::OptimalRegistration()
{
	model.reset(new PointCloudT);
	data.reset(new PointCloudT);

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
	//distance.resize(model->size());
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
			filtered_model->points.push_back(cloud_model->points[j]);
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
			filtered_data->points.push_back(cloud_model->points[k]);
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
OptimalRegistration::estimateEveloped(const double probability)
{
	if ((model->points.size() == 0) || (data->points.size() == 0))
	{
		qDebug("Invalid cloud input!");
		return false;
	}
	
	std::vector<double> datum_plane = { 0.0, 0.0, 1.0, 0.0 };
	std::vector< std::vector<double> > slicing_planes;
	int layers = 10;
	createSlicingPlanes(datum_plane, model, slicing_planes, layers);
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
		estimatePointsBySlicing(slicing_planes[i], model, data, 1.0, points_on_planes_m, points_on_planes_d);
		computeConvexHull(points_on_planes_m, chull_points_inside, polygons_inside);
		computeConvexHull(points_on_planes_d, chull_points_outside, polygons_outside);

		for (int j = 0; j < chull_points_inside->points.size(); j++)
		{
			bool _enveloped_point = estimateInnerPoint(chull_points_inside->points[j], chull_points_outside);
			if (_enveloped_point)
				_enveloped_count++;
		}
		total_count += chull_points_inside->points.size();
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
	model.reset(new PointCloudT);
	model = cloud;
	return true;
}

bool
OptimalRegistration::setDataCloud(PointCloudT::Ptr cloud)
{
	data.reset(new PointCloudT);
	data = cloud;
	return true;
}

double 
OptimalRegistration::getEnvelopedRate()
{
	return enveloped_rate;
}