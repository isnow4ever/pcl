#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

// Types
typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::Normal> PointCloudN;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

// Align a rigid object to a scene with clutter and occlusions
int
main (int argc, char **argv)
{
  // Point clouds
  PointCloudT::Ptr object (new PointCloudT);
  PointCloudN::Ptr normals(new PointCloudN);
  FeatureCloudT::Ptr object_features (new FeatureCloudT);
  std::string filename;
  filename = "data/blade_data.ply";

  if (!pcl::io::loadPLYFile(filename, *object))
  {
	  printf("Cannot open ply file!");
  }

  //VoxelGrid Filter Downsampling
  pcl::VoxelGrid<PointT> grid;
  const float leaf = 1.00;
  grid.setLeafSize(leaf, leaf, leaf);
  grid.setInputCloud(object);
  grid.filter(*object);
  
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(object);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(1);
  ne.compute(*normals);

  printf("%d", normals->size());

  //pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  //pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZ, pcl::Normal, pcl::Label>mps;
  mps.setMinInliers(1000);
  mps.setAngularThreshold(0.017453*2.0);
  mps.setDistanceThreshold(0.02);
  mps.setInputNormals(normals);
  mps.setInputCloud(object);
  std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
  //std::vector<pcl::ModelCoefficients> model_coefficients;
  //std::vector<pcl::PointIndices> inlier_indices;
  //pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>);
  //std::vector<pcl::PointIndices> label_indices;
  //std::vector<pcl::PointIndices> boundary_indices;

  mps.segmentAndRefine(regions);
  printf("%d", regions.size());

  boost::shared_ptr<pcl::visualization::PCLVisualizer> mpsvis;
  char name[1024];
  unsigned char red[6] = { 255,   0,   0, 255, 255,   0 };
  unsigned char grn[6] = { 0, 255,   0, 255,   0, 255 };
  unsigned char blu[6] = { 0,   0, 255,   0, 255, 255 };

  pcl::PointCloud<PointT>::Ptr contour(new pcl::PointCloud<PointT>);

  for (size_t i = 0; i < regions.size(); i++)
  {
	  Eigen::Vector3f centroid = regions[i].getCentroid();
	  Eigen::Vector4f model = regions[i].getCoefficients();
	  pcl::PointXYZ pt1 = pcl::PointXYZ(centroid[0], centroid[1], centroid[2]);
	  pcl::PointXYZ pt2 = pcl::PointXYZ(centroid[0] + (0.5f * model[0]),
		  centroid[1] + (0.5f * model[1]),
		  centroid[2] + (0.5f * model[2]));
	  sprintf(name, "normal_%d", unsigned(i));
	  mpsvis->addArrow(pt2, pt1, 1.0, 0, 0, false, name);

	  contour->points = regions[i].getContour();
	  sprintf(name, "plane_%02d", int(i));
	  pcl::visualization::PointCloudColorHandlerCustom <PointT> color(contour, red[i % 6], grn[i % 6], blu[i % 6]);
	  if (!mpsvis->updatePointCloud(contour, color, name))
		  mpsvis->addPointCloud(contour, color, name);
	  mpsvis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
  }
  mpsvis->spinOnce();

  system("pause");
  return (0);
}
