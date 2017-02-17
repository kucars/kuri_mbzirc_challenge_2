#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// Types

typedef pcl::PointXYZ PointPT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointPT> PointCloudPT;
typedef pcl::PointCloud<PointNT> PointCloudNT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerNT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointPT> ColorHandlerPT;



bool is_initialized_ = false;
bool is_done_ = false;
PointCloudPT::Ptr pc_current_, pc_prev_;

void callbackVelo(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

// Align a rigid object to a scene with clutter and occlusions
int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_velodyne_alignment");
  ros::NodeHandle node;

  ros::Subscriber sub_velo  = node.subscribe("/velodyne_points", 1, callbackVelo);

  ros::spin();
  return 0;
}

void callbackVelo(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  if (is_done_)
    return;


  // Convert msg to pointcloud
  PointCloudPT cloud;

  pcl::fromROSMsg (*cloud_msg, cloud);
  pc_current_ = cloud.makeShared();

  if (!is_initialized_)
  {
    pc_prev_ = pc_current_;
    is_initialized_ = true;
    return;
  }



  // Point clouds
  PointCloudPT::Ptr object = pc_current_;
  PointCloudPT::Ptr scene = pc_prev_;

  PointCloudNT::Ptr object_aligned (new PointCloudNT);
  PointCloudNT::Ptr object_normals (new PointCloudNT);
  PointCloudNT::Ptr scene_normals (new PointCloudNT);
  FeatureCloudT::Ptr object_features (new FeatureCloudT);
  FeatureCloudT::Ptr scene_features (new FeatureCloudT);

  printf("Cloud1 size: %lu, Cloud2 size: %lu\n\n", object->points.size(), scene->points.size());

  // Filter points above and below a certain point (ground and sky)
  pcl::console::print_highlight ("Filtering z...\n");
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-0.8, 4);
  //pass.setFilterLimitsNegative (true);
  pass.setInputCloud (object);
  pass.filter (*object);
  pass.setInputCloud (scene);
  pass.filter (*scene);



  // Downsample
  pcl::console::print_highlight ("Downsampling...\n");
  pcl::VoxelGrid<PointPT> grid;
  const float leaf = 0.1f;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (object);
  grid.filter (*object);
  grid.setInputCloud (scene);
  grid.filter (*scene);

  printf("Cloud1 size: %lu, Cloud2 size: %lu\n\n", object->points.size(), scene->points.size());



  // Estimate normals for scene and object
  pcl::console::print_highlight ("Estimating scene normals...\n");
  pcl::NormalEstimationOMP<PointPT,PointNT> nest;
  //nest.setKSearch (10);
  nest.setRadiusSearch (1.0);
  {
    pcl::ScopeTime t("Normal Estimation for scene");
    nest.setInputCloud (scene);
    nest.compute (*scene_normals);

  }
  {
    pcl::ScopeTime t("Normal Estimation for object");
    nest.setInputCloud (object);
    nest.compute (*object_normals);
  }

  printf("Cloud1 size: %lu, Cloud2 size: %lu\n\n", object_normals->points.size(), scene_normals->points.size());



  // Estimate features
  pcl::console::print_highlight ("Estimating features...\n");
  FeatureEstimationT fest;
  fest.setRadiusSearch (0.5);
  {
    pcl::ScopeTime t("Feature Estimation for scene");
    fest.setInputCloud (scene_normals);
    fest.setInputNormals (scene_normals);
    fest.compute (*scene_features);
  }
  {
    pcl::ScopeTime t("Feature Estimation for object");
    fest.setInputCloud (object_normals);
    fest.setInputNormals (object_normals);
    fest.compute (*object_features);
  }

  printf("Cloud1 size: %lu, Cloud2 size: %lu\n\n", object_features->points.size(), object_features->points.size());


  // Perform alignment
  pcl::console::print_highlight ("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
  align.setInputSource (object_normals);
  align.setSourceFeatures (object_features);
  align.setInputTarget (scene_normals);
  align.setTargetFeatures (scene_features);
  align.setMaximumIterations (10000); // Number of RANSAC iterations
  align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (10); // Number of nearest features to use
  align.setSimilarityThreshold (0.7f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
  align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
  {
    pcl::ScopeTime t("Alignment");
    align.align (*object_aligned);
  }

  if (align.hasConverged ())
  {
    // Print results
    printf ("\n");
    Eigen::Matrix4f transformation = align.getFinalTransformation ();
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());

    // Show alignment
    pcl::visualization::PCLVisualizer visu("Alignment");
    visu.addPointCloud (scene_normals, ColorHandlerNT (scene_normals, 0.0, 255.0, 0.0), "scene");
    visu.addPointCloud (object_aligned, ColorHandlerNT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
    visu.spin ();
  }
  else
  {
    pcl::console::print_error ("Alignment failed!\n");

    // Show clouds
    pcl::visualization::PCLVisualizer visu("Inputs");

    visu.addPointCloudNormals<PointPT, PointNT>(scene, scene_normals, 2, 0.2, "scene_w_normals");
    visu.addPointCloudNormals<PointPT, PointNT>(object, object_normals, 2, 0.2, "object_w_normals");

    visu.addPointCloud<PointPT> (scene, ColorHandlerPT(scene, 0.0, 255.0, 0.0),  "scene");
    visu.addPointCloud<PointPT> (object,ColorHandlerPT(object, 255.0, 0.0, 0.0), "object");

    visu.spin ();
  }

  // Update clouds
  pc_prev_ = pc_current_;
  is_done_ = true;

}
