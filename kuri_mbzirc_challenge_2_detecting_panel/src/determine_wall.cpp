#include "ros/ros.h"
#include <iostream>
#include "sensor_msgs/PointCloud2.h"
#include <tf/transform_listener.h>
#include "velodyne_pointcloud/point_types.h"

#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <unistd.h>

ros::Subscriber sub_velo;
ros::Publisher  pub_wall;

typedef velodyne_pointcloud::PointXYZIR VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

// ======
// Prototypes
// ======
void callbackVelo(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
void computeBoundingBox(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& pc_vector,std::vector<Eigen::Vector3f>& dimension_list, std::vector<Eigen::Vector4f>& centroid_list);
void extractPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointcloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& pc_vector);
void prunePlanes(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& pc_vector, std::vector<Eigen::Vector3f>& dimension_list, std::vector<Eigen::Vector4f>& centroid_list,std::vector<Eigen::Vector4f>& sifted_centroid_list);
void getClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointcloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, std::vector<pcl::PointIndices>& cluster_indices);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "determine_wall");
  ros::NodeHandle node;

  // subscribe to VelodyneScan packets
  sub_velo = node.subscribe("/velodyne_points", 1, callbackVelo);
  pub_wall = node.advertise<sensor_msgs::PointCloud2>("panel_wall", 10);

  ros::spin();
  return 0;
}


void callbackVelo(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  // Convert to pcl pointcloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointcloud;

  pcl::fromROSMsg (*cloud_msg, cloud);
  input_pointcloud = cloud.makeShared();




  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (input_pointcloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.2); // 20cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

    // Publish message
    if (cloud_cluster->size() > 0)
    {
      sensor_msgs::PointCloud2 cloud_msg;
      pcl::toROSMsg(*cloud_cluster, cloud_msg);
      cloud_msg.header.frame_id = "velodyne";
      cloud_msg.header.stamp = ros::Time::now();
      pub_wall.publish(cloud_msg);

      sleep(1);
    }

  }



  /*
  // Get clusters based on Eucliad distance
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  getClusters(input_pointcloud, cloud_filtered, cluster_indices);

  std::cout << cluster_indices.size() << "\n";

  // Extract Planes
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pc_vector;
  extractPlanes(cloud_filtered, pc_vector);

  // Get size of each plane
  std::vector<Eigen::Vector3f> dimension_list;
  std::vector<Eigen::Vector4f> centroid_list;
  computeBoundingBox(pc_vector, dimension_list, centroid_list);


  pcl::PointCloud<pcl::PointXYZ> cloud_planes;
  for (int i=0; i<pc_vector.size(); i++)
  {
    cloud_planes += *pc_vector[i];
  }


  // Publish message
  if (pc_vector.size() > 0)
  {
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud_planes, cloud_msg);
    cloud_msg.header.frame_id = "velodyne";
    cloud_msg.header.stamp = ros::Time::now();
    pub_wall.publish(cloud_msg);
  }

  /*
  // Prune planes based on geometric and topological constraints
  std::vector<Eigen::Vector4f> sifted_centroid_list;
  prunePlanes(pc_vector, dimension_list, centroid_list, sifted_centroid_list);
  */
}

void getClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointcloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, std::vector<pcl::PointIndices>& cluster_indices)
{
  /*
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud (input_pointcloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  */

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

  // If the cluster tolerance is too small the algorithm will fail to find anything or divid one object into multiple laysers
  // Since the Velodyne has a lower resolution between layers
  ec.setClusterTolerance (0.2); // 20cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);
}

void extractPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointcloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& pc_vector)
{
  // == Extract planes
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  // vertical plane detection
  seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
  seg.setAxis(Eigen::Vector3f(0,0,1));
  seg.setEpsAngle (0.4);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);


  //change this to include objects on or attached to the plane
  seg.setDistanceThreshold (0.02);
  int nr_points = (int) input_pointcloud->points.size ();
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> local_pc_vector;

  while (input_pointcloud->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (input_pointcloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (input_pointcloud);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);

    // save the pc in the local pc vector
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp;
    cloud_temp = cloud_plane->makeShared();
    local_pc_vector.push_back(cloud_temp);

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *input_pointcloud = *cloud_f;
  }

  std::cout << "Found " << local_pc_vector.size() << " planes\n";
  pc_vector.insert(pc_vector.end(), local_pc_vector.begin(), local_pc_vector.end());
}


void computeBoundingBox(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& pc_vector,std::vector<Eigen::Vector3f>& dimension_list, std::vector<Eigen::Vector4f>& centroid_list)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  Eigen::Vector3f one_dimension;

  for (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>::const_iterator iterator = pc_vector.begin(), end = pc_vector.end(); iterator != end; ++iterator)
  {
    cloud_plane=*iterator;

    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloud_plane, pcaCentroid);

    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cloud_plane, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_plane, *cloudPointsProjected, projectionTransform);

    // Get the minimum and maximum points of the transformed cloud.
    pcl::PointXYZ minPoint, maxPoint;

    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/  watch?v=mHVwd8gYLnI
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
    // save the centroid into the centroid vector
    centroid_list.push_back(pcaCentroid);
    // save dimenstion into dimension list
    one_dimension[0] = maxPoint.x - minPoint.x;
    one_dimension[1] = maxPoint.y - minPoint.y;
    one_dimension[2] = maxPoint.z - minPoint.z;
    dimension_list.push_back(one_dimension);
  }
}




void prunePlanes(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& pc_vector, std::vector<Eigen::Vector3f>& dimension_list, std::vector<Eigen::Vector4f>& centroid_list,std::vector<Eigen::Vector4f>& sifted_centroid_list)
{
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pc_vector_temp;

  for (int i=0; i<pc_vector.size(); i++)
  {
    // Filtered with plane dimension
    Eigen::Vector3f one_dimension = dimension_list[i];
    Eigen::Vector4f one_centroid = centroid_list[i];

    std::cout<< "x: " << one_dimension[1] << "\ty: " << one_dimension[2] << std::endl;

    if( one_dimension[0,1]>0.5 &&
        one_dimension[0,1]<1.5 &&
        one_dimension[0,2]<1.5 &&
        one_dimension[0,2]/one_dimension[0,1]<2)
    {
      //append centroid list to the sifted_centroid_list
      sifted_centroid_list.push_back(one_centroid);
      pc_vector_temp.push_back(pc_vector[i]);
    }
  }

  pc_vector = pc_vector_temp;
}
