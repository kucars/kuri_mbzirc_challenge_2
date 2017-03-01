#include <ros/ros.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <Eigen/Dense>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <kuri_mbzirc_challenge_2_exploration/gps_conversion.h>
#include <kuri_mbzirc_challenge_2_tools/pose_conversion.h>

struct GeoPoint
{
  double lat, lon;
};

std::vector<GeoPoint> arena_bounds_;
GPSHandler ref_gps;
geometry_msgs::Quaternion ref_gps_quaternion;
ros::Publisher pub_points;



typedef pcl::PointXYZ PcPoint;
typedef pcl::PointCloud<PcPoint> PcCloud;
typedef pcl::PointCloud<PcPoint>::Ptr PcCloudPtr;
typedef std::vector<PcCloudPtr> PcCloudPtrList;




bool pointWithinBounds (GeoPoint p)
{
  // Adapted from http://wiki.unity3d.com/index.php?title=PolyContainsPoint
  bool inside = false;

  int j = arena_bounds_.size()-1;

  for (int i = 0; i < arena_bounds_.size(); j = i++)
  {
    GeoPoint c1 = arena_bounds_[i];
    GeoPoint c2 = arena_bounds_[j];

    if ( ((c1.lat <= p.lat && p.lat < c2.lat) || (c2.lat <= p.lat && p.lat < c1.lat)) &&
       (p.lon < (c2.lon - c1.lon) * (p.lat - c1.lat) / (c2.lat - c1.lat) + c1.lon))
       inside = !inside;
  }

  return inside;
}


void callbackVelo(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  if (!ref_gps.is_init())
    return;

  // Convert msg to pointcloud
  PcCloud cloud;

  pcl::fromROSMsg (*cloud_msg, cloud);
  PcCloud::Ptr pc_current = cloud.makeShared();

  // Get transformation
  Eigen::Matrix4d Ti = pose_conversion::getTransformationMatrix(ref_gps_quaternion);

  // Orient pointcloud towards north
  PcCloud::Ptr pc_rotated (new PcCloud);
  pcl::transformPointCloud (*pc_current, *pc_rotated, Ti);

  // Check which points are within bounds
  PcCloud::Ptr final_cloud (new PcCloud);


  //printf("CURRENT lat: %lf\tlon: %lf\n", ref_gps.getLat(), ref_gps.getLon() );
  for (int i=0; i<pc_rotated->points.size(); i++)
  {
    PcPoint p = pc_rotated->points[i];
    GeoPoint g;

    /*
    double r = p.x*p.x + p.y*p.y;
    if (r > 1)
      continue;
      */

    // Convert from local to gps coordinates
    ref_gps.projectCartesianToGPS(p.x, -p.y, &g.lat, &g.lon);

    bool isWithinBounds = pointWithinBounds(g);
    //printf("lat: %lf\tlon: %lf\t%d\n", g.lat, g.lon, isWithinBounds);

    // Check if gps coordinates within predefined bounds
    if (isWithinBounds)
      final_cloud->points.push_back(pc_current->points[i]);
  }

  //Publish message
  sensor_msgs::PointCloud2 cloud_cluster_msg;
  pcl::toROSMsg(*final_cloud, cloud_cluster_msg);
  cloud_cluster_msg.header.frame_id = cloud_msg->header.frame_id;
  cloud_cluster_msg.header.stamp = ros::Time::now();
  pub_points.publish(cloud_cluster_msg);
}

void callbackGPS(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  ref_gps.update(msg->latitude, msg->longitude, msg->header.seq);
}

void callbackIMU(const sensor_msgs::Imu::ConstPtr& msg)
{
  ref_gps_quaternion = msg->orientation;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_gps_filter_velodyne");
  ros::NodeHandle node_handle("kuri_challenge2_exploration");

  // Load parameters
  for (int i=1; i<=4; i++)
  {
    GeoPoint c;
    char param_name[50]; // Holds name of parameter

    sprintf(param_name, "arena_gps_bounds/corner%d/lat", i);
    node_handle.param(param_name, c.lat, 999.0);

    sprintf(param_name, "arena_gps_bounds/corner%d/lon", i);
    node_handle.param(param_name, c.lon, 999.0);

    if (c.lat == 999.0| c.lon == 999.0)
    {
      printf("Error: Parameter arena_gps_bounds/corner%d not defined. Exiting.\n", i);
      return -1;
    }

    arena_bounds_.push_back(c);
  }

  // Topic handlers
  ros::Subscriber sub_gps   = node_handle.subscribe("/gps/fix", 1, callbackGPS);
  ros::Subscriber sub_imu   = node_handle.subscribe("/imu/data", 1, callbackIMU);
  ros::Subscriber sub_velo  = node_handle.subscribe("/velodyne_points", 1, callbackVelo);
  pub_points = node_handle.advertise<sensor_msgs::PointCloud2>("/explore/filtered_gps_points", 10);

  ros::spin();

  return 0;
}
