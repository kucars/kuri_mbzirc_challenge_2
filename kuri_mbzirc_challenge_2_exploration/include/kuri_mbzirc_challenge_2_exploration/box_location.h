#ifndef KURI_MBZIRC_CHALLENGE_2_EXPLORATION_BOX_LOCATION_H_
#define KURI_MBZIRC_CHALLENGE_2_EXPLORATION_BOX_LOCATION_H_

#include <ros/ros.h>
#include <iostream>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include "velodyne_pointcloud/point_types.h"

#include <laser_geometry/laser_geometry.h>

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

#include <actionlib/server/simple_action_server.h>
#include <kuri_mbzirc_challenge_2_msgs/BoxPositionAction.h>
#include <kuri_mbzirc_challenge_2_tools/pose_conversion.h>

#include <unistd.h>

typedef kuri_mbzirc_challenge_2_msgs::BoxPositionAction       ServerAction;
typedef kuri_mbzirc_challenge_2_msgs::BoxPositionFeedback     ServerFeedback;
typedef kuri_mbzirc_challenge_2_msgs::BoxPositionResult       ServerResult;
typedef kuri_mbzirc_challenge_2_msgs::BoxPositionGoalConstPtr ServerGoalConstPtr;

const double DEG2RAD = M_PI/180.0;
const double RAD2DEG = 1/DEG2RAD;



// ======
// Prototypes
// ======

class BoxLocator
{
protected:
  // =====
  // Variables
  // =====
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<ServerAction>* as_;
  bool is_done_;

  ros::Subscriber sub_odom_;
  ros::Subscriber sub_velo_;
  ros::Publisher  pub_wall_;
  ros::Publisher  pub_lines_;
  ros::Publisher  pub_points_;
  ros::Publisher  pub_poses_;
  tf::TransformListener *tf_listener_;

  geometry_msgs::PoseArray waypoints_;

  bool bypass_action_handler_;
  nav_msgs::Odometry current_odom;


  // =====
  // Methods
  // =====
  void callbackOdom(const nav_msgs::Odometry::ConstPtr& odom_msg);
  void callbackVelo(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

  void callbacksEnable();
  void callbacksDisable();



  void computeBoundingBox(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& pc_vector,std::vector<Eigen::Vector3f>& dimension_list, std::vector<Eigen::Vector4f>& centroid_list, std::vector<std::vector<pcl::PointXYZ> >& corners);
  geometry_msgs::PoseArray computeWaypoint(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double distance);
  void getCloudClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& pc_vector);

  void drawPoints(std::vector<geometry_msgs::Point> points, std::string frame_id);
  std::vector<double> generateRange(double start, double end, double step);


public:
  BoxLocator(actionlib::SimpleActionServer<ServerAction>* actionserver, bool bypass);
  bool run();
  bool run(ServerResult* as_result);
};


#endif
