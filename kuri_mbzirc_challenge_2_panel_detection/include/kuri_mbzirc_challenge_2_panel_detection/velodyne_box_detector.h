#ifndef KURI_MBZIRC_CHALLENGE_2_EXPLORATION_BOX_LOCATION_H_
#define KURI_MBZIRC_CHALLENGE_2_EXPLORATION_BOX_LOCATION_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/common/common.h>

#include <kuri_mbzirc_challenge_2_msgs/BoxPositionAction.h>


typedef kuri_mbzirc_challenge_2_msgs::BoxPositionAction Action;
typedef kuri_mbzirc_challenge_2_msgs::BoxPositionGoal Goal;
typedef kuri_mbzirc_challenge_2_msgs::BoxPositionFeedback Feedback;
typedef kuri_mbzirc_challenge_2_msgs::BoxPositionResult Result;
typedef kuri_mbzirc_challenge_2_msgs::BoxPositionGoalConstPtr GoalConstPtr;

typedef pcl::PointXYZ PcPoint;
typedef pcl::PointCloud<PcPoint> PcCloud;
typedef pcl::PointCloud<PcPoint>::Ptr PcCloudPtr;
typedef std::vector<PcCloudPtr> PcCloudPtrList;



struct BoxCluster{
  sensor_msgs::PointCloud2 point_cloud;
  geometry_msgs::Pose pose;
  double confidence;
};

// ======
// Classes
// ======
class BoxPositionActionHandler
{
protected:
  bool is_initiatializing_;

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<Action> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  Feedback feedback_;
  Result result_;

  double range_max_;
  double range_min_;
  double angle_max_;
  double angle_min_;
  double detect_new_distance_;

  std::vector<BoxCluster> cluster_list;
  PcCloudPtr pc_current_;
  PcCloudPtr pc_prev_;
public:
  ros::Subscriber sub_odom;
  ros::Subscriber sub_velo;
  ros::Publisher  pub_wall;
  ros::Publisher  pub_lines;
  ros::Publisher  pub_points;
  tf::TransformListener *tf_listener;
  nav_msgs::Odometry current_odom;

  BoxPositionActionHandler(std::string name);
  ~BoxPositionActionHandler(){}

  // actionlib
  void executeCB(const GoalConstPtr &goal);
  void setSuccess(bool success = true);

  void callbackVelo(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
  void callbackOdom(const nav_msgs::Odometry::ConstPtr& odom_msg);

  void computeBoundingBox(PcCloudPtrList& pc_vector,std::vector<Eigen::Vector3f>& dimension_list, std::vector<Eigen::Vector4f>& centroid_list, std::vector<std::vector<PcPoint> >& corners);
  void getCloudClusters(PcCloudPtr cloud_ptr, PcCloudPtrList& pc_vector);
  void getInitialBoxClusters();
  PcCloudPtrList extractBoxClusters(PcCloudPtr cloud_ptr);

  void drawPoints(std::vector<geometry_msgs::Point> points, std::string frame_id);

  std::vector<double> generateRange(double start, double end, double step);
  Eigen::Matrix4d convertStampedTransform2Matrix4d(tf::StampedTransform t);
  geometry_msgs::Quaternion getQuaternionFromYaw(double yaw);
};

#endif
