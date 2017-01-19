#ifndef KURI_MBZIRC_CHALLENGE_2_EXPLORATION_BOX_LOCATION_H_
#define KURI_MBZIRC_CHALLENGE_2_EXPLORATION_BOX_LOCATION_H_

#include <ros/ros.h>

#include <kuri_mbzirc_challenge_2_msgs/BoxPositionAction.h>


const double DEG2RAD = M_PI/180.0;
const double RAD2DEG = 1/DEG2RAD;


typedef kuri_mbzirc_challenge_2_msgs::BoxPositionAction BoxPositionAction;
typedef kuri_mbzirc_challenge_2_msgs::BoxPositionFeedback BoxPositionFeedback;
typedef kuri_mbzirc_challenge_2_msgs::BoxPositionResult BoxPositionResult;
typedef kuri_mbzirc_challenge_2_msgs::BoxPositionGoalConstPtr BoxPositionGoalConstPtr;


// ======
// Classes
// ======
class BoxPositionActionHandler
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<BoxPositionAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  BoxPositionFeedback feedback_;
  BoxPositionResult result_;

public:
  bool is_node_enabled;

  BoxPositionActionHandler(std::string name);
  ~BoxPositionActionHandler(){}

  // actionlib
  void executeCB(const BoxPositionGoalConstPtr &goal);
  void setSuccess(geometry_msgs::PoseArray waypoints);
  void setFailure();
};



// ======
// Prototypes
// ======
void computeBoundingBox(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& pc_vector,std::vector<Eigen::Vector3f>& dimension_list, std::vector<Eigen::Vector4f>& centroid_list, std::vector<std::vector<pcl::PointXYZ> >& corners);
void getCloudClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& pc_vector);

void drawPoints(std::vector<geometry_msgs::Point> points, std::string frame_id);
std::vector<double> generateRange(double start, double end, double step);

void callbackVelo(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
void callbackOdom(const nav_msgs::Odometry::ConstPtr& odom_msg);



// =====
// Variables
// =====
ros::Subscriber sub_odom;
ros::Subscriber sub_velo;
ros::Publisher  pub_wall;
ros::Publisher  pub_lines;
ros::Publisher  pub_points;
ros::Publisher  pub_poses;
tf::TransformListener *tf_listener;

BoxPositionActionHandler *action_handler;
bool bypass_action_handler = false;
std::string actionlib_topic = "get_box_cluster";
nav_msgs::Odometry current_odom;

#endif
