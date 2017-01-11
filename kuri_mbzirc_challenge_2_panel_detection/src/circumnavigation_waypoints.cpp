#include "ros/ros.h"
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
#include <kuri_mbzirc_challenge_2_msgs/PanelPositionAction.h>

#include <unistd.h>

const double DEG2RAD = M_PI/180.0;
const double RAD2DEG = 1/DEG2RAD;


typedef kuri_mbzirc_challenge_2_msgs::PanelPositionAction PanelPositionAction;
typedef kuri_mbzirc_challenge_2_msgs::PanelPositionFeedback PanelPositionFeedback;
typedef kuri_mbzirc_challenge_2_msgs::PanelPositionResult PanelPositionResult;
typedef kuri_mbzirc_challenge_2_msgs::PanelPositionGoalConstPtr PanelPositionGoalConstPtr;


class PanelPositionActionHandler
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<PanelPositionAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  PanelPositionFeedback feedback_;
  PanelPositionResult result_;

public:
  bool is_node_enabled;

  PanelPositionActionHandler(std::string name) :
    as_(nh_, name, boost::bind(&PanelPositionActionHandler::executeCB, this, _1), false),
    action_name_(name)
  {
    is_node_enabled = false;
    as_.start();
  }

  ~PanelPositionActionHandler(void){}

  void executeCB(const PanelPositionGoalConstPtr &goal)
  {
    is_node_enabled = true;

    ros::Rate r(30);
    while (1)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        // set the action state to preempted
        as_.setPreempted();
        break;
      }

      if (!is_node_enabled)
      {
        // set the action state to succeeded
        as_.setSucceeded(result_);
        break;
      }

      ros::spinOnce();
      r.sleep();
    }
  }

  void setSuccess(geometry_msgs::PoseArray waypoints)
  {
    is_node_enabled = false;

    result_.success = true;
    result_.waypoints = waypoints;
  }

  void setFailure()
  {
    is_node_enabled = false;

    result_.success = false;
  }


};


typedef velodyne_pointcloud::PointXYZIR VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

struct HoughLine
{
    double r;
    double angle;
    int votes;

    bool operator<(const HoughLine & other) const
    {
        return (votes > other.votes);
    }
};



// ======
// Prototypes
// ======
void computeBoundingBox(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& pc_vector,std::vector<Eigen::Vector3f>& dimension_list, std::vector<Eigen::Vector4f>& centroid_list, std::vector<std::vector<pcl::PointXYZ> >& corners);
void getCloudClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& pc_vector);

void drawHoughLine(std::vector<HoughLine> lines, std::string frame_id);
void drawPoints(std::vector<geometry_msgs::Point> points, std::string frame_id);
std::vector<geometry_msgs::Point> findCorners(std::vector<HoughLine> lines);
std::vector<double> generateRange(double start, double end, double step);
std::vector<HoughLine> houghTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hough);

void callbackScan(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
void callbackOdom(const nav_msgs::Odometry::ConstPtr& odom_msg);


// =====
// Variables
// =====
ros::Subscriber sub_odom;
ros::Subscriber sub_scan;
ros::Publisher  pub_wall;
ros::Publisher  pub_lines;
ros::Publisher  pub_points;
ros::Publisher  pub_poses;
tf::TransformListener *tf_listener;

PanelPositionActionHandler *action_handler;
bool bypass_action_handler = false;
std::string actionlib_topic = "get_panel_cluster";
nav_msgs::Odometry current_odom;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "circumnavigation_waypoints");
  ros::NodeHandle node;

  // Topic handlers
  sub_scan  = node.subscribe("/scan", 1, callbackScan);
  sub_odom  = node.subscribe("/odometry/filtered", 1, callbackOdom);

  action_handler = new PanelPositionActionHandler(actionlib_topic);

  pub_wall  = node.advertise<sensor_msgs::PointCloud2>("/explore/PCL", 10);
  pub_lines = node.advertise<visualization_msgs::Marker>("/explore/HoughLines", 10);
  pub_points= node.advertise<visualization_msgs::Marker>("/explore/points", 10);
  pub_poses = node.advertise<geometry_msgs::PoseArray>("/explore/poses", 10);

  tf_listener = new tf::TransformListener();

  // Parse input
  if (argc > 1)
  {
    // Enable node without waiting for actionlib calls
    bypass_action_handler = true;
    std::cout << "Bypassing actionlib\n";
  }
  else
  {
    std::cout << "Waiting for messages on the \"" << actionlib_topic << "\" actionlib topic\n";
  }

  ros::spin();
  return 0;
}

geometry_msgs::Quaternion getQuaternionFromYaw(double yaw)
{
  geometry_msgs::Quaternion quat;

  tf::Quaternion tf_q;
  tf_q = tf::createQuaternionFromYaw(yaw);

  quat.x = tf_q.getX();
  quat.y = tf_q.getY();
  quat.z = tf_q.getZ();
  quat.w = tf_q.getW();

  return quat;
}

Eigen::Matrix4d convertStampedTransform2Matrix4d(tf::StampedTransform t)
{
  // Get translation
  Eigen::Vector3d T1(
      t.getOrigin().x(),
      t.getOrigin().y(),
      t.getOrigin().z()
  );

  // Get rotation matrix
  tf::Quaternion qt = t.getRotation();
  tf::Matrix3x3 R1(qt);

  Eigen::Matrix3d R;
  tf::matrixTFToEigen(R1,R);

  // Set
  Eigen::Matrix4d tf_eigen;
  tf_eigen.setZero ();
  tf_eigen.block (0, 0, 3, 3) = R;
  tf_eigen.block (0, 3, 3, 1) = T1;
  tf_eigen (3, 3) = 1;

  return tf_eigen;
}

geometry_msgs::PoseArray computeSimpleWaypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double size)
{
  // Wait until we have odometry data
  if (! &current_odom) //Check if pointer is null
  {
    ros::Rate r(30);
    ros::spinOnce();
    r.sleep();
  }

  geometry_msgs::PoseArray pose_list;
  pcl::PointXYZ minPoint, maxPoint;
  geometry_msgs::Point center;

  double box_size = size/2;

  // Find center of bounds (rough)
  pcl::getMinMax3D(*cloud, minPoint, maxPoint);
  center.x = (minPoint.x + maxPoint.x)/2;
  center.y = (minPoint.y + maxPoint.y)/2;


  // Make sure we generate waypoints in clockwise order in 3x3 grid
  const int pair_count = 8*2;
  static const int ij_pairs[] = {-1,-1,
                                -1, 0,
                                -1, 1,
                                 0, 1,
                                 1, 1,
                                 1, 0,
                                 1,-1,
                                 0,-1};

  // Compute waypoints
  std::vector<geometry_msgs::Pose> temp_poses;
  for (int m=0; m<pair_count; m+=2)
  {
    int i = ij_pairs[m];
    int j = ij_pairs[m+1];

    geometry_msgs::Pose p;
    p.position.x = center.x + box_size*i;
    p.position.y = center.y + box_size*j;

    double yaw = atan2(-j,-i);
    p.orientation = getQuaternionFromYaw(yaw);

    temp_poses.push_back(p);
  }

  // Find closest waypoint
  double min_dist = 1/.0;
  int min_dist_idx = 0;
  for (int i=0; i<temp_poses.size(); i++)
  {
    double x = temp_poses[i].position.x - current_odom.pose.pose.position.x;
    double y = temp_poses[i].position.y - current_odom.pose.pose.position.y;

    double d = x*x + y*y;

    if (d < min_dist)
    {
      min_dist = d;
      min_dist_idx = i;
    }
  }

  // Order waypoints
  for (int i=min_dist_idx; i<temp_poses.size(); i++)
    pose_list.poses.push_back( temp_poses[i] );

  for (int i=0; i<min_dist_idx; i++)
    pose_list.poses.push_back( temp_poses[i] );

  return pose_list;
}

void callbackOdom(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  if (!action_handler->is_node_enabled && !bypass_action_handler)
    return;

  current_odom = *odom_msg;
}

void callbackScan(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  if (!action_handler->is_node_enabled && !bypass_action_handler)
    return;

  // Convert scan message to cloud message
  sensor_msgs::PointCloud2 cloud_msg;
  laser_geometry::LaserProjection projector_;
  projector_.projectLaser(*scan_msg, cloud_msg);
  //projector_.transformLaserScanToPointCloud("base_link", *scan_msg, cloud, *tf_listener);

  // Convert msg to pointcloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointcloud;

  pcl::fromROSMsg (cloud_msg, cloud);
  input_pointcloud = cloud.makeShared();


  // Remove points that are too close or too far
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  double laser_min_range = 0.6; // Used to ignore the bumpers
  double laser_max_range = 5; // Used to ignore far points

  for (int i=0; i<input_pointcloud->points.size(); i++)
  {
    pcl::PointXYZ p = input_pointcloud->points[i];
    double r = sqrt(p.x*p.x + p.y*p.y);
    if (r > laser_max_range || r < laser_min_range)
      continue;

    cloud_filtered->points.push_back (p);
  }

  if (cloud_filtered->points.size() == 0)
  {
    std::cout << "No laser points detected nearby.\n";
    action_handler->setFailure();
    return;
  }


  // Get clusters
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pc_vector;
  getCloudClusters(cloud_filtered ,pc_vector);


  // Get size of each cluster
  std::vector<Eigen::Vector3f> dimension_list;
  std::vector<Eigen::Vector4f> centroid_list;
  std::vector<std::vector<pcl::PointXYZ> > corners_list;
  computeBoundingBox(pc_vector, dimension_list, centroid_list, corners_list);

  // Only keep the clusters that are likely to be panels
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pc_vector_clustered;
  for (int i = 0; i< dimension_list.size(); i++)
  {
    if (dimension_list[i][2] <= 1.5 && dimension_list[i][1] <= 1.5)
      pc_vector_clustered.push_back(pc_vector[i]);
  }


  if (pc_vector_clustered.size() == 0)
  {
    std::cout << "Could not find panel cluster.\n";
    action_handler->setFailure();
    return;
  }

  if (pc_vector_clustered.size() > 1)
  {
    std::cout << "Found multiple panel clusters. Using the first one.\n";
  }

  // Publish cluster clouds
  for (int i=0; i<pc_vector_clustered.size(); i++)
  {
    sensor_msgs::PointCloud2 cloud_cluster_msg;
    pcl::toROSMsg(*pc_vector_clustered[i], cloud_cluster_msg);
    cloud_cluster_msg.header.frame_id = scan_msg->header.frame_id;
    cloud_cluster_msg.header.stamp = ros::Time::now();
    pub_wall.publish(cloud_cluster_msg);

    if (pc_vector_clustered.size() > 1)
      usleep(100*1000);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud = pc_vector_clustered[0];




  /*
  // Hough transform
  std::vector<HoughLine> lines = houghTransform(cluster_cloud);

  // Draw Hough lines
  drawHoughLine(lines, scan_msg->header.frame_id);

  // Get intersecting points
  std::vector<geometry_msgs::Point> corners;
  corners = findCorners(lines);

  // Draw points
  drawPoints(corners, scan_msg->header.frame_id);
  */


  // Transform cluster to world frame
  pcl::PointCloud<pcl::PointXYZ>::Ptr tf_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  try
  {
    tf::StampedTransform transform;
    Eigen::Matrix4d tf_eigen;

    tf_listener->lookupTransform("/odom", scan_msg->header.frame_id, ros::Time(0), transform);
    tf_eigen = convertStampedTransform2Matrix4d(transform);


    pcl::transformPointCloud(*cluster_cloud, *tf_cloud, tf_eigen);

    // Compute waypoints around object
    geometry_msgs::PoseArray waypoints;
    waypoints = computeSimpleWaypoints(tf_cloud, 5);

    // Publish waypoints
    waypoints.header.frame_id = "/odom";
    pub_poses.publish(waypoints);

    action_handler->setSuccess(waypoints);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    return;
  }

}

void computeBoundingBox(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& pc_vector,std::vector<Eigen::Vector3f>& dimension_list, std::vector<Eigen::Vector4f>& centroid_list, std::vector<std::vector<pcl::PointXYZ> >& corners)
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

    // save the centroid into the centroid vector
    centroid_list.push_back(pcaCentroid);
    // save dimenstion into dimension list
    one_dimension[0] = maxPoint.x - minPoint.x;
    one_dimension[1] = maxPoint.y - minPoint.y;
    one_dimension[2] = maxPoint.z - minPoint.z;
    dimension_list.push_back(one_dimension);


    // Transform back
    Eigen::Matrix4f bboxTransform(Eigen::Matrix4f::Identity());
    bboxTransform.block<3,3>(0,0) = eigenVectorsPCA;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_corners_pca (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_corners_base (new pcl::PointCloud<pcl::PointXYZ>);

    cloud_corners_pca->points.push_back(minPoint);
    cloud_corners_pca->points.push_back(maxPoint);

    for (int i=0; i<cloud_corners_pca->points.size(); i++)
    {
      cloud_corners_pca->points[i].x -= projectionTransform(0,3);
      cloud_corners_pca->points[i].y -= projectionTransform(1,3);
      cloud_corners_pca->points[i].z -= projectionTransform(2,3);
    }

    pcl::transformPointCloud(*cloud_corners_pca, *cloud_corners_base, bboxTransform);

    // Extract corners
    std::vector<pcl::PointXYZ> c;
    for (int i=0; i<cloud_corners_base->points.size(); i++)
      c.push_back(cloud_corners_base->points[i]);

    // Save list of corners
    corners.push_back(c);
  }
}

void drawHoughLine(std::vector<HoughLine> lines, std::string frame_id)
{
  visualization_msgs::Marker marker_msg;
  marker_msg.header.frame_id = frame_id;
  marker_msg.header.stamp = ros::Time::now();
  marker_msg.type = marker_msg.LINE_LIST;
  marker_msg.pose.orientation.w = 1;

  marker_msg.scale.x = 0.01;
  marker_msg.color.a = 1.0;
  marker_msg.color.b = 1.0;


  double arc = 75*DEG2RAD;
  for (int i=0; i<lines.size(); i++)
  {
    double r = lines[i].r;
    double t = lines[i].angle;
    double r_new = r/cos(arc);

    double x1, x2, y1, y2;
    x1 = r_new*cos(t+arc);
    y1 = r_new*sin(t+arc);
    x2 = r_new*cos(t-arc);
    y2 = r_new*sin(t-arc);

    geometry_msgs::Point p1;
    p1.x = x1;
    p1.y = y1;
    marker_msg.points.push_back(p1);

    geometry_msgs::Point p2;
    p2.x = x2;
    p2.y = y2;
    marker_msg.points.push_back(p2);
  }

  pub_lines.publish(marker_msg);
}

void drawPoints(std::vector<geometry_msgs::Point> points, std::string frame_id)
{
  // Publish
  visualization_msgs::Marker marker_msg;
  marker_msg.header.frame_id = frame_id;
  marker_msg.header.stamp = ros::Time::now();
  marker_msg.type = marker_msg.POINTS;

  marker_msg.scale.x = 0.05;
  marker_msg.scale.y = 0.05;
  marker_msg.color.a = 1.0;
  marker_msg.color.g = 1.0;

  marker_msg.points = points;

  pub_points.publish(marker_msg);
}

std::vector<geometry_msgs::Point> findCorners(std::vector<HoughLine> lines)
{
  std::vector<HoughLine> line_pairs;

  // Get pairs
  for (int i=0; i<lines.size()-1; i++)
  {
    double min_angle = 80*DEG2RAD;
    double max_angle = 100*DEG2RAD;

    for (int j=i+1; j<lines.size(); j++)
    {
      double t1 = lines[i].angle;
      double t2 = lines[j].angle;

      double angle = t1-t2;
      if (angle > M_PI)
        angle -= M_PI;
      if (angle < -M_PI)
        angle += M_PI;
      angle = fabs(angle);

      if (min_angle <= angle && angle <= max_angle)
      {
        line_pairs.push_back(lines[i]);
        line_pairs.push_back(lines[j]);
      }
    }
  } //get pairs

  // Find corners
  std::vector<geometry_msgs::Point> corners;
  for (int i=0; i<line_pairs.size(); i+=2)
  {
    double r1, r2, t1, t2, m1, m2, c1, c2, x, y;
    r1 = line_pairs[i].r;
    t1 = line_pairs[i].angle;
    r2 = line_pairs[i+1].r;
    t2 = line_pairs[i+1].angle;

    m1 = -1/tan(t1);
    c1 = r1 / sin(t1);
    m2 = -1/tan(t2);
    c2 = r2 / sin(t2);

    x = (c2-c1)/(m1-m2);
    y = m1*x + c1;

    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    corners.push_back(p);
  }

  return corners;
}

std::vector<double> generateRange(double start, double end, double step)
{
  std::vector<double> vec;

  vec.push_back(start);

  while(1)
  {
    start += step;
    vec.push_back(start);

    if (start > end)
      break;
  }

  return vec;
}

void getCloudClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& pc_vector)
{
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_ptr);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (1.0); // 100cm - big since we're sure the panel is far from other obstacles (ie. barriers)
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (2500);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_ptr);
  ec.extract (cluster_indices);

  // Get the cloud representing each cluster
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_ptr->points[*pit]);

    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    pc_vector.push_back(cloud_cluster);
  }
}

std::vector<HoughLine> houghTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hough)
{
  double hough_range_res = 0.1;
  double hough_angle_res = 5*DEG2RAD;
  int hough_threshold = 15;

  // Get max r to set up rhos
  double r_max = 0;
  for (int i = 0; i< cloud_hough->points.size(); i++)
  {
    pcl::PointXYZ p = cloud_hough->points[i];
    double r = p.x*p.x + p.y*p.y;

    if (r > r_max)
      r_max = r;
  }
  r_max = sqrt(r_max);

  // Generate rho, theta and vote arrays
  std::vector<double> rhos, thetas;
  rhos = generateRange(0, r_max, hough_range_res);
  thetas = generateRange(-M_PI, M_PI, hough_angle_res);

  int sizeR = rhos.size();
  int sizeT = thetas.size();
  int *votes = (int*)calloc(sizeR * sizeT, sizeof(int));


  // Analyze each point (transform)
  for (int i = 0; i< cloud_hough->points.size(); i++)
  {
    pcl::PointXYZ p = cloud_hough->points[i];

    for (int i_t = 0; i_t < sizeT; i_t++)
    {
      double r = p.x*cos(thetas[i_t]) + p.y*sin(thetas[i_t]);
      if (r < 0)
        continue;

      r = hough_range_res*round(r/hough_range_res); // round to nearest value of rho
      int i_r = r / hough_range_res; // find corresponding index of rho

      votes[i_r*sizeT + i_t]++;
    }
  }

  /*
  // Find max votes
  int v_max = 0;
  for (int i_r=0; i_r < sizeR; i_r++)
  {
    for (int i_t=0; i_t<sizeT; i_t++)
    {
      if (votes[i_r*sizeT + i_t] > v_max)
        v_max = votes[i_r*sizeT + i_t];
    }
  }
  */

  std::vector<HoughLine> lines_out;
  for (int i_r=0; i_r < sizeR; i_r++)
  {
    for (int i_t=0; i_t<sizeT; i_t++)
    {
      if (votes[i_r*sizeT + i_t] >= hough_threshold)
      {
        bool isMaxima = true;

        for (int k_r=i_r-2; k_r <= i_r+2; k_r++)
        {
          if (!isMaxima)
            break;

          if (k_r < 0 || k_r >= sizeR)
            continue;

          for (int k_t=i_t-2; k_t<=i_t+2; k_t++)
          {
            if (i_t == k_t && i_r == k_r)
              continue;

            if (k_t < 0 || k_t > sizeT)
              continue;

            if (votes[i_r*sizeT + i_t] < votes[k_r*sizeT + k_t])
            {
              isMaxima = false;
              break;
            }
          } //for k_t
        } //for k_r

        if (isMaxima)
        {
            HoughLine h;
            h.r = rhos[i_r];
            h.angle = thetas[i_t];
            h.votes = votes[i_r*sizeT + i_t];
            lines_out.push_back(h);
        }


      } //if votes > hough_threshold
    } // for i_t
  } //for i_r


  // Sort in descending order of votes
  sort(lines_out.begin(), lines_out.end());

  // Clean up
  free(votes);

  return lines_out;
}
