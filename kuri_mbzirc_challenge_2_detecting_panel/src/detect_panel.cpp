#include "ros/ros.h"
#include <iostream>

#include "geometry_msgs/Point.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"

#include <tf/transform_listener.h>
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

ros::Subscriber sub_velo;
ros::Subscriber sub_scan;
ros::Publisher  pub_wall;
ros::Publisher  pub_lines;
ros::Publisher  pub_points;
tf::TransformListener *tf_listener;

bool isNodeEnabled = false;

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

  PanelPositionActionHandler(std::string name) :
    as_(nh_, name, boost::bind(&PanelPositionActionHandler::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~PanelPositionActionHandler(void){}

  void executeCB(const PanelPositionGoalConstPtr &goal)
  {
    isNodeEnabled = true;

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

      if (!isNodeEnabled)
      {
        // set the action state to succeeded
        as_.setSucceeded(result_);
        break;
      }

      ros::spinOnce();
      r.sleep();
    }
  }

  void setSuccess()
  {
    isNodeEnabled = false;
    result_.success = true;
  }

  void setFailure()
  {
    isNodeEnabled = false;
    result_.success = false;
  }


};








PanelPositionActionHandler *action_handler;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "determine_wall");
  ros::NodeHandle node;

  // Topic handlers
  sub_velo  = node.subscribe("/scan", 1, callbackScan);

  action_handler = new PanelPositionActionHandler("get_panel_cluster");

  pub_wall  = node.advertise<sensor_msgs::PointCloud2>("/explore/PCL", 10);
  pub_lines = node.advertise<visualization_msgs::Marker>("/explore/HoughLines", 10);
  pub_points= node.advertise<visualization_msgs::Marker>("/explore/points", 10);

  tf_listener = new tf::TransformListener();
  tf_listener->setExtrapolationLimit(ros::Duration(0.1));

  ros::spin();
  return 0;
}

void callbackScan(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  if (!isNodeEnabled)
      return;

  // Conver scan message to cloud message
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

  std::vector<pcl::PointXYZ> corners;

  //corners = corners_list[0];
  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud = pc_vector_clustered[0];


  // Compute object centroid
  std::vector<geometry_msgs::Point> pt_out;

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



  if (corners.size() == 0)
  {
    // Create bounds based on edge alone
    // Assume we only have a single line from the panel. Get the center of this line
    pcl::PointXYZ minPoint, maxPoint;
    geometry_msgs::Point center, p1, p2, p3, p4;

    pcl::getMinMax3D(*cluster_cloud, minPoint, maxPoint);

    center.x = (minPoint.x + maxPoint.x)/2;
    center.y = (minPoint.y + maxPoint.y)/2;

    pt_out.push_back(center);


    double box_size = 5/2;
    p1.x = center.x + box_size;
    p1.y = center.y + box_size;

    p2.x = center.x + box_size;
    p2.y = center.y - box_size;

    p3.x = center.x - box_size;
    p3.y = center.y - box_size;

    p4.x = center.x - box_size;
    p4.y = center.y + box_size;

    pt_out.push_back(p1);
    pt_out.push_back(p2);
    pt_out.push_back(p3);
    pt_out.push_back(p4);
  }
  else
  {
    for (int i=0; i<corners.size(); i++)
    {
      geometry_msgs::Point p;
      p.x = corners[i].x;
      p.y = corners[i].y;
      p.z = corners[i].z;

      pt_out.push_back(p);
    }
  }

  // Draw points
  drawPoints(pt_out, scan_msg->header.frame_id);



  action_handler->setSuccess();



  /*
  // Get min and max points
  pcl::PointXYZ minPoint, maxPoint;
  pcl::getMinMax3D(*cluster_cloud, minPoint, maxPoint);
  */

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
