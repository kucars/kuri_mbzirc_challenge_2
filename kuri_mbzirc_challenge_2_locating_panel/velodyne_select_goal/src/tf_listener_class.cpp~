#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
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

typedef pcl::PointXYZ XYZPoint;

class Listener
{
public:
  pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud (new pcl::PointCloud<pcl::PointXYZ>) goal_cloud;
  void center_cloud_cb(const sensor_msgs::PointCloud::ConstPtr& inMsg);
};
// %EndTag(CLASS_WITH_DECLARATION)%

void Listener::callback(const std_msgs::String::ConstPtr& msg)
{
     if (inMsg->points.size()>0)
     {
    ros::Time now = ros::Time(0);
    sensor_msgs::PointCloud cloud_in;
    cloud_in.header.stamp = inMsg->header.stamp;
    cloud_in.header.frame_id = inMsg->header.frame_id;
    cloud_in.points=inMsg->points;
 



         //center_cloud.points.resize(sifted_centroid_list.size()); 
         //one_centroid=sifted_centroid_list[0];
         std::cout<<"centroids are:"<<std::endl;
         std::cout<<inMsg->points[0].x<<std::endl;   
         tf::StampedTransform transform; 
         tf::TransformListener tf_listener; 

         
         //ros::Time now = ros::Time::now();


         ros::Duration(2).sleep();


         tf_listener.waitForTransform("/base_link","/velodyne",now,ros::Duration(0.0));
         tf_listener.lookupTransform("/base_link","/velodyne",now,transform);
         sensor_msgs::PointCloud gobal;
         
         //cloud_in=*inMsg;
         try {
        //listener.transformPointCloud("/base_link",cloud_in, gobal);

        tf_listener.transformPointCloud("/base_link",now,cloud_in, "/velodyne",gobal);

        std::cout<<"New centroids are:"<<std::endl;
        std::cout<<gobal.points[0].x<<std::endl;
                 } catch (tf2::ExtrapolationException &ex){
        }


        //update class member (accumulate goal point candidates)
        //cloud->points.size ()=cloud->points.size ()+gobal.points.size();
         p_cloud->height   = 1;
         p_cloud->is_dense = true;
         p_cloud->width=p_cloud->points.size();


          for (size_t i = 0; i < gobal.points.size(); ++i)
       {
       //create single point
        XYZPoint p;
        p.x = gobal.points[i].x;
        p.y = gobal.points[i].y;
        p.z = gobal.points[i].z;
       //assign pointer value
        cloud->points.push_back(p);
  
        // assign pointcloud value
        p_cloud->points.push_back(p);
        ++p_cloud->width;        
        }
        std::cout<<"Goal_point_cloud size is :"<<std::endl;
        std::cout<<p_cloud->width<<std::endl;
        //clustering with pcl




        // check the point number of each cluster to find the most stable goal point candidate) 
           
     }





}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener_class");
  ros::NodeHandle n;




// %Tag(SUBSCRIBER)%
  Listener listener;
  ros::Subscriber sub = n.subscribe("center_cloud",10, &Listener::center_cloud_cb, &listener);
// %EndTag(SUBSCRIBER)%

  ros::spin();

  return 0;
}
