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
using namespace ros;
typedef pcl::PointXYZ XYZPoint;

class Listener
{
public:
 
 // pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud;
  Listener(ros::NodeHandle node);
  Publisher output;

  //p_cloud.width=0; 
  //int time_step_counter=0;
  int time_step_counter;
  void center_cloud_cb(const sensor_msgs::PointCloud::ConstPtr& inMsg);
};
// %EndTag(CLASS_WITH_DECLARATION)%




Listener::Listener(ros::NodeHandle node){

    time_step_counter=0;
    output=node.advertise<sensor_msgs::PointCloud>("goal_cloud", 10);

    pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud_dummy (new pcl::PointCloud<pcl::PointXYZ>);

    p_cloud= p_cloud_dummy;



}



void Listener::center_cloud_cb(const sensor_msgs::PointCloud::ConstPtr& inMsg)
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

         ros::Duration(1).sleep();


         //tf_listener.waitForTransform("/base_link","/velodyne",now,ros::Duration(0.0));
         //tf_listener.lookupTransform("/base_link","/velodyne",now,transform);
         sensor_msgs::PointCloud gobal;
         
         //cloud_in=*inMsg;
         try {
        //listener.transformPointCloud("/base_link",cloud_in, gobal);




        //the first string is target coordinate 
        tf_listener.transformPointCloud("/base_link",now,cloud_in, "/velodyne",gobal);

        std::cout<<"New centroids are:"<<std::endl;
        std::cout<<gobal.points[0].x<<std::endl;
                 } 
        catch (tf2::ExtrapolationException &ex){
        }


        //update class member (accumulate goal point candidates)
        //cloud->points.size ()=cloud->points.size ()+gobal.points.size();






std::cout<<"here1:"<<std::endl;
         p_cloud->height   = 1;
std::cout<<"here1.5:"<<std::endl;
         p_cloud->is_dense = true;
        // p_cloud->width=p_cloud->points.size();
        
     std::cout<<"here2:"<<std::endl;

          for (size_t i = 0; i < gobal.points.size(); ++i)
       {
       //create single point
        XYZPoint p;
       std::cout<<"here in loop:"<<std::endl;
    

        p.x = gobal.points[i].x;
        p.y = gobal.points[i].y;
        p.z = gobal.points[i].z;
       //assign pointer value
       //cloud->points.push_back(p);
  
        // assign pointcloud value
        p_cloud->points.push_back(p);
           std::cout<<p_cloud->points.size()<<std::endl;
        ++p_cloud->width;        
        }
        //publish the accumulated pointcloud 
        std::cout<<"Goal_point_cloud size is :"<<std::endl;
        std::cout<<p_cloud->width<<std::endl;


      
        std::cout<<"here3:"<<std::endl;


       sensor_msgs::PointCloud goal_cloud;
       //center_cloud.header.stamp= inMsg->header.stamp;
       goal_cloud.header.stamp= ros::Time::now();
       goal_cloud.header.frame_id = inMsg->header.frame_id;







       if (p_cloud->points.size()>0)
       {
         goal_cloud.points.resize(p_cloud->points.size());

  
     
         for (size_t i = 0; i < p_cloud->points.size(); ++i)
         {
            goal_cloud.points[i].x=p_cloud->points[i].x;
            goal_cloud.points[i].y=p_cloud->points[i].y;
            goal_cloud.points[i].z=p_cloud->points[i].z;      
         }

       }
       output.publish(goal_cloud);































       // make a counter here if certain timae interval is exceeded and no cluser meet the threshold clear everything thing in the point cloud p_cloud
      
         ++time_step_counter;
         if(time_step_counter>=30)
           {
                p_cloud->points.clear(); 
                p_cloud->width=0;
                time_step_counter=0;
                  
           }
          



        // Clustering with pcl

        std::vector<pcl::PointIndices> cluster_indices;
        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (p_cloud);
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        // Clustering
        ec.setClusterTolerance (0.2); // 20cm
        ec.setMinClusterSize (5);
        ec.setMaxClusterSize (30);
        ec.setSearchMethod (tree);
        ec.setInputCloud (p_cloud);
        ec.extract (cluster_indices);
        std::cout<<"cluster no. :"<<cluster_indices.size()<<std::endl;
         







        // check the point number of each cluster to find the most stable goal point candidate) 
          for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
              {

                      if(it->indices.size()>15)
                      {
                              std::cout<<"new goal point get!"<<std::endl;       
                              // take mean 
                              //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
                               double point_num=0;
                               double x=0;
                               double y=0;
                               double z=0;
                               for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                               {   

                                   XYZPoint p;
                                    
                                   p=p_cloud->points[*pit]; //*
                                   x+=p.x;
                                   y+=p.y;
                                   z+=p.z;
                                   ++point_num;
                               }      
                               x=x/point_num;
                               y=y/point_num;
                               z=z/point_num; 
                               // set this point as the goal point
                               std::vector<double> goal_vector;
                               goal_vector.push_back(x);
                               goal_vector.push_back(y);
                               goal_vector.push_back(z);
                               ros::param::set("/panel_goal", goal_vector);
                               //std::cout<<"It is"<<"," << x<<"," <<y<<","<<z<std::endl;  
                               std::cout<<"It is"<< x<<"," <<y<<","<<z<<std::endl; 


                                p_cloud->points.clear(); 
              			p_cloud->width=0;
              			time_step_counter=0;
                                ros::shutdown();
                                

                      }
 
         
                        
              } 

     }
  

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener_class");
  ros::NodeHandle n;




// %Tag(SUBSCRIBER)%
  Listener listener(n);

  ros::Subscriber sub = n.subscribe("center_cloud",10, &Listener::center_cloud_cb, &listener);
// %EndTag(SUBSCRIBER)%

  ros::spin();

  return 0;
}
