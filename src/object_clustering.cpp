#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <signal.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>


ros::Publisher pub_debug_pcl;
ros::Publisher xyz_pub;
double cluster_tolerans = 0.01;
int cluster_size_min = 20;
int cluster_size_max = 500;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
  ros::param::getCached("/object_clustring/cluster_tolerans",cluster_tolerans);
  ros::param::getCached("/object_clustring/cluster_size/min",cluster_size_min);
  ros::param::getCached("/object_clustring/cluster_size/max",cluster_size_max);
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_planes (new pcl::PointCloud<pcl::PointXYZ>);
  pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2,*cloud_without_planes);

  if(!(*cloud_without_planes).empty()){
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_without_planes);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (cluster_tolerans);
    ec.setMinClusterSize (cluster_size_min);
    ec.setMaxClusterSize (cluster_size_max);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_without_planes);
    ec.extract (cluster_indices);

    // For each cluster calculate centroids and store in coordinates array
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
      bool object = true;
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
        if(cloud_without_planes->points[*pit].z>1){
          object = false;
        }
      }
      if(object){
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
          cloud_cluster->points.push_back (cloud_without_planes->points[*pit]);
        }
      }
    }
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    (*cloud_cluster).header.frame_id = (*cloud_without_planes).header.frame_id;
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_cluster, output);
    pub_debug_pcl.publish(output);
  }
}


// ctrl-c handler in order to save parameters in PCL_shape_classifier_params.yaml
void mySigintHandler(int sig){
  ros::param::set("/object_clustring/cluster_tolerans",cluster_tolerans);
  ros::param::set("/object_clustring/cluster_size/min",cluster_size_min);
  ros::param::set("/object_clustring/cluster_size/max",cluster_size_max);
  system("rosparam dump -v ~/catkin_ws/src/turtlebot_object_detection/parameters/PCL_object_clustering.yaml /object_clustring");
  ros::shutdown();
}


int main (int argc, char** argv){
  system("rosparam load ~/catkin_ws/src/turtlebot_object_detection/parameters/PCL_object_clustering.yaml /object_clustring");
  ros::init (argc, argv, "object_clustring", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;

  // In case of ctrl-c handle that with mySigintHandler
  signal(SIGINT, mySigintHandler);

  // Create a ROS subscriber for the input point cloud (Otherwise topic from voxel grid)
  ros::Subscriber sub = nh.subscribe ("/camera/depth/cloud_without_planes", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud and coordinates of object
  pub_debug_pcl = nh.advertise<sensor_msgs::PointCloud2> ("/camera/depth/object_clusters", 1);
  //xyz_pub = nh.advertise<std_msgs::Float32MultiArray> ("/PCL_woden_shape_recognition", 1);

  // Spin
  ros::spin ();
  return 0;
}
