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
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

ros::Publisher tf_pub;
tf::TransformListener *tf_listener;

void callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_out(new pcl::PointCloud<pcl::PointXYZ>);
  pcl_conversions::toPCL(*input,pcl_pc2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_in(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*pcl_in);

  ros::Time now = ros::Time::now();
  tf_listener->waitForTransform("/base_link", (*pcl_in).header.frame_id, now, ros::Duration(5.0));
  pcl_ros::transformPointCloud("/base_link", *pcl_in, *pcl_out, *tf_listener);


  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg (*pcl_out, output);
  tf_pub.publish(output);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "PCL_transform");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/camera/depth/filtered_points", 1, callback);
  tf_pub = nh.advertise<sensor_msgs::PointCloud2> ("/camera/depth/cloud_transformed", 1);

  tf_listener    = new tf::TransformListener();

  ros::spin();
  //delete tf_listener;
  //return 0;
}
