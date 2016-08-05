#include "object_detection.h"

ros::Publisher pub;
double segmentation_distance_thresh = 0.1;
double segmentation_rate_of_points_left = 0.2;
double sor_mean_k = 3.0;
double sor_stdev_thresh = 1.0;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){

  ros::param::getCached("/PCL_floor_and_wall_removal/plane_segmentation/distance_threshold", segmentation_distance_thresh);
  ros::param::getCached("/PCL_floor_and_wall_removal/plane_segmentation/rate_of_points_left", segmentation_rate_of_points_left);
  ros::param::getCached("/PCL_floor_and_wall_removal/statistical_outlier_removal/mean_k", sor_mean_k);
  ros::param::getCached("/PCL_floor_and_wall_removal/statistical_outlier_removal/stddev_mult_thresh", sor_stdev_thresh);

  // Create a container for the data.
  sensor_msgs::PointCloud2 output;
  // converting from sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_planes (new pcl::PointCloud<pcl::PointXYZ>);

  // Checking size of original cloud for thresholding when to stop removing planes
  int original_size = (*cloud).width;

  // 1. Including PassThrough filter here. Removes areas that we are not interested in
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0, 5);
  pass.filter(*cloud);

/*
  // Plane segmentation
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (segmentation_distance_thresh);

  // Extract the planar inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setNegative (true);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);
  extract.setIndices (inliers);
  extract.setInputCloud (cloud);
  extract.filter (*cloud_without_planes);


  while((*cloud_without_planes).width>segmentation_rate_of_points_left*original_size){
    seg.setInputCloud (cloud_without_planes);
    seg.segment (*inliers, *coefficients);
    extract.setIndices (inliers);
    extract.setInputCloud (cloud_without_planes);
    extract.filter (*cloud_without_planes);
  }
*/
  // 3. Statistical outlier removal
  pcl::StatisticalOutlierRemoval <pcl::PointXYZ> sor;
  sor.setInputCloud(cloud); //cloud_without_planes);
  sor.setMeanK (sor_mean_k);
  sor.setStddevMulThresh(sor_stdev_thresh);
  sor.filter(*cloud_without_planes);

  // Publish the data.
  pcl::toROSMsg (*cloud_without_planes, output);
  pub.publish (output);

}

// ctrl-c handler in order to save parameters in PCL_floor_and_wall_removal .yaml
void mySigintHandler(int sig){
  // Dump all parameters in objecd_detection_params.yaml
  ros::param::set("/PCL_floor_and_wall_removal/plane_segmentation/distance_threshold", segmentation_distance_thresh);
  ros::param::set("/PCL_floor_and_wall_removal/plane_segmentation/rate_of_points_left", segmentation_rate_of_points_left);
  ros::param::set("/PCL_floor_and_wall_removal/statistical_outlier_removal/mean_k", sor_mean_k);
  ros::param::set("/PCL_floor_and_wall_removal/statistical_outlier_removal/stddev_mult_thresh", sor_stdev_thresh);
  system("rosparam dump -v ~/catkin_ws/src/turtlebot_object_detection/parameters/PCL_floor_and_wall_removal.yaml /PCL_floor_and_wall_removal");
  ros::shutdown();
}

int main (int argc, char** argv){
  // Initialize ROS with NoSigintHandler
  system("rosparam load ~/catkin_ws/src/turtlebot_object_detection/parameters/PCL_floor_and_wall_removal.yaml /PCL_floor_and_wall_removal");
  ros::init (argc, argv, "floor_and_wall_removal", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;

  // In case of ctrl-c handle that with mySigintHandler
  signal(SIGINT, mySigintHandler);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/cloud_transformed", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/camera/depth/cloud_without_planes", 1);

  // Spin
  ros::spin ();
}
