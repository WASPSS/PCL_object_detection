#include "PCL_ground_removal.h"

// Initiating and declaring global variables.
ros::Publisher pub;
double pass_through_min = 0.01;
double pass_through_max = 5.0;
double sor_mean_k = 3.0;
double sor_stdev_thresh = 1.0;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){

 ros::param::getCached("/PCL_ground_removal/pass_through/min", pass_through_min);
 ros::param::getCached("/PCL_ground_removal/pass_through/max", pass_through_max);
 ros::param::getCached("/PCL_ground_removal/statistical_outlier_removal/mean_k", sor_mean_k);
 ros::param::getCached("/PCL_ground_removal/statistical_outlier_removal/stddev_mult_thresh", sor_stdev_thresh);

 // Create a container for the output data.
 sensor_msgs::PointCloud2 output;
 // converting from sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
 pcl::PCLPointCloud2 pcl_pc2;
 pcl_conversions::toPCL(*input,pcl_pc2);
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
 pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

 pass_through(cloud, "z", pass_through_min, pass_through_max);
statistical_outlier_removal(cloud,sor_mean_k, sor_stdev_thresh);

 // Publish the data.
 pcl::toROSMsg (*cloud, output);
 pub.publish (output);

}

// ctrl-c handler in order to save parameters in PCL_ground_removal.yaml
void mySigintHandler(int sig){
 // Dump all parameters in PCL_ground_removal.yaml
 ros::param::set("/PCL_ground_removal/pass_through/min", pass_through_min);
 ros::param::set("/PCL_ground_removal/pass_through/max", pass_through_max);
 ros::param::set("/PCL_ground_removal/statistical_outlier_removal/mean_k", sor_mean_k);
 ros::param::set("/PCL_ground_removal/statistical_outlier_removal/stddev_mult_thresh", sor_stdev_thresh);
 system("rosparam dump -v ~/catkin_ws/src/turtlebot_object_detection/parameters/PCL_ground_removal.yaml /PCL_ground_removal");
 ros::shutdown();
}

int main (int argc, char** argv){
 // Load all parameters from PCL_ground_removal.yaml
 system("rosparam load ~/catkin_ws/src/turtlebot_object_detection/parameters/PCL_ground_removal.yaml /PCL_ground_removal");
 // Initialize ROS with NoSigintHandler
 ros::init (argc, argv, "PCL_ground_removal", ros::init_options::NoSigintHandler);
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
