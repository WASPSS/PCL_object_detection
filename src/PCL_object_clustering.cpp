#include "PCL_object_clustering.h"


ros::Publisher pub_debug_pcl;
ros::Publisher xyz_pub, pub_centroid;
double cluster_tolerans = 0.01;
int cluster_size_min = 20;
int cluster_size_max = 500;
double clusters_highest_point = 0.5;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
 // Updating parameters from the parameter server
 ros::param::getCached("/PCL_object_clustering/cluster_tolerans",cluster_tolerans);
 ros::param::getCached("/PCL_object_clustering/cluster_size/min",cluster_size_min);
 ros::param::getCached("/PCL_object_clustering/cluster_size/max",cluster_size_max);
 ros::param::getCached("/PCL_object_clustering/clusters_highest_point",clusters_highest_point);

 // Converting from PointCloud2 msg to pcl::PointCloud
 pcl::PCLPointCloud2 pcl_pc2;
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_planes (new pcl::PointCloud<pcl::PointXYZ>);
 pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
 pcl::fromPCLPointCloud2(pcl_pc2,*cloud_without_planes);

 if(!(*cloud_without_planes).empty()){
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_without_planes);
  // Getting indeces for each found cluster with parameters cluster_tolerans,
  // cluster_size_min and cluster_size_max
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (cluster_tolerans);
  ec.setMinClusterSize (cluster_size_min);
  ec.setMaxClusterSize (cluster_size_max);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_without_planes);
  ec.extract (cluster_indices);

  /* For each cluster we store cluster in tmp_cloud, calculate its centroids.
     If cluster highest point is less than clusters_highest_point we publish
     centroid and append tmp_cloud to cloud_cluster. When we checked through all
     possible centroids we publish cloud_cluster to visualize in rviz.*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
   pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
   for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
    tmp_cloud->points.push_back (cloud_without_planes->points[*pit]);
   }
   pcl::PointXYZ min_point, max_point;
   pcl::getMinMax3D(*tmp_cloud,min_point,max_point);
   if(max_point.z < clusters_highest_point){
    Eigen::Vector4f c;
    pcl::compute3DCentroid(*tmp_cloud, c);
    //std::cout << "Centroid is " << c << std::endl;
    object_detecter_2d::object_loc object_loc_msg;
    object_loc_msg.ID = 10;
    object_loc_msg.point.x = c(0);
    object_loc_msg.point.y = c(1);
    object_loc_msg.point.z = c(2);
    pub_centroid.publish(object_loc_msg);
    (*cloud_cluster) = (*cloud_cluster)+(*tmp_cloud);

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
 ros::param::set("/PCL_object_clustering/cluster_tolerans",cluster_tolerans);
 ros::param::set("/PCL_object_clustering/cluster_size/min",cluster_size_min);
 ros::param::set("/PCL_object_clustering/cluster_size/max",cluster_size_max);
 ros::param::set("/PCL_object_clustering/clusters_highest_point",clusters_highest_point);
 system("rosparam dump -v ~/catkin_ws/src/turtlebot_object_detection/parameters/PCL_object_clustering.yaml /PCL_object_clustering");
 ros::shutdown();
}


int main (int argc, char** argv){
 system("rosparam load ~/catkin_ws/src/turtlebot_object_detection/parameters/PCL_object_clustering.yaml /PCL_object_clustering");
 ros::init (argc, argv, "PCL_object_clustering", ros::init_options::NoSigintHandler);
 ros::NodeHandle nh;

 // In case of ctrl-c handle that with mySigintHandler
 signal(SIGINT, mySigintHandler);

 // Create a ROS subscriber for the input point cloud (Otherwise topic from voxel grid)
 ros::Subscriber sub = nh.subscribe ("/camera/depth/cloud_without_planes", 1, cloud_cb);

 // Create a ROS publisher for the output point cloud and coordinates of object
 pub_debug_pcl = nh.advertise<sensor_msgs::PointCloud2> ("/camera/depth/object_clusters", 1);
 pub_centroid = nh.advertise<object_detecter_2d::object_loc> ("/object_location", 10);
 //xyz_pub = nh.advertise<std_msgs::Float32MultiArray> ("/PCL_woden_shape_recognition", 1);

 // Spin
 ros::spin ();
 return 0;
}
