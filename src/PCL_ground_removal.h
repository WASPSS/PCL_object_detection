#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <signal.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>


void pass_through(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string axis, double min, double max){
 pcl::PassThrough<pcl::PointXYZ> pass;
 pass.setInputCloud(cloud);
 pass.setFilterFieldName(axis);
 pass.setFilterLimits(min, max);
 pass.filter(*cloud);
}

void statistical_outlier_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double mean_k, double stddev_thresh){
 // 3. Statistical outlier removal
 pcl::StatisticalOutlierRemoval <pcl::PointXYZ> sor;
 sor.setInputCloud(cloud);
 sor.setMeanK (mean_k);
 sor.setStddevMulThresh(stddev_thresh);
 sor.filter(*cloud);
}





/*
// Example code for plane segmentation and plane removal

// Checking size of original cloud for thresholding when to stop removing planes
int original_size = (*cloud).width;

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
