#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <iostream>



int
main ()
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> plane_seg_cloud;
  pcl::PCDReader cloud_reader;
  pcl::PCDWriter cloud_writer;
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  std::string path_input = "/home/ros/test_ws/src/pcl_tutorial/clouds/inputs/";
  std::string path_output = "/home/ros/test_ws/src/pcl_tutorial/clouds/output/";
 
  // Reading the cloud
  cloud_reader.read(path_input+std::string("229639000.pcd"), cloud);

  //   Planner_segmentation
  pcl::SACSegmentation<pcl::PointXYZ> plane_seg;
  plane_seg.setModelType(pcl::SACMODEL_PLANE);
  plane_seg.setMethodType(pcl::SAC_RANSAC);
  plane_seg.setDistanceThreshold(0.01);
  plane_seg.setInputCloud(cloud.makeShared());
  plane_seg.segment(*inliers, *coefficients);


  // Extracting Points
  pcl::ExtractIndices<pcl::PointXYZ> extract_indicies;
  extract_indicies.setInputCloud(cloud.makeShared());
  extract_indicies.setIndices(inliers);
  extract_indicies.setNegative(true);
  extract_indicies.filter(plane_seg_cloud);

  // Write
  cloud_writer.write<pcl::PointXYZ>(path_output+std::string("neg_plane_seg.pcd"), plane_seg_cloud, false);

  return(0);
}




