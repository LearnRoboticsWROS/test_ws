#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>



void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*cloud_msg, cloud);
  //pcl::PCDWriter cloud_writer;
  //std::string path = "~/test_ws/src/pcl_tutorial/clouds/";

  //cloud_writer.write<pcl::PointXYZ>(path + std::string("plane_seg.pcd"), cloud, false);
  //pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
  char filename[100];
  sprintf(filename, "/home/learnroboticswros/test_ws/src/pcl_tutorial/clouds/test.pcd");
  pcl::io::savePCDFileASCII (filename, cloud);
 
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "poit_cloud_plane_segmentation");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, cloud_cb);
  //ros::Publisher pub;
  // Create a ROS publisher for the output point cloud
  //pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  // ros::spin ();
  return (0);

}


// #include <iostream>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// int
//   main ()
// {
//   pcl::PointCloud<pcl::PointXYZ> cloud;
//   // Fill in the cloud data
//   cloud.width    = 5;
//   cloud.height   = 1;
//   cloud.is_dense = false;
//   cloud.resize (cloud.width * cloud.height);
//   for (auto& point: cloud)
//   {
//     point.x = 1024 * rand () / (RAND_MAX + 1.0f);
//     point.y = 1024 * rand () / (RAND_MAX + 1.0f);
//     point.z = 1024 * rand () / (RAND_MAX + 1.0f);
//   }
//   pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
//   std::cerr << "Saved " << cloud.size () << " data points to test_pcd.pcd." << std::endl;
//   for (const auto& point: cloud)
//     std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;
//   return (0);
// }