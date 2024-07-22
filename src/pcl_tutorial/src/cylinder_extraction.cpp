#include <iostream>
#include <filesystem>

#include <math.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZ PointT;


void cloud_saver(const std::string& file_name,std::string& path, pcl::PointCloud<PointT>::Ptr cloud_arg){
    pcl::PCDWriter cloud_writer;
    cloud_writer.write<pcl::PointXYZ>(path+std::string(file_name),*cloud_arg, false);
}

int main(){
    // Basic Cloud objects
    pcl::PointCloud<PointT>::Ptr cloud        (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr voxel_cloud  (new pcl::PointCloud<PointT>);
    pcl::ModelCoefficients::Ptr  coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr       inliers      (new pcl::PointIndices);
    pcl::PCDReader               cloud_reader;
    pcl::PCDWriter               cloud_writer;



    // Cloud Loading
    std::string path_input="/home/ros/test_ws/src/pcl_tutorial/clouds/inputs/";
    cloud_reader.read (path_input+std::string("neg_plane_seg.pcd"),*cloud);

    // Voxel filter applying
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(0.025,0.025,0.025); // play
    voxel_filter.filter(*voxel_cloud);

   
    //cloud_saver("test_voxel.pcd", path_output, voxel_cloud);

    // Normals computation objects
    pcl::NormalEstimation<PointT,pcl::Normal>            normals_estimator;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> () );
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals  (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr       cylinder_co    (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr            cylinder_in    (new pcl::PointIndices);

    pcl::ExtractIndices<PointT>  extract_cylinder;
    pcl::PointCloud<PointT>::Ptr  extracted_cylinder (new pcl::PointCloud<PointT> ());
    pcl::ExtractIndices<pcl::Normal>   extract_cylinder_temp;


    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> cylinder_segmentor;
   
    normals_estimator.setSearchMethod(tree);
    normals_estimator.setInputCloud(voxel_cloud);
    normals_estimator.setKSearch(50); // play
    normals_estimator.compute(*cloud_normals);


    // Parameters for segmentation
    cylinder_segmentor.setOptimizeCoefficients(true);
	cylinder_segmentor.setModelType(pcl::SACMODEL_CYLINDER);
	cylinder_segmentor.setMethodType(pcl::SAC_RANSAC);
	cylinder_segmentor.setNormalDistanceWeight(0.1);
	cylinder_segmentor.setMaxIterations(10000);
	cylinder_segmentor.setDistanceThreshold(0.01); // play
	cylinder_segmentor.setRadiusLimits(0.01, 0.1); // play

    std::string path_output="/home/ros/test_ws/src/pcl_tutorial/clouds/output/";
    int l=0;
    while(true)
    {
        // Perform the segmentation
        cylinder_segmentor.setInputCloud(voxel_cloud);
        cylinder_segmentor.setInputNormals(cloud_normals);
        cylinder_segmentor.segment(*cylinder_in,*cylinder_co);

        // extracting indices
        extract_cylinder.setInputCloud(voxel_cloud);
        extract_cylinder.setIndices(cylinder_in);
        extract_cylinder.setNegative(false);
        extract_cylinder.filter(*extracted_cylinder);

        if (!extracted_cylinder ->points.empty())
        {
            std::stringstream ss;
            ss << "ex_cylinder"<<l<<".pcd";
            cloud_saver(ss.str() , path_output, extracted_cylinder);
            l++;
            extract_cylinder.setNegative(true);
            extract_cylinder.filter(*voxel_cloud);

            // processing normals
            extract_cylinder_temp.setInputCloud(cloud_normals);
            extract_cylinder_temp.setIndices(cylinder_in);
            extract_cylinder_temp.setNegative(true);
            extract_cylinder_temp.filter(*cloud_normals);


        }
        else
        {
            return 0;
        }


        
        // cloud_saver("extracted_cylinder.pcd" , path_output, extracted_cylinder);
    }


    return 0;
}