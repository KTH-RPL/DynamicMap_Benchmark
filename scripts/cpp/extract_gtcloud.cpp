/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * MIT License
 * Author: Kin ZHANG (https://kin-zhang.github.io/)
 * Date: 2023-03-28 21:10
 * Description: extract the ground truth kitti dataset
 *              set intensity to 0 for static
 *              set intensity to 1 for dynamic
 */

#include <iostream>
#include <string>
#include <fstream>
#include <filesystem>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <glog/logging.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "timer.h"

int main(int argc, char* argv[]) {

	google::InitGoogleLogging(argv[0]);
	google::InstallFailureSignalHandler();
	FLAGS_colorlogtostderr = true;
	google::SetStderrLogging(google::INFO);

    if(argc < 2) {
        LOG(ERROR) << "Usage: " << argv[0] << " [pcd_folder, e.g. /data/00/pcd]";
        return 0;
    }

    std::string pcd_folder = argv[1];
    int downsample_flag = 0;
    double grid_size = 0.2; // 0.2m same with ERASOR downsample ground truth, but benchmark didn't downsample!

    if(argc == 2) {
        LOG(INFO) << "No downsample flag, set to 0, save all points";
    }
    else{
        downsample_flag = std::stoi(argv[2]);
        LOG(INFO) << "Set Downsample, will downsample with grid: " << grid_size;
    }
    
    TIC;
    // sorted
    std::vector<std::string> filenames;
    for (const auto & entry : std::filesystem::directory_iterator(pcd_folder)) {
        filenames.push_back(entry.path().string());
    }
    std::sort(filenames.begin(), filenames.end());

    pcl::PointCloud<pcl::PointXYZI>::Ptr gt_cloud (new pcl::PointCloud<pcl::PointXYZI>());
    // pcl::PointCloud<pcl::PointXYZI>::Ptr gt_cloud_intensity (new pcl::PointCloud<pcl::PointXYZI>());

    
    // HARD CODE HERE: since ego vehicle pts is labeled as unlabeled we will set to dynamic pts also
    float max_dist_square = pow(3, 2); // 2.71 is two wheel base
    int cnt = 0;


    for (const auto & filename : filenames) {
        // check if the file is pcd
        if (filename.find(".pcd") == std::string::npos) {
            continue;
        }
        std::size_t pos1 = filename.find_last_of('/');
        std::size_t pos2 = filename.find_last_of('.');
        std::string number_string = filename.substr(pos1 + 1, pos2 - pos1 - 1);
        int frame_id_int = std::stoi(number_string);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_intensity (new pcl::PointCloud<pcl::PointXYZI>());


        if (pcl::io::loadPCDFile<pcl::PointXYZI> (filename, *cloud) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
            return (-1);
        }

        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose(0, 3) = cloud->sensor_origin_[0];
        pose(1, 3) = cloud->sensor_origin_[1];
        pose(2, 3) = cloud->sensor_origin_[2];
        Eigen::Quaternionf q(cloud->sensor_orientation_.w(), cloud->sensor_orientation_.x(), cloud->sensor_orientation_.y(), cloud->sensor_orientation_.z());
        pose.block<3, 3>(0, 0) = q.toRotationMatrix();

        // copy cloud to cloud_intensity -> no modification for intensity
        pcl::copyPointCloud(*cloud, *cloud_intensity);

        // extract post from VIEWPOINT in pcl
        float x = cloud->sensor_origin_[0];
        float y = cloud->sensor_origin_[1];

        for(pcl::PointCloud<pcl::PointXYZI>::iterator it = cloud->begin(); it != cloud->end(); ++it) {
            uint32_t label = it->intensity;
            int semantic_id = label & 0xFFFF;
            double dist_square = pow(it->x - x, 2) + pow(it->y - y, 2);

            // HARD CODE HERE: since ego vehicle pts is labeled as unlabeled/outlier we will set to dynamic pts also
            if (dist_square < max_dist_square && (semantic_id == 0 || semantic_id == 1)) {
                it->intensity = 252;
                cloud_intensity->at(it - cloud->begin()).intensity = 252; // modify the intensity to ego dynamic
                semantic_id = 252;
            }
            
            // transform intensity to label as uint32_t, check semantic KITTI config file:
            // https://github.com/PRBonn/semantic-kitti-api/blob/master/config/semantic-kitti-mos.yaml#L33-L41
            if (semantic_id == 252|| semantic_id == 253 || semantic_id == 254 || semantic_id == 255 
                || semantic_id == 256 || semantic_id == 257 || semantic_id == 258 || semantic_id == 259) {
                it->intensity = 1;
            } else {
                it->intensity = 0;
            }
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

        if(downsample_flag!=0){
            pcl::VoxelGrid<pcl::PointXYZI> sor;
            sor.setInputCloud(cloud);
            sor.setLeafSize(grid_size, grid_size, grid_size);
            sor.filter(*cloud_filtered);
            // 2. Find nearest point to update intensity (index and id)
            pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
            kdtree.setInputCloud(cloud);

            int K = 1;
            std::vector<int>   pointIdxNKNSearch(K);
            std::vector<float> pointNKNSquaredDistance(K);

            // Set dst <- output
            for (auto &pt: cloud_filtered->points) {
                if (kdtree.nearestKSearch(pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
                    pt.intensity = (*cloud)[pointIdxNKNSearch[0]].intensity;
                }
            }
        }
        else{
            cloud_filtered = cloud;
        }

        // transform the point cloud ---> If you already transform to world no need
        // pcl::transformPointCloud(*cloud_filtered, *cloud_filtered, pose);
        *gt_cloud += *cloud_filtered;
        // *gt_cloud_intensity += *cloud_intensity;
        cnt++;
    }
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    if(downsample_flag!=0){
        
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setInputCloud(gt_cloud);
        sor.setLeafSize(grid_size, grid_size, grid_size);
        sor.filter(*cloud_filtered);
        // 2. Find nearest point to update intensity (index and id)
        pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
        kdtree.setInputCloud(gt_cloud);

        int K = 1;
        std::vector<int>   pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        // Set dst <- output
        for (auto &pt: cloud_filtered->points) {
            if (kdtree.nearestKSearch(pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
                pt.intensity = (*gt_cloud)[pointIdxNKNSearch[0]].intensity;
            }
        }
    }
    else{
        cloud_filtered = gt_cloud;
    }
    // file name in last folder pcd_folder
    std::filesystem::path folder_path(pcd_folder);
    std::filesystem::path gtcloud_path = folder_path.parent_path() / "gt_cloud.pcd";
    pcl::io::savePCDFileBinary(gtcloud_path.string(), *cloud_filtered);

    LOG(INFO) << "gt_cloud saved to " << gtcloud_path.string() << " Check file there";
    TOC("extract_gtcloud", true);

    return 0;
}
