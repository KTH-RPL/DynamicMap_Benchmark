/**
 * @date: 2023-04-04 18:29
 * @author: Qingwen Zhang(https://kin-zhang.github.io/)
 * Copyright (C) 2023-now, RPL, KTH Royal Institute of Technology
 * 
 * @details: extract pcd file and also insert the pose in PCD VIEWPOINT Field
 * So that we don't need pose.txt, pose.csv file etc.
 *  Input: PCD files with pose in PCD VIEWPOINT Field
 *  Output: raw map pcd file
 * 
 * This file is part of DynamicMap_Benchmark (https://github.com/KTH-RPL/DynamicMap_Benchmark).
 * If you find this repo helpful, please cite the respective publication as 
 * listed on the above website.
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include <glog/logging.h>
#include "timer.h"

using PointType = pcl::PointXYZI;
// using PointType = pcl::PointXYZRGB;
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    FLAGS_colorlogtostderr = true;
    google::SetStderrLogging(google::INFO);

    // if no argv, return 0
    if (argc < 2) {
        LOG(ERROR) << "Usage: " << argv[0] << " [pcd_folder, e.g. /data/00/pcd]";
        return 1;
    }
    std::string pcd_folder = argv[1];
    // check if the folder exists
    if (!std::filesystem::exists(pcd_folder)) {
        LOG(ERROR) << "File does not exist: " << pcd_folder;
        return 1;
    }

    TIC;
    // sorted
    std::vector<std::string> filenames;
    for (const auto & entry : std::filesystem::directory_iterator(pcd_folder)) {
        filenames.push_back(entry.path().string());
    }

    pcl::PointCloud<PointType>::Ptr rawmap(new pcl::PointCloud<PointType>);
    for (const auto & filename : filenames) {
        // check if the file is pcd
        if (filename.find(".pcd") == std::string::npos) {
            continue;
        }
        // read pcd file
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
        if (pcl::io::loadPCDFile<PointType>(filename, *cloud) == -1) {
            LOG(ERROR) << "Couldn't read file " << filename;
            return 1;
        }
        // get the pose
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose(0, 3) = cloud->sensor_origin_[0];
        pose(1, 3) = cloud->sensor_origin_[1];
        pose(2, 3) = cloud->sensor_origin_[2];
        Eigen::Quaternionf q(cloud->sensor_orientation_.w(), cloud->sensor_orientation_.x(), cloud->sensor_orientation_.y(), cloud->sensor_orientation_.z());
        pose.block<3, 3>(0, 0) = q.toRotationMatrix();

        // NOTE: transform the point cloud ---> If you already transform to world no need
        // pcl::transformPointCloud(*cloud, *cloud, pose);

        // add to rawmap
        *rawmap += *cloud;
    }
    // file name in last folder pcd_folder
    std::filesystem::path folder_path(pcd_folder);
    std::filesystem::path rawmap_path = folder_path.parent_path() / "raw_map.pcd";
    pcl::io::savePCDFileBinary(rawmap_path.string(), *rawmap);
    LOG(INFO) << "Saved raw map... there are " << rawmap->size() << " points to " << rawmap_path.string();
    return 0;
}
