/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * MIT License
 * @author: Kin ZHANG (https://kin-zhang.github.io/)
 * @date: 2023-04-12 18:14
 * @details: TODO
 * 
 * Input: Algorithm Result PCD file, and GT PC file
 * Output: raw map pcd file with label 0/1
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include <glog/logging.h>
#include "timer.h"


int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    FLAGS_colorlogtostderr = true;
    google::SetStderrLogging(google::INFO);

    if (argc < 4) {
        LOG(ERROR) << "Usage: " << argv[0] << " [pcd_folder] [algorithm_output.pcd] [min_dis_cnt_as_same]";
        return 1;
    }

    std::string pcd_folder = argv[1];
    std::string run_pcd_name = argv[2];
    float min_dis_cnt_as_same = std::stof(argv[3]);

    // check if the folder exists
    if (!std::filesystem::exists(pcd_folder)) {
        LOG(ERROR) << "File does not exist: " << pcd_folder;
        return -1;
    }
    std::string export_pcd_name = run_pcd_name.substr(0, run_pcd_name.size() - 4) + "_exportGT.pcd";
    std::filesystem::path folder_path(pcd_folder);
    if (!std::filesystem::exists(folder_path / run_pcd_name)) {
        LOG(ERROR) << "File does not exist: " << run_pcd_name << " in " << pcd_folder;
        return -1;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr gt_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr et_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(folder_path / "gt_cloud.pcd", *gt_cloud) == -1) {
        LOG(ERROR) << "Couldn't read gt_cloud.pcd";
        return -1;
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(folder_path / run_pcd_name, *et_cloud) == -1) {
        LOG(ERROR) << "Couldn't read:" << run_pcd_name;
        return -1;
    }

    TIC;
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(et_cloud);

    // check whether every gt point is in the et_cloud
    for (auto& point : gt_cloud->points) {
        std::vector<int> nearest_indices(1);
        std::vector<float> nearest_distances(1);

        // if the nearest point in et_cloud for gt is too far, then this gt point is not in the et_cloud
        // which means mark as dynamic since et_cloud remove this one.
        if (kdtree.nearestKSearch(point, 1, nearest_indices, nearest_distances) > 0) {
            // Revised Code
            auto point_et = et_cloud->points[nearest_indices[0]];
            pcl::PointXYZI left_bottom_corner = point_et;
            left_bottom_corner.x -= min_dis_cnt_as_same;
            left_bottom_corner.y -= min_dis_cnt_as_same;
            left_bottom_corner.z -= min_dis_cnt_as_same;

            pcl::PointXYZI right_top_corner = point_et;
            right_top_corner.x += min_dis_cnt_as_same;
            right_top_corner.y += min_dis_cnt_as_same;
            right_top_corner.z += min_dis_cnt_as_same;

            if ((point.x >= left_bottom_corner.x && point.x <= right_top_corner.x) &&
                (point.y >= left_bottom_corner.y && point.y <= right_top_corner.y) &&
                (point.z >= left_bottom_corner.z && point.z <= right_top_corner.z)) {
                point.intensity = 0;
            } else {
                point.intensity = 1;
            }

            // Original Code
            // float distance = std::sqrt(nearest_distances[0]);
            // if (distance > min_dis_cnt_as_same) {
            //     point.intensity = 1;
            // } else {
            //     point.intensity = 0;
            // }
        } else {
            std::cout << "Search failed" << std::endl;
        }
    }
    if (!std::filesystem::exists(folder_path / "eval")) {
        std::filesystem::create_directory(folder_path / "eval");
    }
    pcl::io::savePCDFileBinary(folder_path / "eval" / export_pcd_name, *gt_cloud);
    LOG(INFO) << "Exported GT PCD file: " << export_pcd_name << " in " << pcd_folder;
    TOC("Export Estimate Label PCD file based on GT", true);
    return 0;
}
