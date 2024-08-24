/**
 * @date: 2023-12-16 22:20
 * @author: Qingwen Zhang(https://kin-zhang.github.io/)
 * Copyright (C) 2023-now, RPL, KTH Royal Institute of Technology
 * 
 * @details: Translate our pcds to rosbag
 *  Input: DynamicMap Benchmark Dataset format, check our repo: https://github.com/KTH-RPL/DynamicMap_Benchmark
 *  Output: a rosbag file with PointCloud2 messages [already transformed to world frame.] and Odometry messages
 * 
 * Example run: ./build/pcds_to_rosbag /home/kin/bags/231209/clean_pcd /home/kin/test/mybag.bag
 *
 * This file is part of DynamicMap_Benchmark (https://github.com/KTH-RPL/DynamicMap_Benchmark).
 * If you find this repo helpful, please cite the respective publication as 
 * listed on the above website.
 */

#include <iostream>
#include <string>
#include <fstream>
#include <filesystem>
#include <glog/logging.h>


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "timer.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>
#include <tf/tf.h>
#include <pcl_conversions/pcl_conversions.h>

using PointType = pcl::PointXYZ;
int main(int argc, char* argv[]) {

    ros::init(argc, argv, "pcds_to_rosbag");

    // Create a NodeHandle object
    ros::NodeHandle nh;

	google::InitGoogleLogging(argv[0]);
	google::InstallFailureSignalHandler();
	FLAGS_colorlogtostderr = true;
	google::SetStderrLogging(google::INFO);

    if(argc < 3) {
        LOG(ERROR) << "Usage: " << argv[0] << " [pcd_folder] [save_bag_path], e.g. /data/00/pcd /data/myrosbag.bag";
        return 1;
    }

    std::string pcd_folder = argv[1];
    std::string bag_save_path = argv[2];
    // check if the folder exists
    if (!std::filesystem::exists(pcd_folder)) {
        LOG(ERROR) << "File does not exist: " << pcd_folder;
        return 1;
    }

    std::string bag_folder = bag_save_path.substr(0, bag_save_path.find_last_of("/"));
    if (!std::filesystem::exists(bag_folder)) {
        std::filesystem::create_directories(bag_folder);
        LOG(INFO) << "Create rosbag path: " << bag_folder;
    }

    TIC;
    // sorted
    std::vector<std::string> filenames;
    std::vector<std::string> only_filenames;
    for (const auto & entry : std::filesystem::directory_iterator(pcd_folder)) {
        filenames.push_back(entry.path().string());
    }
    std::sort(filenames.begin(), filenames.end());

    rosbag::Bag bag;
    bag.open(bag_save_path, rosbag::bagmode::Write);
    ros::Time start_time = ros::Time::now();
    int i = 0;
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

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        output.header.frame_id = "map";

        nav_msgs::Odometry odom;
        odom.header.frame_id = "map";

        odom.pose.pose.position.x = cloud->sensor_origin_[0];
        odom.pose.pose.position.y = cloud->sensor_origin_[1];
        odom.pose.pose.position.z = cloud->sensor_origin_[2];

        odom.pose.pose.orientation.x = cloud->sensor_orientation_.x();
        odom.pose.pose.orientation.y = cloud->sensor_orientation_.y();
        odom.pose.pose.orientation.z = cloud->sensor_orientation_.z();
        odom.pose.pose.orientation.w = cloud->sensor_orientation_.w();

        ros::Time current_time = start_time + ros::Duration(0.1 * i); // 10Hz
        bag.write("/point_cloud", current_time, output);
        bag.write("/odometry", current_time, odom);
        i++;
    }
    bag.close();

    TOC("Translate to Rosbag", true);
    return 0;
}