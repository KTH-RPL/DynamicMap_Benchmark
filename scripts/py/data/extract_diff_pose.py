'''
# Created: 2024-01-22 17:35
# Copyright (C) 2024-now, RPL, KTH Royal Institute of Technology
# Author: Qingwen ZHANG  (https://kin-zhang.github.io/)
#
# If you find this repo helpful, please cite the respective publication in DynamicBenchmark.
# This script is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
# 
# Description: extract different poses result:
# 1. from the SemanticKITTI (with SuMa more detail: https://github.com/PRBonn/semantic-kitti-api/issues/140
# 2. from the kiss-icp result (with kiss-icp more detail: https://github.com/PRBonn/kiss-icp/issues/267
# 3. from the KITTI odometry dataset (with KITTI more detail: https://www.cvlibs.net/datasets/kitti/eval_odometry.php
'''


import sys, os
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..' ))
sys.path.append(BASE_DIR)


import numpy as np
# reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.as_quat.html
from scipy.spatial.transform import Rotation as R
from utils import bc
from utils.semkitti_api import parse_calibration, parse_poses
from utils.pcdpy3 import save_pcd
from av2.geometry.se3 import SE3

# data format for different pose results, reorganize by hand
DATA_PATH = "/home/kin/data/KITTI/pose_diff/" # [folder: velodyne, labels, files: times.txt, calib.txt, semantic_kitti_pose.txt, kitti_pose.txt, kiss_icp_pose.txt]
SEQUENCE = "00"
SAVE_DATA_PATH = f"/home/kin/data/{SEQUENCE}" # will add `pose/pcd` to the end of the folder name

if __name__ == "__main__":
    # selected by ERASOR: https://github.com/LimHyungTae/ERASOR/blob/master/scripts/semantickitti2bag/kitti2node.py
    FIX_SEQ_FRAME_RANGE = {"00": [4390, 4530], 
                        "01": [150, 250], 
                        "02": [860, 950], 
                        "05": [2350, 2670],
                        "07": [630, 820]}
    
    BASE_PATH = os.path.dirname(os.path.abspath(__file__))

    # get the path of the data
    semantic_kitti_pose = os.path.join(DATA_PATH + SEQUENCE + "/semantic_kitti_pose.txt")
    kitti_pose = os.path.join(DATA_PATH + SEQUENCE + "/kitti_pose.txt")
    kiss_icp_pose = os.path.join(DATA_PATH + SEQUENCE + "/kiss_icp_pose.txt")
    calib_file = os.path.join(DATA_PATH + SEQUENCE + "/calib.txt")

    # folder
    pts_folder    = os.path.join(DATA_PATH + SEQUENCE + "/velodyne")
    labels_folder = os.path.join(DATA_PATH + SEQUENCE + "/labels")

    print(f"pcd_files from folder: {pts_folder}")

    # Check whether folder and files exist
    for i in [pts_folder, labels_folder]:
        assert os.path.isdir(i), f"{i} does not exist, {bc.FAIL}please check the path{bc.ENDC}"
    for i in [semantic_kitti_pose, kitti_pose, kiss_icp_pose]:
        assert os.path.isfile(i), f"{i} does not exist, {bc.FAIL}please check the path{bc.ENDC}"

    if SEQUENCE in FIX_SEQ_FRAME_RANGE:
        start, end = FIX_SEQ_FRAME_RANGE[SEQUENCE]
        print(f"Extract {SEQUENCE} from {start} to {end}")

    pts_files = sorted(os.listdir(pts_folder))

    for pose_file in [semantic_kitti_pose]:
        ### Extract pose
        odom_name = pose_file.split('/')[-1].replace('_pose.txt', '')
        poses_list = parse_poses(pose_file, parse_calibration(calib_file))
        SaveDataFolder = f"{SAVE_DATA_PATH}/{odom_name}/pcd"
        if not os.path.exists(SaveDataFolder):
            os.makedirs(SaveDataFolder)
            print(f"{bc.OKGREEN}Create{bc.ENDC} {SaveDataFolder} Since it does not exist..")
            
        for i, pts_file in enumerate(pts_files):
            frame_index = pts_file.split('.')[0]
            if start <= int(frame_index) <= end:
                # get label
                label_file = frame_index + '.label'

                # get pose
                T = poses_list[int(frame_index)]
                xyz = T[:, -1]  # tran
                qxyzw = R.from_matrix(T[:3, :3]).as_quat()  # quat

                # setup the file path and name
                pcd_file = f"{pts_folder}/{pts_file}"
                label_file = f"{labels_folder}/{label_file}"
                save_pcd_file = f"{SaveDataFolder}/{frame_index}.pcd"

                labels = np.fromfile(label_file, dtype=np.uint32)
                points = np.fromfile(pcd_file, dtype=np.float32).reshape(-1, 4)

                # NOTE: We transform the point cloud to the world frame based on pose
                pose2world = SE3(T[:3, :3], T[:3, -1])
                points = np.hstack((pose2world.transform_point_cloud(points[:,:3]), points[:, 3].reshape(-1, 1)))

                assert labels.shape[0] == points.shape[0], f"labels.shape[0] != points.shape[0], {bc.FAIL}please check the path{bc.ENDC}"
                save_points = np.hstack((points[:, :3], labels.reshape(-1, 1))) # [x, y, z, intensity]
                save_pcd(save_pcd_file, save_points, np.array([xyz[0], xyz[1],xyz[2], qxyzw[3], qxyzw[0], qxyzw[1], qxyzw[2]]))
        os.system(f"/home/kin/code_online/DynamicMap_Benchmark/scripts/build/extract_gtcloud {SaveDataFolder} 1")
    print(f"{os.path.basename( __file__ )}: All codes run {bc.OKGREEN}successfully, Close now.. {bc.ENDC}")
