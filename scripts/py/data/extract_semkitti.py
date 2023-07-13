'''
# Created: 2023-04-04 21:44
# Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
# Author: Kin ZHANG  (https://kin-zhang.github.io/)
#
# If you find this repo helpful, please cite the respective publication in the repo.
# This script is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
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

# original data path when you downloaded the semantic kitti dataset
ORIGIN_PATH = "/home/kin/bags/KITTI/SemanticKitti/"
SEQUENCE = "00"
SaveDataFolder = f"/home/kin/data/{SEQUENCE}/pcd"

if __name__ == "__main__":
    # full frame range for each sequence
    # FIX_SEQ_FRAME_RANGE = {"00": [0, 10000], 
    #                     "01": [0, 10000], 
    #                     "02": [0, 10000], 
    #                     "05": [0, 10000],
    #                     "07": [0, 10000]}

    # selected by ERASOR: https://github.com/LimHyungTae/ERASOR/blob/master/scripts/semantickitti2bag/kitti2node.py
    FIX_SEQ_FRAME_RANGE = {"00": [4390, 4530], 
                        "01": [150, 250], 
                        "02": [860, 950], 
                        "05": [2350, 2670],
                        "07": [630, 820]}
    
    BASE_PATH = os.path.dirname(os.path.abspath(__file__))

    # get the path of the data
    pose_file = os.path.join(ORIGIN_PATH + "data_odometry_labels/dataset/sequences", SEQUENCE, "poses.txt")
    calib_file = os.path.join(ORIGIN_PATH + "data_odometry_calib/dataset/sequences", SEQUENCE, "calib.txt")
    pts_folder = os.path.join(ORIGIN_PATH + "data_odometry_velodyne/dataset/sequences", SEQUENCE, "velodyne")
    labels_folder = os.path.join(ORIGIN_PATH + "data_odometry_labels/dataset/sequences", SEQUENCE, "labels")

    print(f"pcd_files from folder: {pts_folder}")

    # Check whether folder and files exist
    for i in [pts_folder, labels_folder]:
        assert os.path.isdir(i), f"{i} does not exist, {bc.FAIL}please check the path{bc.ENDC}"
    for i in [pose_file]:
        assert os.path.isfile(i), f"{i} does not exist, {bc.FAIL}please check the path{bc.ENDC}"
        
    if not os.path.exists(SaveDataFolder):
        os.makedirs(SaveDataFolder)
        print(f"{bc.OKGREEN}Create{bc.ENDC} {SaveDataFolder} Since it does not exist..")

    ### Extract pose
    poses_list = parse_poses(pose_file, parse_calibration(calib_file))
    
    if SEQUENCE in FIX_SEQ_FRAME_RANGE:
        start, end = FIX_SEQ_FRAME_RANGE[SEQUENCE]
        print(f"Extract {SEQUENCE} from {start} to {end}")

    pts_files = sorted(os.listdir(pts_folder))
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

    print(f"{os.path.basename( __file__ )}: All codes run {bc.OKGREEN}successfully, Close now.. {bc.ENDC}")
