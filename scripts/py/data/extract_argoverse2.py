
# Created: 2023-04-17 18:05
# @Copyright (C) 2023-now, RPL, KTH Royal Institute of Technology
# @Author: Kin ZHANG  (https://kin-zhang.github.io/)
# @Detail: This script is used to extract the point cloud data from AV2 dataset.
# Check more argoverse dataset here: https://www.argoverse.org/av2.html#lidar-link
# 
# If you find this repo helpful, please cite the respective publication in DynamicBenchmark.
# This script is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import av2.utils.io as io_utils
from av2.utils.typing import NDArrayFloat
import numpy as np
from scipy.spatial.transform import Rotation as R
from pathlib import Path

from tqdm import tqdm
import sys, os
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..' ))
sys.path.append(BASE_DIR)

from utils.pcdpy3 import save_pcd
from utils import bc

DATA_FOLDER = "/home/kin/bags/av2/07YOTznatmYypvQYpzviEcU3yGPsyaGg__Spring_2020" # The one we used in paper
SAVE_FOLDER = "/home/kin/data/av2/pcd"

if __name__ == "__main__":
    cetner_scense_xyz = np.array([0,0,0])
    save_folder = DATA_FOLDER.split('/')[-1][:4]
    if not os.path.exists(SAVE_FOLDER):
        os.makedirs(SAVE_FOLDER)
        print(f"{bc.OKGREEN}Create{bc.ENDC} {SAVE_FOLDER} Since it does not exist..")

    # list all files in the folder
    pts_files = sorted(os.listdir(f"{DATA_FOLDER}/sensors/lidar"))
    poses_dict = io_utils.read_city_SE3_ego(Path(DATA_FOLDER))
    sensor_pose_dict = io_utils.read_ego_SE3_sensor(Path(DATA_FOLDER))
    ego2up_lidar = sensor_pose_dict['up_lidar'] # but some problem here
    # check more info: https://argoverse.github.io/user-guide/datasets/lidar.html
    
    for i, pts_file in tqdm(enumerate(pts_files), total=len(pts_files)):
        frame_index = pts_file.split('.')[0]
        abs_path_pts_file = Path(f"{DATA_FOLDER}/sensors/lidar/{pts_file}")
        pose = poses_dict.get(int(frame_index), None)
        if pose is None:
            print(f"frame {frame_index} has no pose")
            continue

        pose2origin = ego2up_lidar.inverse().compose(pose)
        if i == 0:
            cetner_scense_xyz = np.array([pose.translation[0], pose.translation[1], pose.translation[2]])
            pose2origin.translation = np.array([0,0,0])
        else:
            pose2origin.translation = pose.translation - cetner_scense_xyz

        # extract point cloud data with intensity
        sweep_df = io_utils.read_feather(abs_path_pts_file)
        data_xyzi: NDArrayFloat = sweep_df[["x", "y", "z", "intensity"]].to_numpy().astype(np.float64)

        # detail view you can know the data frame now is based on ego
        ego2up_lidar.rotation = np.eye(3)
        data = ego2up_lidar.inverse().transform_point_cloud(data_xyzi[:, :3])

        # NOTE: We transform the point cloud to the world frame based on pose
        data = pose2origin.transform_point_cloud(data[:,:3])
        
        data = np.hstack((data, data_xyzi[:, 3].reshape(-1, 1)))

        # need qw, qx, qy, qz to save pcd in VIEWPOINT
        qxyzw = R.from_matrix(pose2origin.rotation).as_quat()  # quat
        pose_array = [pose2origin.translation[0], pose2origin.translation[1], pose2origin.translation[2], \
                    qxyzw[3], qxyzw[0], qxyzw[1], qxyzw[2]]
        save_pcd(f"{SAVE_FOLDER}/{i:06d}.pcd", data, pose_array)

    print(f"{bc.OKGREEN}Done{bc.ENDC} Check: {SAVE_FOLDER}..")