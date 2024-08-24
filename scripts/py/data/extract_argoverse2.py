
"""
# @date: 2023-04-17 18:05
# @author: Qingwen Zhang (https://kin-zhang.github.io/)
# Copyright (C) 2023-now, RPL, KTH Royal Institute of Technology
# 
# @detail: Extract the point cloud data from AV2 dataset.
# Check more Argoverse dataset here: https://www.argoverse.org/av2.html#lidar-link
# 
# This file is part of DynamicMap_Benchmark (https://github.com/KTH-RPL/DynamicMap_Benchmark).
# If you find this repo helpful, please cite the respective publication as 
# listed on the above website.
"""




import numpy as np
from scipy.spatial.transform import Rotation as R
from pathlib import Path
import pandas as pd
from tqdm import tqdm
import sys, os
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..' ))
sys.path.append(BASE_DIR)

import av2.utils.io as io_utils
from av2.utils.typing import NDArrayFloat, NDArrayByte
from av2.datasets.sensor.constants import RingCameras, StereoCameras
from av2.utils.synchronization_database import SynchronizationDB, get_timestamps_from_sensor_folder
from av2.geometry.se3 import SE3
import av2.geometry.geometry as geometry_utils
from av2.geometry.camera.pinhole_camera import PinholeCamera

from utils.pcdpy3 import save_pcd
from utils import bc

DATA_FOLDER = "/home/kin/bags/av2/Dynamic_Map/07YOTznatmYypvQYpzviEcU3yGPsyaGg__Spring_2020" # The one we used in paper
SAVE_FOLDER = "/home/kin/data/av2/pcd"

def convert_pose_dataframe_to_SE3(pose_df: pd.DataFrame) -> SE3:
    """Convert a dataframe with parameterization of a single pose, to an SE(3) object.

    Args:
        pose_df: parameterization of a single pose.

    Returns:
        SE(3) object representing the egovehicle's pose in the city frame.
    """
    qw, qx, qy, qz = pose_df[["qw", "qx", "qy", "qz"]].to_numpy().squeeze()
    tx_m, ty_m, tz_m = pose_df[["tx_m", "ty_m", "tz_m"]].to_numpy().squeeze()
    city_q_ego: NDArrayFloat = np.array([qw, qx, qy, qz])
    city_t_ego: NDArrayFloat = np.array([tx_m, ty_m, tz_m])
    city_R_ego = geometry_utils.quat_to_mat(quat_wxyz=city_q_ego)
    city_SE3_ego = SE3(rotation=city_R_ego, translation=city_t_ego)
    return city_SE3_ego

def get_city_SE3_ego(data_folder: str, timestamp_ns: int) -> SE3:
    log_poses_df = io_utils.read_feather(Path(data_folder/ "city_SE3_egovehicle.feather"))
    pose_df = log_poses_df.loc[log_poses_df["timestamp_ns"] == timestamp_ns]

    if len(pose_df) == 0:
        raise RuntimeError("Pose was not available for the requested timestamp.")

    city_SE3_ego = convert_pose_dataframe_to_SE3(pose_df)
    return city_SE3_ego

def get_colored_info(data_folder, lidar_timestamp_ns, sweep_lidar):
    lidar_timestamp_ns = int(lidar_timestamp_ns)
    log_id = data_folder.split('/')[-1]
    sweep_rgb: NDArrayByte = np.zeros((sweep_lidar.shape[0], 3), dtype=np.uint8)
    _sdb = SynchronizationDB("./")
    
    # we will fix SynchronizationDB here:
    _sdb.per_log_cam_timestamps_index[log_id] = {}
    for cam_name in list(RingCameras) + list(StereoCameras):
        sensor_folder_wildcard = f"{data_folder}/sensors/cameras/{cam_name}/*.jpg"
        cam_timestamps = get_timestamps_from_sensor_folder(sensor_folder_wildcard)
        _sdb.per_log_cam_timestamps_index[log_id][cam_name] = cam_timestamps
    sensor_folder_wildcard = f"{data_folder}/sensors/lidar/*.feather"
    lidar_timestamps = get_timestamps_from_sensor_folder(sensor_folder_wildcard)
    _sdb.per_log_lidar_timestamps_index[log_id] = lidar_timestamps

    for cam_enum in list(RingCameras):
        cam_name = cam_enum.value
        cam_timestamp_ns = _sdb.get_closest_cam_channel_timestamp(
            lidar_timestamp_ns, cam_name, log_id
        )
        if cam_timestamp_ns is None:
            continue
        img_fpath = Path(f"{DATA_FOLDER}/sensors/cameras/{cam_name}/{cam_timestamp_ns}.jpg")
        if not os.path.exists(img_fpath):
            continue
        cam_timestamp_ns = int(img_fpath.stem)
        
        pinhole_camera = PinholeCamera.from_feather(Path(data_folder), cam_name=cam_name)
        city_SE3_ego_cam_t = get_city_SE3_ego(data_folder=Path(data_folder), timestamp_ns=cam_timestamp_ns)
        city_SE3_ego_lidar_t = get_city_SE3_ego(data_folder=Path(data_folder), timestamp_ns=lidar_timestamp_ns)

        uv, points_cam, is_valid = pinhole_camera.project_ego_to_img_motion_compensated(
            points_lidar_time=data_xyz,
            city_SE3_ego_cam_t=city_SE3_ego_cam_t,
            city_SE3_ego_lidar_t=city_SE3_ego_lidar_t,
        )
        uv_valid = np.round(uv[is_valid]).astype(np.int64)
        u = uv_valid[:, 0]
        v = uv_valid[:, 1]
        img = io_utils.read_img(img_fpath, channel_order="RGB")
        sweep_rgb[is_valid] = img[v, u]
    return sweep_rgb

if __name__ == "__main__":
    center_scenes_xyz = np.array([0,0,0])
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
            center_scenes_xyz = np.array([pose.translation[0], pose.translation[1], pose.translation[2]])
            pose2origin.translation = np.array([0,0,0])
        else:
            pose2origin.translation = pose.translation - center_scenes_xyz

        # extract point cloud data with intensity
        sweep_df = io_utils.read_feather(abs_path_pts_file)
        data_xyz: NDArrayFloat = sweep_df[["x", "y", "z"]].to_numpy().astype(np.float64)
        data_color: NDArrayByte = get_colored_info(DATA_FOLDER, frame_index, data_xyz)

        # detail view you can know the data frame now is based on ego
        ego2up_lidar.rotation = np.eye(3)
        data = ego2up_lidar.inverse().transform_point_cloud(data_xyz[:, :3])

        # NOTE: We transform the point cloud to the world frame based on pose
        data = pose2origin.transform_point_cloud(data[:,:3])
        
        # data = np.hstack((data, data_xyzi[:, 3].reshape(-1, 1)))

        # need qw, qx, qy, qz to save pcd in VIEWPOINT
        qxyzw = R.from_matrix(pose2origin.rotation).as_quat()  # quat
        pose_array = [pose2origin.translation[0], pose2origin.translation[1], pose2origin.translation[2], \
                    qxyzw[3], qxyzw[0], qxyzw[1], qxyzw[2]]
        save_pcd(f"{SAVE_FOLDER}/{i:06d}.pcd", data, pose_array, rgb=data_color)
        # break
    print(f"{bc.OKGREEN}Done{bc.ENDC} Check: {SAVE_FOLDER}..")