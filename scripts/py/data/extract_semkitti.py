'''
# @created: 2023-04-04 21:44
# @updated: 2024-03-27 10:12
# @author: Qingwen Zhang  (https://kin-zhang.github.io/)
# 
# Copyright (C) 2023-now, RPL, KTH Royal Institute of Technology
# If you find this repo helpful, please cite the respective publication in DynamicBenchmark.
# 
'''

import sys, os
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..' ))
sys.path.append(BASE_DIR)


import numpy as np
from tqdm import tqdm
from scipy.spatial.transform import Rotation as R
from av2.geometry.se3 import SE3
import fire, time

# user defined
from utils import bc, filterOutRange
from utils.semkitti_api import parse_calibration, parse_poses
from utils.pcdpy3 import save_pcd

# full frame range for each sequence
# FIX_SEQ_FRAME_RANGE = {"00": [0, 10000], 
#                     "01": [800, 10000], 
#                     "02": [0, 10000], 
#                     "05": [0, 10000],
#                     "07": [0, 10000]}

# selected frame to release the pressure of the memory
FIX_SEQ_FRAME_RANGE = {"00": [4390, 4530], 
                    # "01": [150, 250], 
                    # "02": [860, 950], 
                    "05": [2350, 2670],
                    "07": [630, 820],
                    "19": [150, 512]}

def main(
    original_path: str = "/home/kin/data/KITTI/SemanticKitti", # original data path when you downloaded the semantic kitti dataset
    sequence: str = "00",
    save_data_folder: str = f"/home/kin/data",
    gt_cloud: bool = False,
):
    SaveDataFolder = f"{save_data_folder}/{sequence}/pcd"
    # get the path of the data
    pose_file = os.path.join(original_path + "/data_odometry_labels/dataset/sequences", sequence, "poses.txt")
    calib_file = os.path.join(original_path + "/data_odometry_calib/dataset/sequences", sequence, "calib.txt")
    pts_folder = os.path.join(original_path + "/data_odometry_velodyne/dataset/sequences", sequence, "velodyne")
    labels_folder = os.path.join(original_path + "/data_odometry_labels/dataset/sequences", sequence, "labels")

    if not (os.path.isfile(pose_file) and os.path.isfile(calib_file) and os.path.isdir(labels_folder)):
        print(f"Changing to another mode.. where the data and label are in the same folder..")
        pose_file = os.path.join(original_path, sequence, "poses.txt")
        calib_file = os.path.join(original_path, sequence, "calib.txt")
        pts_folder = os.path.join(original_path, sequence, "velodyne")
        labels_folder = os.path.join(original_path, sequence, "labels")
        
        if not (os.path.isfile(pose_file) and os.path.isfile(calib_file)):
            print(f"{bc.FAIL}Please check the path of the data: {pts_folder} {bc.ENDC}")
            return

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
    
    if sequence in FIX_SEQ_FRAME_RANGE:
        start, end = FIX_SEQ_FRAME_RANGE[sequence]
        print(f"Extract {sequence} from {start} to {end}")
    else:
        start, end = 0, 10000
    

    pts_files = sorted(os.listdir(pts_folder))
    filtered_pts_files = [f for f in pts_files if start <= int(f.split('.')[0]) <= end]

    ground_truth_pts = np.array([])
    for i, pts_file in tqdm(enumerate(filtered_pts_files), total=len(filtered_pts_files), ncols=80):
        frame_index = pts_file.split('.')[0]
        label_file = frame_index + '.label' # get label
        T = poses_list[int(frame_index)] # get pose
        xyz = T[:, -1]  # tran
        qxyzw = R.from_matrix(T[:3, :3]).as_quat()  # quat

        # setup the file path and name
        pcd_file = f"{pts_folder}/{pts_file}"
        label_file = f"{labels_folder}/{label_file}"
        save_pcd_file = f"{SaveDataFolder}/{frame_index}.pcd"

        labels = np.fromfile(label_file, dtype=np.uint32)
        points = np.fromfile(pcd_file, dtype=np.float32).reshape(-1, 4)

        assert labels.shape[0] == points.shape[0], f"sem_labels.shape[0] != points.shape[0], {bc.FAIL}please check the path{bc.ENDC}"

        # NOTE: We have to filter out the point outside 50m range, since that's the nan label outside the range in semantic kitti
        points, sem_labels = filterOutRange(points, labels, max_range=50.0)

        # NOTE: We transform the point cloud to the world frame based on pose
        pose2world = SE3(T[:3, :3], T[:3, -1])
        points = np.hstack((pose2world.transform_point_cloud(points[:,:3]), points[:, 3].reshape(-1, 1)))

        # // transform intensity to label as uint32_t, check semantic KITTI config file:
        # // https://github.com/PRBonn/semantic-kitti-api/blob/master/config/semantic-kitti-mos.yaml#L33-L41
        intensity = np.zeros_like(sem_labels, dtype=np.float32)
        intensity[np.isin(sem_labels, [252, 253, 254, 255, 256, 257, 258, 259])] = 1

        save_points = np.hstack((points[:, :3], intensity.reshape(-1, 1))) # [x, y, z, intensity]
        save_pcd(save_pcd_file, save_points, np.array([xyz[0], xyz[1],xyz[2], qxyzw[3], qxyzw[0], qxyzw[1], qxyzw[2]]))

        # save a ground truth full map
        if gt_cloud:
            ground_truth_pts = np.vstack((ground_truth_pts, save_points)) if ground_truth_pts.size else save_points

    if gt_cloud:
        save_pcd(f"{SaveDataFolder.split('pcd')[0]}/gt_cloud.pcd", ground_truth_pts)
        print(f"Save ground truth map to {SaveDataFolder.split('pcd')[0]}/gt_cloud.pcd")

if __name__ == "__main__":
    start_time = time.time()
    fire.Fire(main)
    print(f"{os.path.basename( __file__ )}: All codes run {bc.OKGREEN}successfully in {time.time() - start_time:.2f} s, Close now.. {bc.ENDC}")
