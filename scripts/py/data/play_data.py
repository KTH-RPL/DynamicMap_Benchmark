'''
# @date: 2024-07-25 17:29
# @author: Qingwen Zhang (https://kin-zhang.github.io/)
# Copyright (C) 2023-now, RPL, KTH Royal Institute of Technology
# 
# @detail: Play data with interactive viewer.
# 
# This file is part of DynamicMap_Benchmark (https://github.com/KTH-RPL/DynamicMap_Benchmark).
# If you find this repo helpful, please cite the respective publication as 
# listed on the above website.
'''

import sys, os
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..' ))
sys.path.append(BASE_DIR)

import numpy as np
from pathlib import Path
from tqdm import tqdm
import open3d as o3d
import fire, time
from scipy.spatial.transform import Rotation as R

from utils import pcdpy3
from utils.o3d_view import MyVisualizer


def xyzqwxyz_to_matrix(xyzqwxyz: list):
    """
    input: xyzqwxyz: [x, y, z, qx, qy, qz, qw] a list of 7 elements
    """
    rotation = R.from_quat([xyzqwxyz[4], xyzqwxyz[5], xyzqwxyz[6], xyzqwxyz[3]]).as_matrix()
    pose = np.eye(4).astype(np.float64)
    pose[:3, :3] = rotation
    pose[:3, 3] = xyzqwxyz[:3]
    return pose

def inv_pose_matrix(pose):
    inv_pose = np.eye(4)
    inv_pose[:3, :3] = pose[:3, :3].T
    inv_pose[:3, 3] = -pose[:3, :3].T.dot(pose[:3, 3])
    return inv_pose

class DynamicMapData:
    def __init__(self, directory):
        super(DynamicMapData, self).__init__()
        self.scene_id = directory.split("/")[-1]
        self.directory = Path(directory) / "pcd"
        self.pcd_files = [os.path.join(self.directory, f) for f in sorted(os.listdir(self.directory)) if f.endswith('.pcd')]

    def __len__(self):
        return len(self.pcd_files)
    
    def __getitem__(self, index_):
        res_dict = {
            'scene_id': self.scene_id,
            'timestamp': self.pcd_files[index_].split("/")[-1].split(".")[0],
        }
        pcd_ = pcdpy3.PointCloud.from_path(self.pcd_files[index_])
        pcd_.xyzi2np()
        pc0 = pcd_.np_data[:,:3]
        pose0 = xyzqwxyz_to_matrix(list(pcd_.viewpoint))
        inv_pose0 = inv_pose_matrix(pose0)
        res_dict['pc'] = pc0 @ inv_pose0[:3, :3].T + inv_pose0[:3, 3]
        return res_dict

def vis(
    data_dir: str = "/home/kin/DATA_HDD/Dynamic_Papers_assets/Benchmark_data/00",
    view_file: str = os.path.abspath(BASE_DIR+"/../../assets/view/default.json"),
    point_size: int = 3,
    speed: int = 1,
    ):
    o3d_vis = MyVisualizer(view_file=view_file, window_title="DynamicMap Benchmark Data Preview")
    opt = o3d_vis.vis.get_render_option()
    # opt.background_color = np.asarray([216, 216, 216]) / 255.0
    # opt.background_color = np.asarray([80/255, 90/255, 110/255])
    # opt.background_color = np.asarray([1, 1, 1])
    opt.point_size = point_size
    dataset = DynamicMapData(data_dir)
    for data_id in (pbar := tqdm(range(0, len(dataset)))):
        data = dataset[data_id]
        now_scene_id = data['scene_id']
        pbar.set_description(f"id: {data_id}, scene_id: {now_scene_id}, timestamp: {data['timestamp']}")

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(data['pc'][:, :3])
        # print(data['pc'].shape)
        o3d_vis.update([pcd, o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)])
        time.sleep(0.05*1/speed)

if __name__ == "__main__":
    fire.Fire(vis)
    pass