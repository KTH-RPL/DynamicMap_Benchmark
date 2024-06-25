'''
# Created: 2023-1-26 16:38
# Updated: 2024-04-15 12:06
# Copyright (C) 2023-now, RPL, KTH Royal Institute of Technology
# Author: Qingwen ZHANG  (https://kin-zhang.github.io/)
# 
# code gits: https://gist.github.com/Kin-Zhang/77e8aa77a998f1a4f7495357843f24ef
# 
# Description as follows:

This file is for open3d view control set from view_file, which should be json
1. use normal way to open any geometry and set view by mouse you want
2. `CTRL+C` it will copy the view detail at this moment.
3. `CTRL+V` to json file, you can create new one
4. give the json file path

Check this part: http://www.open3d.org/docs/release/tutorial/visualization/visualization.html#Store-view-point

Test if you want by run this script: by press 'V' on keyboard, will set from json

# CHANGELOG:
# 2024-04-15 12:06(Qingwen): show a example json text. add hex_to_rgb, color_map_hex, color_map (for color points if needed)
# 2024-01-27 0:41(Qingwen): update MyVisualizer class, reference from kiss-icp 
[python/kiss-icp/tools/visualizer.py](https://github.com/PRBonn/kiss-icp/blob/main/python/kiss_icp/tools/visualizer.py)
'''

import open3d as o3d
import json
import os, sys
from typing import List, Callable
from functools import partial

def hex_to_rgb(hex_color):
    hex_color = hex_color.lstrip("#")
    return tuple(int(hex_color[i:i + 2], 16) / 255.0 for i in (0, 2, 4))

color_map_hex = ['#a6cee3', '#de2d26', '#1f78b4','#b2df8a','#33a02c','#fb9a99','#e31a1c','#fdbf6f','#ff7f00','#cab2d6','#6a3d9a','#ffff99','#b15928',\
                 '#8dd3c7','#ffffb3','#bebada','#fb8072','#80b1d3','#fdb462','#b3de69','#fccde5','#d9d9d9','#bc80bd','#ccebc5','#ffed6f']
color_map = [hex_to_rgb(color) for color in color_map_hex]

class ViewControl:
    def __init__(self, vctrl: o3d.visualization.ViewControl, view_file=None):
        self.vctrl = vctrl
        self.params = None
        if view_file is not None:
            print(f"Init with view_file from: {view_file}")
            self.parse_file(view_file)
            self.set_param()
        else:
            print("Init without view_file")

    def read_viewTfile(self, view_file):
        if view_file is None:
            return
        self.parse_file(view_file)
        self.set_param()

    def save_viewTfile(self, view_file):
        return
    
    def parse_file(self, view_file):
        if view_file is None:
            print(f"\033[91mNo specific view file. Skip to setup viewpoint in open3d. \033[0m")
            return
        if(os.path.exists(view_file)):
            with open((view_file)) as user_file:
                file_contents = user_file.read()
                self.params = json.loads(file_contents)
        else:
            print(f"\033[91mDidn't find the file, please check it again: {view_file} \033[0m")
            print(f"NOTE: If you still have this error, please give the absulote path for view_file")
            sys.exit()

    def set_param(self):
        self.vctrl.change_field_of_view(self.params['trajectory'][0]['field_of_view'])
        self.vctrl.set_front(self.params['trajectory'][0]['front'])
        self.vctrl.set_lookat(self.params['trajectory'][0]['lookat'])
        self.vctrl.set_up(self.params['trajectory'][0]['up'])
        self.vctrl.set_zoom(self.params['trajectory'][0]['zoom'])

class MyVisualizer:
    def __init__(self, view_file=None, window_title="Default"):
        self.params = None
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(window_name=window_title)
        self.o3d_vctrl = ViewControl(self.vis.get_view_control(), view_file=view_file)
        self.view_file = view_file

        self.block_vis = True
        self.play_crun = False
        self.reset_bounding_box = True
        print(
            f"\n{window_title.capitalize()} initialized. Press:\n"
            "\t[SPACE] to pause/start\n"
            "\t  [ESC] to exit\n"
            "\t    [N] to step\n"
        )
        self._register_key_callback(["Ā", "Q", "\x1b"], self._quit)
        self._register_key_callback([" "], self._start_stop)
        self._register_key_callback(["N"], self._next_frame)

    def show(self, assets: List):
        self.vis.clear_geometries()

        for asset in assets:
            self.vis.add_geometry(asset)
            self.o3d_vctrl.read_viewTfile(self.view_file)

        self.vis.update_renderer()
        self.vis.poll_events()
        self.vis.run()
        self.vis.destroy_window()

    def update(self, assets: List, clear: bool = True):
        if clear:
            self.vis.clear_geometries()

        for asset in assets:
            self.vis.add_geometry(asset, reset_bounding_box=False)
            self.vis.update_geometry(asset)

        if self.reset_bounding_box:
            self.vis.reset_view_point(True)
            if self.view_file is not None:
                self.o3d_vctrl.read_viewTfile(self.view_file)
            self.reset_bounding_box = False

        self.vis.update_renderer()
        while self.block_vis:
            self.vis.poll_events()
            if self.play_crun:
                break
        self.block_vis = not self.block_vis

    def _register_key_callback(self, keys: List, callback: Callable):
        for key in keys:
            self.vis.register_key_callback(ord(str(key)), partial(callback))
    def _next_frame(self, vis):
        self.block_vis = not self.block_vis
    def _start_stop(self, vis):
        self.play_crun = not self.play_crun
    def _quit(self, vis):
        print("Destroying Visualizer. Thanks for using ^v^.")
        vis.destroy_window()
        os._exit(0)

if __name__ == "__main__":
    json_content = """{
	"class_name" : "ViewTrajectory",
	"interval" : 29,
	"is_loop" : false,
	"trajectory" : 
	[
		{
			"boundingbox_max" : [ 3.9660897254943848, 2.427476167678833, 2.55859375 ],
			"boundingbox_min" : [ 0.55859375, 0.83203125, 0.56663715839385986 ],
			"field_of_view" : 60.0,
			"front" : [ 0.27236083595988803, -0.25567329763523589, -0.92760484038816615 ],
			"lookat" : [ 2.4114965637897101, 1.8070288935660688, 1.5662280268112718 ],
			"up" : [ -0.072779625398507866, -0.96676294585190281, 0.24509698622097265 ],
			"zoom" : 0.47999999999999976
		}
	],
	"version_major" : 1,
	"version_minor" : 0
}
"""
    # write to json file
    view_json_file = "view.json"
    with open(view_json_file, 'w') as f:
        f.write(json_content)
    sample_ply_data = o3d.data.PLYPointCloud()
    pcd = o3d.io.read_point_cloud(sample_ply_data.path)
    # 1. define
    viz = o3d.visualization.VisualizerWithKeyCallback()
    # 2. create
    viz.create_window(window_name="TEST ON Change View point through JSON, Press V Please")
    # 3. add geometry
    viz.add_geometry(pcd)
    # 4. get control !!! must step by step
    ctr = viz.get_view_control()

    o3d_vctrl = ViewControl(ctr)

    def set_view(viz):
        #Your update routine
        o3d_vctrl.read_viewTfile(view_json_file)
        viz.update_renderer()
        viz.poll_events()
        viz.run()

    viz.register_key_callback(ord('V'), set_view)
    viz.run()
    viz.destroy_window()
    print("\033[92mAll o3d_view codes run successfully, Close now..\033[0m See you!")

    # or:
    # viz = MyVisualizer(view_file, window_title="Check Pose")
    # viz.show([*pcds, *draw_tfs])