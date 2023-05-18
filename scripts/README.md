Evaluation/Create Data
---

- Create the data you need for the unified benchmark. But if you are using the dataset we provide, you don't need to read this part. Or maybe you want to create your own dataset, you can read this part for guidance.
- Evaluate the performance of the methods. 
- Compare the result and output the visualization like the paper.

It's better to view this `md` file through outline.

## Evaluation

All the methods will output the **clean map**, so we need to extract the ground truth label from gt label based on clean map. Why we need this? Since maybe some methods will downsample in their pipeline, so we need to extract the gt label from the downsampled map.

### 0. Create the eval data
```bash
# Check export_eval_pcd.cpp
./export_eval_pcd [folder that you have the output pcd] [method_name_output.pcd] [min_dis to view as the same point]

# example:
./export_eval_pcd /home/kin/bags/VLP16_cone_two_people octomapfg_output.pcd 0.05
```

### 1. Print the score
Check the script and the only thing you need do is change the folder path to your data folder. Or downloaded the result data to test if you want.
```bash
python3 evaluate_benchmark_all_pts.py
```

Or you can view the [eval_ipynb book](py/eval/eval_benchmark.ipynb)

### Optional: Create the visualization

Make sure you have the `gt_cloud.pcd`, `method_output.pcd` and `method_output_exportGT.pcd` which produced by above part.

Since you may wonder what's mistake happened in each methods, label with different intensity to view in CloudCompare. Check `vis_miss_pt.py`. 

```bash
python3 vis_miss_pt.py
```

## Data Creation

Create the data format nicely, for easily benchmarking, no need for downloading the dataset from our link, but maybe benchmark on your custom dataset.

For [cpp](../src) files. Please build the tools first, Dependence is glog, gflags, pcl.

```
cd benchmarks
mkdir build && cd build
cmake .. && make
```

### Open-source Dataset

This part is for open-source dataset, include SemanticKITTI and Argoverse2.0.

#### KITTI / SemanticKITTI

##### A.1 GetData extract_semkitti.py

extract the semantic-kitti dataset from the raw data

Need build `src/extract_semkitti.cpp` also, you will get an executable file `extract_semkitti` in `build` folder.

Then run the script:
```
python3 benchmarks/scripts/extract_semkitti.py
```
Note!! 

1. our pose in kitti is different compared with semantickitti gt one, we maintain the same with ERASOR did for easily benchmark for them.
2. The ego vehicle point is labeled as 0 which means unlabeled in the semanticKITTI dataset. But based on the dis and unlabeled we will set the ego vehicle point as dynamic since it should view as dynamic object.
3. The export pcd files are transformed to world frame. And you can get the sensor pose in the PCD VIEWPOINT Field, so you don't need pose file etc.


##### A.2 GetData export_gt.cpp

You need run the previous one to get exported pcd with pose and transformed.

This one is for export the ground truth map correctly without modified them like force remove the pts around ego vehicle.

```
./extract_gtcloud /home/kin/workspace/DUFOMap/data/KITTI_00
```
Folder should be like this:
```
➜  /home/kin/workspace/DUFOMap/data/KITTI_00 tree -L 2
.
└── pcd
    ├── 004390.pcd
    ├── 004391.pcd
    ├── 004392.pcd
    ├── ...
```
And you will get
```bash
➜  /home/kin/workspace/DUFOMap/data/KITTI_00 tree -L 2
.
├── gt_cloud_intensity.pcd
├── gt_cloud.pcd
```

- `_intensity` seems no modify on intensity, keep the semanticKITTI format which higher 8 is instance id and lower 8 is semantic id.
- without one `gt_cloud.pcd` is modified intensity as 1 for dynamic and 0 for static.


##### A.3 GetData export_rawmap.cpp

No need for octomap/dufomap but for other methods. Since they need a prior map to run.

```bash
./create_rawmap /home/kin/workspace/DUFOMap/data/KITTI_00/pcd
```
And you will get
```bash
I20230404 22:59:45.916883 2025988 create_rawmap.cpp:81] Saved raw map... there are 11771122 points to /home/kin/workspace/DUFOMap/data/KITTI_01/raw_map.pcd
```

#### Argoverse2.0

##### B.1 Download the argoverse dataset

Check this issue: https://github.com/argoverse/av2-api/issues/161

Installing s5cmd

```bash
#!/usr/bin/env bash

export INSTALL_DIR=$HOME/.local/bin
export PATH=$PATH:$INSTALL_DIR
export S5CMD_URI=https://github.com/peak/s5cmd/releases/download/v1.4.0/s5cmd_1.4.0_$(uname | sed 's/Darwin/macOS/g')-64bit.tar.gz

mkdir -p $INSTALL_DIR
curl -sL $S5CMD_URI | tar -C $INSTALL_DIR -xvzf - s5cmd
```

Download the val dataset since train is toooo big for me, totally is 5T for train dataset although no label.

```bash
s5cmd --no-sign-request cp 's3://argoai-argoverse/av2/lidar/val/*' /home/kin/bags/av2/val
```

##### B.2 Run extract_argoverse2.py

This time no need cpp file since argoverse have their own api things and we just need to use it. Also I write with save pcd in utils.

Check their [python api](https://pypi.org/project/av2/), [github](https://github.com/argoverse/av2-api)
```bash
pip install av2
```

But please check the folder path inside the script.
```bash
python3 benchmarks/scripts/extract_argoverse2.py
```

##### B.3 GetData export_rawmap.cpp

No need for octomap/dufomap but for other methods. Since they need a prior map to run.

```bash
./create_rawmap /home/kin/workspace/DUFOMap/data/KITTI_00/pcd
```
And you will get
```bash
I20230404 22:59:45.916883 2025988 create_rawmap.cpp:81] Saved raw map... there are 11771122 points to /home/kin/workspace/DUFOMap/data/KITTI_01/raw_map.pcd
```

### Custom Dataset

For our custom dataset, we normally record the pointcloud with rosbag, and then running some slam methods to get the pose. If you don't have clue to use the slam package, check [simple_ndt_slam](https://github.com/Kin-Zhang/simple_ndt_slam) repo the only dependence you need in the repo is ROS. If you don't have ROS/Ubuntu, you can directly use the `docker` to run.

Then, directly export rosbag file [which have pose/tf and pointcloud topic] to pcd we want

Here is the rosbag to pcd folder script: note here tf is tf2 topic, check more in `simple_ndt_slam` repo.

```
/home/kin/workspace/mapping_ws/src/simple_ndt_slam/tools/build/bag2pcd_tf /home/kin/Downloads/dynablox/hauptgebaeude/sequence_1/2020-02-20-11-58-45.bag /home/kin/DOALS /os1_cloud_node/points 1
```

