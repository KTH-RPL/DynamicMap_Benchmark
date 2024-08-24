# Data Description

In this section, we will introduce the data format we use in the benchmark, and how to visualize the data easily.
Next section on creation will show you how to create this format data from your own data.

## Benchmark Unified Format

We saved all our data into **PCD files**, first let me introduce the [PCD file format](https://pointclouds.org/documentation/tutorials/pcd_file_format.html):

The important two for us are `VIEWPOINT`, `POINTS` and `DATA`:

- **VIEWPOINT** - specifies an acquisition viewpoint for the points in the dataset. This could potentially be later on used for building transforms between different coordinate systems, or for aiding with features such as surface normals, that need a consistent orientation.

    The viewpoint information is specified as a translation (tx ty tz) + quaternion (qw qx qy qz). The default value is:

    ```bash
    VIEWPOINT 0 0 0 1 0 0 0
    ```

- **POINTS** - specifies the number of points in the dataset.

- **DATA** - specifies the data type that the point cloud data is stored in. As of version 0.7, three data types are supported: ascii, binary, and binary_compressed. We saved as binary for faster reading and writing.

### A Header Example

I directly show a example header here from `004390.pcd` in KITTI sequence 00:

```
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH 125883
HEIGHT 1
VIEWPOINT -15.6504 17.981 -0.934952 0.882959 -0.0239536 -0.0058903 -0.468802
POINTS 125883
DATA binary
```

In this `004390.pcd` we have 125883 points, and the pose (sensor center) of this frame is: `-15.6504 17.981 -0.934952 0.882959 -0.0239536 -0.0058903 -0.468802`. 

Again, all points from data frames are ==already transformed to the world frame== and VIEWPOINT is the sensor pose.

### How to read PCD files

In C++, we usually use PCL library to read PCD files, here is a simple example:

```cpp
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

pcl::PointCloud<pcl::PointXYZI>::Ptr pcd(new pcl::PointCloud<pcl::PointXYZI>);
pcl::io::loadPCDFile<pcl::PointXYZI>("data/00/004390.pcd", *pcd);
```

In Python, we have a simple script to read PCD files in [the benchmark code](https://github.com/KTH-RPL/DynamicMap_Benchmark/blob/main/scripts/py/utils/pcdpy3.py), or from [my gits](https://gist.github.com/Kin-Zhang/bd6475bdfa0ebde56ab5c060054d5185), you don't need to read the script in detail but use it directly.

```python
import pcdpy3 # the script I provided
pcd_data = pcdpy3.PointCloud.from_path('data/00/004390.pcd')
pc = pcd_data.np_data[:,:3] # shape (N, 3) N: the number of point, 3: x y z
# if the header have intensity or rgb field, you can get it by:
# pc_intensity = pcd_data.np_data[:,3] # shape (N,)
# pc_rgb = pcd_data.np_data[:,3:6] # shape (N, 3)
```

## Download benchmark data

We already processed the data in the benchmark, you can download the data from the [following links](https://zenodo.org/records/10886629):


| Dataset | Description | Sensor Type | Total Frame Number | Size |
| --- | --- | --- | --- | --- |
| KITTI sequence 00 | in a small town with few dynamics (including one pedestrian around) | VLP-64 | 141 | 384.8 MB |
| KITTI sequence 05 | in a small town straight way, one higher car, the benchmarking paper cover image from this sequence. | VLP-64 | 321 | 864.0 MB |
| Argoverse2 | in a big city, crowded and tall buildings (including cyclists, vehicles, people walking near the building etc. | 2 x VLP-32 | 575 | 1.3 GB |
| KTH campus (no gt) | Collected by us (Thien-Minh) on the KTH campus. Lots of people move around on the campus. | Leica RTC360 | 18 | 256.4 MB |
| Semi-indoor | Collected by us, running on a small 1x2 vehicle with two people walking around the platform. | VLP-16 | 960 | 620.8 MB |
| Twofloor (no gt) | Collected by us (Bowen Yang) in a quadruped robot. A two-floor structure environment with one pedestrian around. | Livox-mid 360 | 3305 | 725.1 MB |

Download command:
```bash
wget https://zenodo.org/api/records/10886629/files-archive.zip

# or download each sequence separately
wget https://zenodo.org/records/10886629/files/00.zip
wget https://zenodo.org/records/10886629/files/05.zip
wget https://zenodo.org/records/10886629/files/av2.zip
wget https://zenodo.org/records/10886629/files/kthcampus.zip
wget https://zenodo.org/records/10886629/files/semindoor.zip
wget https://zenodo.org/records/10886629/files/twofloor.zip
```

## Visualize the data

We provide a simple script to visualize the data in the benchmark, you can find it in [scripts/py/data/play_data.py](https://github.com/KTH-RPL/DynamicMap_Benchmark/blob/main/scripts/py/data/play_data.py). You may want to download the data and requirements first.

```bash
cd scripts/py

# download the data
wget https://zenodo.org/records/10886629/files/twofloor.zip

# https://github.com/KTH-RPL/DynamicMap_Benchmark/blob/main/scripts/py/requirements.txt
pip install -r requirements.txt
```

Run it:
```bash
python data/play_data.py --data_folder /home/kin/data/twofloor --speed 1 # speed 1 for normal speed, 2 for faster with 2x speed
```

It will pop up a window to show the point cloud data, you can use the mouse to rotate, zoom in/out, and move the view. Terminal show the help information to start/stop the play.

<center>
![type:video](https://github.com/user-attachments/assets/158040bd-02ab-4fd4-ab93-2dcacabf342a)
</center>

The axis here shows the sensor frame. The video is play in sensor-frame, so you can see the sensor move around in the video.

