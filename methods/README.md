Methods in Benchmark
---

Demo Image, results you can have after reading this README:

<img src="../assets/imgs/octomap_gf_00.png" style="width: 70%;" alt="Octomap Image">

All of them have same dependencies [PCL, Glog, yaml-cpp], we will show how to install and build:
## Install & Build

Test computer and System:

- Desktop setting: i9-12900KF, 64GB RAM, Swap 90GB, 1TB SSD
- System setting: Ubuntu 20.04 [ROS noetic-full installed in system]

Dependencies listed following if you want to install them manually, or you can use docker to build and run if you don't like trash your env.
### Docker
If you want to use docker, please check [Dockerfile](../Dockerfile) and [docker-compose.yml](../docker-compose.yml) for more details. 
```
cd DynamicMap_Benchmark
docker build -t zhangkin/dynamic_map .
docker run -it --rm --name dynamicmap -v /home/kin/data:/home/kin/data zhangkin/dynamic_map /bin/zsh
```
- `-v` means link your data folder to docker container, so you can use your data in docker container. `-v ${your_env_path}:${container_path}`
- If it's hard to build, you can always use `docker pull zhangkin/dynamic_map` to pull the image from docker hub.
- 
### PCL / OpenCV
Normally, you will directly have PCL and OpenCV library if you installed ROS-full in your computer. 
OpenCV is required by Removert only, PCL is required by Benchmark.

Reminder for ubuntu 20.04 may occur this error:
```bash
fatal error: opencv2/cv.h: No such file or directory
```
ln from opencv4 to opencv2
```bash
sudo ln -s /usr/include/opencv4/opencv2 /usr/include/opencv2
```

### glog gflag [for print]
or you can install through `sudo apt install`
```sh
sh -c "$(wget -O- https://raw.githubusercontent.com/Kin-Zhang/Kin-Zhang/main/Dockerfiles/latest_glog_gflag.sh)"
```

### yaml-cpp [for config]
Please set the FLAG, check this issue if you want to know more: https://github.com/jbeder/yaml-cpp/issues/682, [TOOD inside the CMakeLists.txt](https://github.com/jbeder/yaml-cpp/issues/566)

If you install in Ubuntu 22.04, please check this commit: https://github.com/jbeder/yaml-cpp/commit/c86a9e424c5ee48e04e0412e9edf44f758e38fb9 which is the version could build in 22.04

```sh
cd ${Tmp_folder}
git clone https://github.com/jbeder/yaml-cpp.git && cd yaml-cpp
env CFLAGS='-fPIC' CXXFLAGS='-fPIC' cmake -Bbuild
cmake --build build --config Release
sudo cmake --build build --config Release --target install
```

### Build

```bash
cd ${methods you want}
cmake -B build && cmake --build build
```

## RUN

Check each methods config file in their own folder `config/*.yaml` or `assets/*.yaml`
```bash
./build/${methods_name}_run ${data_path} ${config.yaml} -1
```

For example, if you want to run octomap with GF

```bash
./build/octomap_run /home/kin/data/00 assets/config_fg.yaml -1
```

Then you can get a time table with clean map result from Octomap w GF like top image shows. Or ERASOR on semindoor dataset:

```bash
./build/erasor_run /home/kin/data/semindoor config/seq_semindoor.yaml -1
```

![ERASOR Image](../assets/imgs/erasor_semindoor.png)

All Running commands, `-1` means all pcd files in the `pcd` folder, if you only want to run `10` frames change to `10`.

```
./build/removert_run ${data_path} ${config.yaml} -1
./build/octomap_run ${data_path} ${config.yaml} -1
./build/erasor_run ${data_path} ${config.yaml} -1
```

Then maybe you would like to have quantitative and qualitative result, check [scripts/py/eval](../scripts/py/eval).

## Quick view on results

Both with quantitative table and qualitative result

TODO