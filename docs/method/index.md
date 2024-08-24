# Methods

In this section we will introduce how to run the methods in the benchmark.

Here is a demo result you can have after reading this README:

<img src="https://github.com/KTH-RPL/DynamicMap_Benchmark/blob/main/assets/imgs/octomap_gf_00.png?raw=true" style="width: 100%;" alt="Octomap Image">

## Install & Build

Test computer and System:

- Desktop setting: i9-12900KF, 64GB RAM, Swap 90GB, 1TB SSD
- System setting: Ubuntu 20.04 [ROS noetic-full installed in system]

### Setup

We show the dependencies for [our octomap](https://github.com/Kin-Zhang/octomap) as an example.

```bash
sudo apt update && sudo apt install -y libpcl-dev 
sudo apt install -y libgoogle-glog-dev libgflags-dev
```

#### Docker option

You can use docker to build and run if you don't like trash your env and is able to run all methods in our benchmark.

If you want to use docker, please check [Dockerfile](https://github.com/KTH-RPL/DynamicMap_Benchmark/blob/main/Dockerfile) for more details.  This can also be a reference for you to install the dependencies.
```
cd DynamicMap_Benchmark
docker build -t zhangkin/dynamic_map .
docker run -it --rm --name dynamicmap -v /home/kin/data:/home/kin/data zhangkin/dynamic_map /bin/zsh
```
- `-v` means link your data folder to docker container, so you can use your data in docker container. `-v ${your_env_path}:${container_path}`
- If it's hard to build, you can always use `docker pull zhangkin/dynamic_map` to pull the image from docker hub.

### Build

```bash
git clone https://github.com/Kin-Zhang/octomap
cd octomap
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

Then you can get a time table with clean map result from Octomap w GF like top image shows. You can also output the voxel map by changing the config file.

![](https://raw.githubusercontent.com/Kin-Zhang/octomap/feat/benchmark/assets/imgs/demo.png)


Other methods like DUFOMap (by cd into the benchmark submodule or clone alone), you can run like this:

```bash
git clone https://github.com/Kin-Zhang/dufomap.git
cmake -B build -D CMAKE_CXX_COMPILER=g++-10 && cmake --build build
./build/dufomap_run /home/kin/data/semindoor assets/config.toml
```

![](https://raw.githubusercontent.com/KTH-RPL/dufomap/main/assets/demo.png)

All Running commands, `-1` means all pcd files in the `pcd` folder, if you only want to run `10` frames change to `10`.

```bash
./build/octomap_run ${data_path} ${config.yaml} -1
./build/dufomap_run ${data_path} ${config.toml}

# beautymap
python main.py --data_dir data/00 --dis_range 40 --xy_resolution 1 --h_res 0.5

# deflow
python main.py checkpoint=/home/kin/deflow_best.ckpt dataset_path=/home/kin/data/00
```

Then maybe you would like to have quantitative and qualitative result, check [evaluation](../evaluation/index.md) part.