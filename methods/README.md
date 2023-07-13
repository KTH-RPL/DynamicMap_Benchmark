Methods in Benchmark
---

Demo Image, results you can have after reading this README:

<img src="../assets/imgs/octomap_gf_00.png" style="width: 70%;" alt="Octomap Image">

All of them have same dependencies [PCL, Glog, yaml-cpp], we will show how to install and build:
## Install & Build

Test computer and System:

- Desktop setting: i9-12900KF, 64GB RAM, Swap 90GB, 1TB SSD
- System setting: Ubuntu 20.04
- Test Date: 2023/04/06
- Modified Version commit: https://github.com/Kin-Zhang/octomap/tree/92a9f14cafa5f567c47b58cbe91137c9a6ba1b0c

Dependencies:
### PCL
Normally, you will directly have PCL library if you installed ROS-full in your computer. 

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