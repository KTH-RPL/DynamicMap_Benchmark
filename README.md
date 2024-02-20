A Dynamic Points Removal Benchmark in Point Cloud Maps
---

[![arXiv](https://img.shields.io/badge/arXiv-2307.07260-b31b1b.svg)](https://arxiv.org/abs/2307.07260) 
[![video](https://img.shields.io/badge/video-youtube-09a4d7.svg)](https://youtu.be/pCHsNKXDJQM?si=nhbAnPrbaZJEqbjx)

Here is preview on readme in codes. Task detect dynamic points in maps and remove them, enhancing the maps:

<center>
<img src="assets/imgs/background.png" width="80%">
</center>

**Folder** quick view:

- `methods` : contains all the methods in the benchmark
- `scripts/py/eval`: eval the result pcd compared with ground truth, get quantitative table
- `scripts/py/data` : pre-process data before benchmark. We also directly provided all the dataset we tested in the map. We run this benchmark offline in computer, so we will extract only pcd files from custom rosbag/other data format [KITTI, Argoverse2]

**Quick** try:

- Teaser data on KITTI sequence 00 only 384.8MB in [Zenodo online drive](https://zenodo.org/record/8160051)
- Clone our repo:
  ```bash
  git clone --recurse-submodules https://github.com/KTH-RPL/DynamicMap_Benchmark.git
  ```
- Go to methods folder, build and run through 
  ```bash
  cd methods/${methods_name} && cmake -B build && cmake --build build
  ./build/${methods_name}_run ${data_path, e.g. /data/00} ${config.yaml} -1 
  ```

## Methods:

Please check in [`methods`](methods) folder.

- [x] ERASOR: [RAL 2021 official link](https://github.com/LimHyungTae/ERASOR), [**benchmark implementation**](https://github.com/Kin-Zhang/ERASOR/tree/feat/no_ros)
- [x] Removert: [IROS 2020 official link](https://github.com/irapkaist/removert), [**benchmark implementation**](TODO)
- [x] Octomap w GF: [**Benchmark improvement ITSC 2023**](https://github.com/Kin-Zhang/octomap/tree/feat/benchmark) origin mapping from [ICRA2010 & AR 2013 official link](https://github.com/OctoMap/octomap_mapping)
- [ ] DUFOMap: Under review [**official link**](https://github.com/KTH-RPL/dufomap)
- [ ] dynablox: [RAL 2023 official link](https://github.com/ethz-asl/dynablox), [**Benchmark Adaptation**](done_but_not_public_yet)

Please note that we provided the comparison methods also but modified a little bit for us to run the experiments quickly, but no modified on their methods' core. Please check the LICENSE of each method in their official link before using it.

You will find all methods in this benchmark under `methods` folder. So that you can easily reproduce the experiments. And we will also directly provide [the result data](TODO) so that you don't need to run the experiments by yourself.

Last but not least, feel free to pull request if you want to add more methods. Welcome!

## Dataset & Scripts

Download all these dataset from [Zenodo online drive](https://zenodo.org/record/8160051). Or create by yourself through the [scripts we provided](scripts/README.md).

- [x] [Semantic-Kitti, outdoor small town](https://semantic-kitti.org/dataset.html) VLP-64
- [x] [Argoverse2.0, outdoor US cities](https://www.argoverse.org/av2.html#lidar-link) VLP-32
- [x] [UDI-Plane] Our own dataset, Collected by VLP-16 in a small vehicle.
- [ ] [KTH-Campuse] Our [Multi-Campus Dataset](https://mcdviral.github.io/), Collected by [Leica RTC360 3D Laser Scan](https://leica-geosystems.com/products/laser-scanners/scanners/leica-rtc360).
- [ ] [HKUST-Building] Our [fusionportable Dataset](https://fusionportable.github.io/dataset/fusionportable/), collected by [Leica BLK360 Imaging Laser Scanner](https://leica-geosystems.com/products/laser-scanners/scanners/blk360)
- [ ] [KTH-Indoor] Our own dataset, Collected by VLP-16/Mid-70 in kobuki.

Welcome to contribute your dataset with ground truth to the community through pull request.

### Evaluation

First all the methods will output the clean map, if you are only **user on map clean task,** it's **enough**. But for evaluation, we need to extract the ground truth label from gt label based on clean map. Why we need this? Since maybe some methods downsample in their pipeline, we need to extract the gt label from the downsampled map.

Check [create dataset readme part](scripts/README.md#evaluation) in the scripts folder to get more information. But you can directly download the dataset through the link we provided. Then no need to read the creation; just use the data you downloaded.

- Visualize the result pcd files in [CloudCompare](https://www.danielgm.net/cc/) or the script to provide, one click to get all evaluation benchmarks and comparison images like paper have check in [scripts/py/eval](scripts/py/eval).

- All color bar also provided in CloudCompare, here is [tutorial how we make the animation video](TODO).

## Acknowledgements

This benchmark implementation is based on codes from several repositories as we mentioned in the beginning. Thanks for these authors who kindly open-sourcing their work to the community. Please see our paper reference section to get more information.

This work was partially supported by the Wallenberg AI, Autonomous Systems and Software Program ([WASP](https://wasp-sweden.org/)) funded by the Knut and Alice Wallenberg Foundation
### Cite Our Paper

Please cite our work if you find these useful for your research.

Benchmark:

```
@inproceedings{zhang2023benchmark,
  author={Zhang, Qingwen and Duberg, Daniel and Geng, Ruoyu and Jia, Mingkai and Wang, Lujia and Jensfelt, Patric},
  booktitle={2023 IEEE 26th International Conference on Intelligent Transportation Systems (ITSC)}, 
  title={A Dynamic Points Removal Benchmark in Point Cloud Maps}, 
  year={2023},
  volume={},
  number={},
  pages={608-614},
  keywords={Point cloud compression;Measurement;Codes;Heuristic algorithms;Benchmark testing;Task analysis;Tuning},
  doi={10.1109/ITSC57777.2023.10422094}}
```

DUFOMap:

```
@article{duberg2023dufomap,
  author={Duberg, Daniel and Zhang, Qingwen and Jia, Mingkai and Jensfelt, Patric},
  title={DUFOMap: DUFOMap: Efficient Dynamic Awareness Mapping}, 
  journal={TB public},
}
```
