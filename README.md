Benchmark of Removed Dynamic in the Point Cloud Map
---

[![arXiv](https://img.shields.io/badge/arXiv-2307.07260-b31b1b.svg)](https://arxiv.org/abs/2307.07260) Will show up in ITSC 2023, Spain.

Here is preview on readme in codes. I'm trying my best on updating all codes and datasets.

Task detect dynamic points in maps and remove them, enhancing the maps:

<center>
<img src="assets/imgs/background.png" width="80%">
</center>

**Folder** quick view:

- `methods` : contains all the methods in the benchmark
- `scripts/py/eval`: eval the result pcd compared with ground truth, get quantitative table
- `scripts/py/data` : pre-process data before benchmark. We also directly provided all the dataset we tested in the map. We run this benchmark offline in computer, so we will extract only pcd files from custom rosbag/other data format [KITTI, Argoverse2]

**Quick** try:

- Teaser data on KITTI sequence 00 only 530Mb, download through [personal One Drive](https://hkustconnect-my.sharepoint.com/:f:/g/personal/qzhangcb_connect_ust_hk/Eki3-eqmhWJDjucGA23TNU4ByF9YuuShic7QZc9aPjmp4w?e=Wd4W0J)
- Go to methods folder, build and run through `./build/${methods_name}_run ${data_path, e.g. /data/00} ${config.yaml} -1 `

## Methods:

Please check in [`methods`](methods) folder.

- [x] ERASOR: [RAL 2021 official link](https://github.com/LimHyungTae/ERASOR), [**benchmark implementation**](https://github.com/Kin-Zhang/ERASOR/tree/feat/no_ros)
- [x] Removert: [IROS 2020 official link](https://github.com/irapkaist/removert), [**benchmark implementation**](TODO)
- [x] Octomap: [ICRA2010 & AR 2013 official link](https://github.com/OctoMap/octomap_mapping), [**benchmark implementation**](https://github.com/Kin-Zhang/octomap/tree/feat/benchmark) Improved this in the paper.
- [ ] DUFOMap: Under review [**official link**](done_but_not_public_yet)
- [ ] [dynablox](https://github.com/ethz-asl/dynablox): [ETH Arxiv official link](https://github.com/ethz-asl/dynablox), [**benchmark implementation**](done_but_not_public_yet)

Please note that we provided the comparison methods also but modified a little bit for us to run the experiments quickly, but no modified on their methods' core. Please check the LICENSE of each method in their official link before using it.

You will find all methods in this benchmark under `methods` folder. So that you can easily reproduce the experiments. And we will also directly provide [the result data](TODO) so that you don't need to run the experiments by yourself.

Last but not least, feel free to pull request if you want to add more methods. Welcome!

## Dataset & Scripts

Download all these dataset from [Zenodo online drive](done but not public yet).

- [x] [Semantic-Kitti, outdoor small town](https://semantic-kitti.org/dataset.html) VLP-64
- [x] [Argoverse2.0, outdoor US cities](https://www.argoverse.org/av2.html#lidar-link) VLP-32
- [ ] [KTH-Campuse] Our [Multi-Campus Dataset](https://mcdviral.github.io/), Collected by Leica RTC360 Total Station.
- [ ] [KTH-Indoor] Our own dataset, Collected by VLP-16/Mid-70 in kobuki.
- [x] [UDI-Plane] Our own dataset, Collected by VLP-16 in a small vehicle.

### Evaluation

First all the methods will output the clean map, so we need to extract the ground truth label from gt label based on clean map. Why we need this? Since maybe some methods downsample in their pipeline, we need to extract the gt label from the downsampled map.

Check [create dataset readme part](scripts/README.md#evaluation) in the scripts folder to get more information. But you can directly download the dataset through the link we provided. Then no need to read the creation; just use the data you downloaded.

- Visualize the result pcd files in [CloudCompare](https://www.danielgm.net/cc/) or the script to provide, one click to get all evaluation benchmarks and comparison images like paper have check in [scripts/py/eval](scripts/py/eval).

- All color bar also provided in CloudCompare, here is [tutorial how we make the animation video](TODO).

## Acknowledgements

This benchmark implementation is based on codes from several repositories as we mentioned in the beginning. Thanks for these authors who kindly open-sourcing their work to the community. Please see our paper reference section to get more information.

### Cite Our Paper

Please cite our work if you find these useful for your research.

Benchmark:

```
@article{zhang2023benchmark,
  author={Qingwen Zhang, Daniel Duberg, Ruoyu Geng, Mingkai Jia, Lujia Wang and Patric Jensfelt},
  title={A Dynamic Points Removal Benchmark in Point Cloud Maps},
  journal={arXiv preprint arXiv:2307.07260},
  year={2023}
}
```

DUFOMap:

```
@article{duberg2023dufomap,
  author={Daniel Duberg*, Qingwen Zhang*, Mingkai Jia and Patric Jensfelt},
  title={{DUFOMap}: TBD}, 
}
```
