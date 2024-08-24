# Introduction

**Welcome to the Dynamic Map Benchmark Wiki Page!**

You can always press `F` or top right search bar to search for specific topics. 

Please give us [a star](https://github.com/KTH-RPL/DynamicMap_Benchmark)🌟 and [cite our work](#cite-our-papers)📖 to support our work if you find this useful.

<details markdown>
  <summary>CHANGELOG:</summary>

- **2024/08/24**: Reorganize the wiki page. Add all scripts usage in the benchmark code, [Data Overview](data/index.md) with visualization, [Method Overview](method/index.md) with a demo to output clean map and [Evaluation](evaluation/index.md) to evaluate the performance of the methods and also automatically output the visualization in the paper.
- 2024/06/25: [Qingwen](https://kin-zhang.github.io/) is starting to work on the wiki page.  
- **2024/04/29** [BeautyMap](https://arxiv.org/abs/2405.07283) is accepted by RA-L'24. Updated benchmark: BeautyMap and DeFlow submodule instruction in the benchmark. Added the first data-driven method [DeFlow](https://github.com/KTH-RPL/DeFlow/tree/feature/dynamicmap) into our benchmark. Feel free to check.
- **2024/04/18** [DUFOMap](https://arxiv.org/abs/2403.01449) is accepted by RA-L'24. Updated benchmark: DUFOMap and dynablox submodule instruction in the benchmark. Two datasets w/o gt for demo are added in the download link. Feel free to check.
- 2024/03/08 **Fix statements** on our ITSC'23 paper: KITTI sequences pose are also from SemanticKITTI which used SuMa. In the DUFOMap paper Section V-C, Table III, we present the dynamic removal result on different pose sources. Check discussion in [DUFOMap](https://arxiv.org/abs/2403.01449) paper if you are interested.
- 2023/06/13 The [benchmark paper](https://arxiv.org/abs/2307.07260) Accepted by ITSC 2023 and release five methods (Octomap, Octomap w GF, ERASOR, Removert) and three datasets (01, 05, av2, semindoor) in [benchmark paper](https://arxiv.org/abs/2307.07260).


</details>

## Overview
Task: Detect and Remove dynamic points from Point Cloud Maps.

![](https://github.com/KTH-RPL/DynamicMap_Benchmark/blob/main/assets/imgs/background.png?raw=true)

Here is a figure that illustrate of ghost points resulting from dynamic objects in KITTI sequence 7. 
The yellow points to the right represent points labeled as belonging to dynamic objects in the dataset. 
These ghost points negatively affect downstream tasks and overall point cloud quality


## 🏘️ Good to start from here

* What kind of data format we use? 
    
    PCD files (pose information saved in `VIEWPOINT` header). Read [Data Section](data/index.md)

* How to evaluate the performance of a method?

    Two python scripts. Read [Evaluation Section](evaluation/index.md)

* How to run a benchmark method on my data?

    Format your data and run the method. Read [Create data](data/creation.md) and [Run method](method/index.md)

## 🎁 Methods we included

Online (w/o prior map):

- [x] DUFOMap (Ours 🚀): [RAL'24](https://arxiv.org/abs/2403.01449), [**Benchmark Instruction**](https://github.com/KTH-RPL/dufomap)
- [x] Octomap w GF (Ours 🚀): [ITSC'23](https://arxiv.org/abs/2307.07260), [**Benchmark improvement ITSC 2023**](https://github.com/Kin-Zhang/octomap/tree/feat/benchmark)
- [x] dynablox: [RAL'23 official link](https://github.com/ethz-asl/dynablox), [**Benchmark Adaptation**](https://github.com/Kin-Zhang/dynablox/tree/feature/benchmark) 
- [x] Octomap: [ICRA'10 & AR'13 official link](https://github.com/OctoMap/octomap_mapping), [**Benchmark implementation**](https://github.com/Kin-Zhang/octomap/tree/feat/benchmark)

Learning-based (data-driven) (w pretrain-weights provided):

- [x] DeFlow (Ours 🚀): [ICRA'24](https://arxiv.org/abs/2401.16122), [**Benchmark Adaptation**](https://github.com/KTH-RPL/DeFlow/tree/feature/dynamicmap)

Offline (need prior map).

- [x] BeautyMap (Ours 🚀): [RAL'24](https://arxiv.org/abs/2405.07283), [**Official Code**](https://github.com/MKJia/BeautyMap)
- [x] ERASOR: [RAL'21 official link](https://github.com/LimHyungTae/ERASOR), [**benchmark implementation**](https://github.com/Kin-Zhang/ERASOR/tree/feat/no_ros)
- [x] Removert: [IROS 2020 official link](https://github.com/irapkaist/removert), [**benchmark implementation**](https://github.com/Kin-Zhang/removert)

Please note that we provided the comparison methods also but modified a little bit for us to run the experiments quickly, but no modified on their methods' core. Please check the LICENSE of each method in their official link before using it.

You will find all methods in this benchmark under `methods` folder. So that you can easily reproduce the experiments. [Or click here to check our score screenshot directly](assets/imgs/eval_demo.png). 
<!-- And we will also directly provide [the result data](TODO) so that you don't need to run the experiments by yourself. ... Where to save this?  -->

Last but not least, **feel free to pull request if you want to add more methods**. Welcome!

## 💖 Acknowledgements

This benchmark implementation is based on codes from several repositories as we mentioned in the beginning. Thanks for these authors who kindly open-sourcing their work to the community. Please see our paper reference section to get more information.

Thanks to HKUST Ramlab's members: Bowen Yang, Lu Gan, Mingkai Tang, and Yingbing Chen, who help collect additional datasets.

This work was partially supported by the Wallenberg AI, Autonomous Systems and Software Program ([WASP](https://wasp-sweden.org/)) funded by the Knut and Alice Wallenberg Foundation

### Cite Our Papers

Please cite our works if you find these useful for your research:

```
@inproceedings{zhang2023benchmark,
  author={Zhang, Qingwen and Duberg, Daniel and Geng, Ruoyu and Jia, Mingkai and Wang, Lujia and Jensfelt, Patric},
  booktitle={IEEE 26th International Conference on Intelligent Transportation Systems (ITSC)}, 
  title={A Dynamic Points Removal Benchmark in Point Cloud Maps}, 
  year={2023},
  pages={608-614},
  doi={10.1109/ITSC57777.2023.10422094}
}
@article{daniel2024dufomap,
  author={Duberg, Daniel and Zhang, Qingwen and Jia, Mingkai and Jensfelt, Patric},
  journal={IEEE Robotics and Automation Letters}, 
  title={{DUFOMap}: Efficient Dynamic Awareness Mapping}, 
  year={2024},
  volume={9},
  number={6},
  pages={5038-5045},
  doi={10.1109/LRA.2024.3387658}
}
@article{jia2024beautymap,
  author={Jia, Mingkai and Zhang, Qingwen and Yang, Bowen and Wu, Jin and Liu, Ming and Jensfelt, Patric},
  journal={IEEE Robotics and Automation Letters}, 
  title={{BeautyMap}: Binary-Encoded Adaptable Ground Matrix for Dynamic Points Removal in Global Maps}, 
  year={2024},
  volume={9},
  number={7},
  pages={6256-6263},
  doi={10.1109/LRA.2024.3402625}
}
```