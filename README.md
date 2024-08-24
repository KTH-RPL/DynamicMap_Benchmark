A Dynamic Points Removal Benchmark in Point Cloud Maps
---

Author: [Qingwen Zhang](http://kin-zhang.github.io) (Kin)

This is **our wiki page README**, please visit our [main branch](https://github.com/KTH-RPL/DynamicMap_Benchmark) for more information about the benchmark.

## Install

If you want to try the MkDocs locally, the only thing you need is `Python` and some python package. If you are worrying it will destory your env, you can try [virual env](https://docs.python.org/3/library/venv.html) or [anaconda](https://www.anaconda.com/).


main package [user is only need for sometime, check the issue section]
```bash
pip install mkdocs-material
```

plugin package
```bash
pip install mkdocs-minify-plugin mkdocs-git-revision-date-localized-plugin mkdocs-git-authors-plugin mkdocs-video
```

### Run
```bash
mkdocs serve
```

## Acknowledgements

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
@article{jia2024beautymap,
  author={Jia, Mingkai and Zhang, Qingwen and Yang, Bowen and Wu, Jin and Liu, Ming and Jensfelt, Patric},
  journal={IEEE Robotics and Automation Letters}, 
  title={BeautyMap: Binary-Encoded Adaptable Ground Matrix for Dynamic Points Removal in Global Maps}, 
  year={2024},
  volume={},
  number={},
  pages={1-8},
  doi={10.1109/LRA.2024.3402625}
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
```
