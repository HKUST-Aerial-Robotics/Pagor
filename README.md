<div align="center">
    <img src="docs/pagor.png" alt="pagor">
</div>
<div align="center">
<a href="https://arxiv.org/abs/2307.12116"><img src="https://img.shields.io/badge/arXiv-2307.12116-004088.svg"/></a>
<a href="https://www.youtube.com/watch?v=BPqbjhZ0FWQ&t=1s">
<img alt="Youtube" src="https://img.shields.io/badge/Video-Youtube-red"/>
</a>
<a ><img alt="PRs-Welcome" src="https://img.shields.io/badge/PRs-Welcome-red" /></a>
<a href="https://github.com/HKUST-Aerial-Robotics/Pagor/stargazers">
<img alt="stars" src="https://img.shields.io/github/stars/HKUST-Aerial-Robotics/Pagor" />
</a>
<a href="https://github.com/HKUST-Aerial-Robotics/Pagor/network/members">
<img alt="FORK" src="https://img.shields.io/github/forks/HKUST-Aerial-Robotics/Pagor?color=FF8000" />
</a>
<a href="https://github.com/HKUST-Aerial-Robotics/Pagor/issues">
<img alt="Issues" src="https://img.shields.io/github/issues/HKUST-Aerial-Robotics/Pagor?color=0088ff"/>
</a>
</div>

## Introduction

This is the official code repository of "Pyramid Semantic Graph-based Global Point Cloud Registration with Low Overlap", which is accepted by IROS'23.

Pagor (PyrAmid Graph-based GlObal Registration) is a robust global point cloud registration algorithm for LiDAR. It takes two point clouds and their semantic labels as input and estimates the relative pose between them.

<div align="center">
    <img src="./docs/cover.png" width="640" alt="cover">
</div>

## NEWS
- An **improved version** can be found in **[G3Reg](https://github.com/HKUST-Aerial-Robotics/G3Reg)**.
- Welcome to try our new **[LiDAR Registration Benchmark](https://github.com/HKUST-Aerial-Robotics/LiDAR-Registration-Benchmark)** which is a comprehensive benchmark for LiDAR registration in robotic applications.

## Prerequisites
### ROS
Follow the [official guide](http://wiki.ros.org/ROS/Installation) to install ROS1.
### GTSAM
Follow the [official guide](https://gtsam.org/get_started/) to install GTSAM
### PCL
Follow the [official guide](https://pointclouds.org/downloads/) to install PCL
### Ubuntu packages
```angular2html
sudo apt install cmake libeigen3-dev libboost-all-dev libgoogle-glog-dev libyaml-cpp-dev
```

## Build and Run
```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/HKUST-Aerial-Robotics/Pagor.git
cd .. && catkin_make
```
If you want to reproduce the KITTI benchmark results of the paper, you can download the KITTI semantic labels [[OneDrive]](https://hkustconnect-my.sharepoint.com/:u:/g/personal/zqiaoac_connect_ust_hk/Ebw6JHkgeKBMhOARra0zNz4BxqK44ye5Qli0el-_g7ioiw?e=nrT4Ji)[[Baidu Cloud]](https://pan.baidu.com/s/1LV6UqJTNFp4SpCSP2N6hxQ?pwd=2023), which is 
generated by LiDAR segmentation model [SalsaNext](https://github.com/TiagoCortinhal/SalsaNext.git). Then merge the downloaded folder with the original KITTI odometry LiDAR dataset, and then modify the path in the configuration file `configs/pagor.yaml`.
```angular2html
dataset:
  name: kitti
  dataset_path: "/media/qzj/Document/datasets/KITTI/odometry/data_odometry_velodyne/dataset/sequences"
  split_dir: "data_split/kitti"
  label_dir: "/labels_salsanext/"
```
The dataset folder structure is as follows:
```angular2html
dataset/
└── sequences
    └── 00
        ├── calib.txt
        ├── labels_salsanext
        ├── poses.txt
        ├── times.txt
        └── velodyne
```
Run the following command:
```
source devel/setup.bash
cd src/Pagor
../../devel/lib/pagor/kitti_bm pagor.yaml
```

## Citation
If you find this work useful in your research, please consider citing:
```
@inproceedings{qiao2023pyramid,
  title={Pyramid Semantic Graph-based Global Point Cloud Registration with Low Overlap},
  author={Qiao, Zhijian and Yu, Zehuan and Yin, Huan and Shen, Shaojie},
  booktitle={2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={11202--11209},
  year={2023},
  organization={IEEE}
}
```

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgements
We want to express our deepest gratitude to the creators of the repositories listed below for sharing their work with the public:
* [Teaser](https://github.com/MIT-SPARK/TEASER-plusplus) (baseline)
* [Quatro](https://github.com/url-kaist/Quatro) (baseline)
* [Segregator](https://github.com/Pamphlett/Segregator) (baseline)
* [CLIPPER](https://github.com/mit-acl/clipper) (maximum clique solver)
* [T-LOAM](https://github.com/zpw6106/tloam) (clustering) 
* [SalsaNext](https://github.com/TiagoCortinhal/SalsaNext.git) (semantic segmentation)
