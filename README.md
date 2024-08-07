[![Vision Homepage](https://img.shields.io/badge/roboticslab-vision-orange.svg)](https://robots.uc3m.es/vision/) [![Latest Release](https://img.shields.io/github/tag/roboticslab-uc3m/vision.svg?label=Latest%20Release)](https://github.com/roboticslab-uc3m/vision/tags)

Vision processing.

Link to Doxygen generated documentation: https://robots.uc3m.es/vision/

## Installation

Installation instructions for installing from source can be found [here](doc/vision-install.md).

## Contributing

#### Posting Issues

1. Read [CONTRIBUTING.md](CONTRIBUTING.md)
2. [Post an issue / Feature request / Specific documentation request](https://github.com/roboticslab-uc3m/vision/issues)

#### Fork & Pull Request

1. [Fork the repository](https://github.com/roboticslab-uc3m/vision/fork)
2. Create your feature branch (`git checkout -b my-new-feature`) off the `master` branch, following the [Forking Git workflow](https://www.atlassian.com/git/tutorials/comparing-workflows/forking-workflow)
3. Commit your changes
4. Push to the branch (`git push origin my-new-feature`)
5. Create a new Pull Request

## Citation

If you found this project useful, please consider citing the following works:

- [YarpCloudUtils](libraries/YarpCloudUtils/)

Bartek Łukawski, Alberto Rodríguez-Sanz, Elisabeth Menendez, Juan G. Victores, and Carlos Balaguer. A user-friendly point cloud processing pipeline for interfacing PCL with YARP. In *XLV Jornadas de Automática*. Universidade da Coruña, 2024.

```bibtex
@inproceedings{lukawski2024jjaa,
    author    = {{\L}ukawski, Bartek and Rodríguez-Sanz, Alberto and Menendez, Elisabeth and Victores, Juan G. and Balaguer, Carlos},
    title     = {A user-friendly point cloud processing pipeline for interfacing {PCL} with {YARP}},
    booktitle = {XLV Jornadas de Automática},
    year      = {2024},
    publisher = {Universidade da Coruña},
    doi       = {10.17979/ja-cea.2024.45.10925},
}
```

## Status

[![Continuous Integration](https://github.com/roboticslab-uc3m/vision/actions/workflows/ci.yml/badge.svg)](https://github.com/roboticslab-uc3m/vision/actions/workflows/ci.yml)

[![Issues](https://img.shields.io/github/issues/roboticslab-uc3m/vision.svg?label=Issues)](https://github.com/roboticslab-uc3m/vision/issues)

## Similar and Related Projects

### General
- [GRIP](https://wpiroboticsprojects.github.io/GRIP) ([WPIRoboticsProjects/GRIP](https://github.com/WPIRoboticsProjects/GRIP)): A tool for developing computer vision algorithms interactively. We have used it for several projects, including [asrob-uc3m/air-hockey](https://github.com/asrob-uc3m/air-hockey/issues/5). [tutorial](http://wpilib.screenstepslive.com/s/4485/m/24194/l/463566-introduction-to-grip).
- [TVM](https://tvm.ai) ([dmlc/tvm](https://github.com/dmlc/tvm)): Open Deep Learning Compiler Stack
- [PCL](http://pointclouds.org) ([PointCloudLibrary/pcl](https://github.com/PointCloudLibrary/pcl)): 2D/3D image and point cloud processing
- [CloudCompare/CloudCompare](https://github.com/CloudCompare/CloudCompare): A 3D point cloud (and triangular mesh) processing software
- [pmp-library/pmp-library](https://github.com/pmp-library/pmp-library): Interactive cloud visualization and processing tools
- [MIT-SPARK/Kimera-VIO](https://github.com/MIT-SPARK/Kimera-VIO): Visual Inertial Odometry with SLAM capabilities and 3D Mesh generation
- [NVlabs/BundleSDF](https://github.com/NVlabs/BundleSDF): Neural 6-DoF Tracking and 3D Reconstruction of Unknown Objects
- YARP
    - [robotology/segmentation](https://github.com/robotology/segmentation) ([doxygen](http://robotology.github.io/segmentation/doxygen/doc/html/modules.html))
    - [icVision](http://juxi.net/projects/icVision) ([Juxi/icVision](https://github.com/Juxi/icVision))

### Visual Servoing
- [ViSP](http://visp.inria.fr/) ([lagadic/visp](https://github.com/lagadic/visp)): Visual Servoing Platform
- YARP
    - [visual-tracking-control](https://robotology.github.io/visual-tracking-control) ([robotology/visual-tracking-control](https://github.com/robotology/visual-tracking-control))
- roboticslab-uc3m
    - [teo-follow-me](https://github.com/roboticslab-uc3m/teo-follow-me)
    - [legacy-matlab-visual-servo](https://github.com/roboticslab-uc3m/legacy-matlab-visual-servo) (legacy)

### Monocular SLAM
- https://github.com/asrob-uc3m/robotDevastation-playground#monocular-slam
