# A Multi-Layered 3D NDT Scan-Matching for Robust Localizationo in Logistics Warehouse Environments
This repository contains the official implementation of the Multi-layered 3D NDT (Normal Distributions Transform) scan-matching method. This approach is specifically designed for robust robot localization within highly dynamic and cluttered logistics warehouse environments

## Overview
Standard 3D NDT localization often assumes a static environment, which fails in practical warehouse scenarios where goods and layouts change frequently. This project introduces a height-based layering strategy that partitions the 3D point-cloud map and LiDAR scans into multiple layers. By evaluating the uncertainty (covariance determinant) of each layer, the system selectively switches to or fuses more stable layers for localization.

## Key Features
- **Height-Direction Partitioning**: Slices the 3D map into several layers to separate static zones from dynamic zones (e.g., floor-level changes).
- **Uncertainty-Based Selection**: Utilizes the covariance determinant to identify and reject layers with high uncertainty caused by environmental changes.
- **Weighted State Fusion**: Combines the most confident scan-matching estimates using a weighted sum approach to enhance accuracy.
- **Sim-to-Real Validation**: Developed and validated using Nvidia Omniverse Isaac Sim for physically accurate virtual testing.

## Tech Stack
|Category|Technology|
|------|------|
|Framework| ROS Melodic (ROS 1)|
|Language|Python 3.8, C++ 14/17|   
|3D Processing|PCL (Point Cloud Library)|   
|Math Engine|Eigen 3|
|SLAM & Mapping|LIO-SAM|
|Simulation|NVIDIA Omniverse Isaac Sim|
|Preprocessing|CloudCompare|


## Citation
```
@Article{sensors23052671,
AUTHOR = {Kim, Taeho and Jeon, Haneul and Lee, Donghun},
TITLE = {A Multi-Layered 3D NDT Scan-Matching Method for Robust Localization in Logistics Warehouse Environments},
JOURNAL = {Sensors},
VOLUME = {23},
YEAR = {2023},
NUMBER = {5},
ARTICLE-NUMBER = {2671},
URL = {https://www.mdpi.com/1424-8220/23/5/2671},
DOI = {10.3390/s23052671}
}
```
