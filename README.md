# Insulator Extraction from UAV LiDAR Point Cloud Based on Multi-Type and Multi-Scale Feature Histogram

This repository contains the official implementation for the paper:  
**"Chen, M.; Li, J.; Pan, J.; Ji, C.; Ma, W. Insulator Extraction from UAV LiDAR Point Cloud Based on Multi-Type and Multi-Scale Feature Histogram. Drones 2024, 8, 241."**​  
 
(https://doi.org/10.3390/drones8060241)

## 📝 Overview
A novel method for automated insulator extraction from UAV LiDAR point clouds using:
- ​**5 Feature Histograms**: HD/HV/HW/VW/VV
- ​**Multi-Scale Adaptive Grids**
- ​**Tower-Type Specific Processing**
- ​**2-Stage Workflow**: Pylon/PL refinement + Insulator extraction

Achieves 100% object accuracy for suspension towers and 97.3% for tension towers with <2s processing per tower.

## 🛠 Installation
### Requirements
- This work is implemented on MATLAB 2022b

### Setup
1. Clone repository:
```bash
git clone https://github.com/c175044/Insulator-Segment-Using-Multi-histogram.git
Add the "function" to the work path

run the script "main.m" or App
