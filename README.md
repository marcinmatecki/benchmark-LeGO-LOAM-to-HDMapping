# LeGO-LOAM-converter


## Example Dataset: 

Download the dataset `reg-1.bag` by clicking [here](https://cloud.cylab.be/public.php/dav/files/7PgyjbM2CBcakN5/reg-1.bag) from [Bunker DVI Dataset](https://charleshamesse.github.io/bunker-dvi-dataset).

## Intended use 

This small toolset allows to integrate SLAM solution provided by [LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM) with [HDMapping](https://github.com/MapsHD/HDMapping).
This repository contains ROS 1 workspace that :
  - submodule to tested revision of LeGO-LOAM
  - a converter that listens to topics advertised from odometry node and save data in format compatible with HDMapping.

## Dependencies

```shell
sudo apt update
sudo apt install -y docker.io
sudo usermod -aG docker $USER
```

## Convert ros1 CustomMsg to PointCloud2

For usage instructions, click [here](https://github.com/MapsHD/livox_bag_aggregate).

## Workspace

```shell
mkdir -p ros_ws/src/
cd ros_ws/src/
git clone https://github.com/MapsHD/benchmark-LeGO-LOAM-to-HDMapping.git --recursive
```

## Docker build
```shell
cd ros_ws/src/benchmark-LeGO-LOAM-to-HDMapping
docker build -t lego-loam_noetic .
```

## Docker run
```shell
cd ros_ws/src/benchmark-LeGO-LOAM-to-HDMapping
chmod +x docker_session_run-ros1-lego-loam.sh 
docker_session_run-ros1-lego-loam.sh <input_bag> <output_folder>

# For usage instructions or options, you can run:
docker_session_run-ros1-lego-loam.sh --help
```