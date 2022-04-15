# cando
- Raspberry 4B
- Ubuntu 20.04 / ROS2 galactic
- StereoCcmera : Arducam 4Channel Adapeter + 8MP Rpicamera V2(2EA)
- yaboom robot

I Planned that mobile robot based yaboom robot H/W & stereo camera work on S-PTAM algorithm. 
yaboom robot is only for moving parts that controled by I2C. other original parts of yaboom will disparted. 
stereo camera will make by arducam's 4 Channel adapter and two 8MP raspberry camera V2 and it will produce a disprityImage based on AnyNet CNN network.

# dependency

## opencv & boost
```
apt -get install -y libopencv-dev \
	libboost-all-dev \
```

## python packages 
```
pip3 install easydict, opencv-python
```

## torch
```
pip3 install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cpu
```

## [AnyNet](https://github.com/mileyan/AnyNet)
AnyNet is CNN network. but it's implementation is based on CUDA. 
so, I made some modified version to run single CPU
cpu version : [AnyNet](https://github.com/Miragecore/AnyNet/tree/devel)
 

## wiringPi
```
git clone https://github.com/WiringPi/WiringPi.git && cd WiringPi/ && \
        ./build && cd .. && rm -rf WiringPi/
```

## ROS2 galactic package
### [vision_Opencv](https://github.com/ros-perception/vision_opencv)(image_geometry, cv_bridge)
### [image_common](https://github.com/ros-perception/image_common)(image_common, camera_calibration-parsers)
### [image_pipeline](https://github.com/ros-perception/image_common)(image_pipeline)

```shell
apt-get install -y python3-rosdep \
                   ros-galactic-camera-calibration-parsers \
                   ros-galactic-image-common \
                   ros-galactic-image-pipeline \
                   ros-galactic-image-geometry
```

# On RaspberryPI
```shell
git clone https://github.com/Miragecore/cando.git
cd cando
git submodule update --init --recursive
```
