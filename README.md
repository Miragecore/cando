# cando
- Raspberry 4B
- Ubuntu 20.04 / ROS2 galactic
- StereoCamera : [Arducam 4Channel Adapeter](https://www.arducam.com/16mp-autofocus-camera-for-raspberry-pi/) + two [8MP Rpicamera V2](https://www.arducam.com/16mp-autofocus-camera-for-raspberry-pi/)
- [yaboom robot](https://www.arducam.com/16mp-autofocus-camera-for-raspberry-pi/)

I planned mobile robot that moves around my home.
It will construct with [yaboom robot H/W](https://category.yahboom.net/collections/r-4-wheels-drive/products/raspbot) & stereo camera.
yaboom robot will use for moving parts that controled by I2C and one Ultrasonic sensor. other original parts of yaboom will be remove.
stereo camera will be made by [arducam's 4 Channel multiplexer](https://www.arducam.com/product/multi-camera-v2-1-adapter-raspberry-pi/) and two [8MP raspberry camera V2](https://www.arducam.com/16mp-autofocus-camera-for-raspberry-pi/) and will make it produce a disprity Image based on [AnyNet](https://github.com/lrse/sptam) or [RTSSNet](https://github.com/lrse/sptam) CNN Network.
after all things are built, I will try [S-PTAM](https://github.com/lrse/sptam) algorithms.

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
AnyNet is CNN network. and it's implementation is based on CUDA. 
so, I modify some coded to run rasberrypi's single CPU and it added as submodule.  
cpu version : [AnyNet](https://github.com/Miragecore/AnyNet/tree/devel)  
Test on Colab AnyNet : [Colab Test](https://github.com/Miragecore/SandBox/blob/main/AnyNetTest-cpu.ipynb)

### when load GPU pretrained model to CPU model  
GPU pretrained model dict's keys are have 'module.' prefix. so, need to remove it for load on single CPU.  
```python
from collections import OrderedDict
new_state_dict = OrderedDict()

for k, v in checkpoint['state_dict'].items():
  name = k.replace("module.", "") # remove `module.`
  new_state_dict[name] = v

model.load_state_dict(new_state_dict, strict=False)
```

## wiringPi
```
git clone https://github.com/WiringPi/WiringPi.git && cd WiringPi/ && \
        ./build && cd .. && rm -rf WiringPi/
```

## ROS2 galactic package
### [vision_Opencv : image_geometry, cv_bridge](https://github.com/ros-perception/vision_opencv)
### [image_common : image_common, image_pipeline, camera_calibration-parsers](https://github.com/ros-perception/image_common)

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
