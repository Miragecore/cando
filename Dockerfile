FROM ros:galactic

MAINTAINER soonBba <miragekimys@gmail.com>

# install ros package
RUN apt-get update && \ 
    apt-get install -y locales \
          apt-utils \
          wget \
          git \
          openssh-client \
          unzip

RUN apt-get install -y python3-rosdep \
                       ros-galactic-camera-calibration-parsers \
                       ros-galactic-image-common \
                       ros-galactic-image-pipeline \
                       ros-galactic-image-geometry

# vim
RUN apt-get install -y vim \
      libncurses5-dev \
      libncursesw5-dev 

#Opencv 
RUN apt install -y libopencv-dev 
#boost
RUN apt install -y libboost-all-dev && \
        rm -rf /var/lib/apt/lists/*

#pigpio
RUN wget https://github.com/joan2937/pigpio/archive/master.zip && \
    unzip master.zip && cd pigpio-master/ && make && sudo make install && \
    cd .. && rm -rf pigpio-master/

#wiringPi
RUN git clone https://github.com/WiringPi/WiringPi.git && cd WiringPi/ && \
        ./build && cd .. && rm -rf WiringPi/


#RUN cat /opt/ros/galactic/setup.bash >> ~/.bashrc
RUN echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc

WORKDIR /usr/local/src

# launch ros package
#CMD ["ros2", "launch", "demo_nodes_cpp", "talker_listener.launch.py"]
CMD ["/bin/bash"]
