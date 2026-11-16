# EROAS simulation environment: ROS 2 Jazzy + Gazebo Harmonic + CUDA (for the
# DAVE multibeam sonar). Dependencies only; run_eroas.sh mounts this repo at
# /root/eroas_ws/src/EROAS and builds it inside the container.
#
#   docker build -t eroas:jazzy -f docker/Dockerfile .
FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# ROS / Gazebo / Python dependencies of the workspace packages
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz \
      ros-jazzy-marine-acoustic-msgs \
      ros-jazzy-xacro \
      ros-jazzy-joint-state-publisher \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-teleop-twist-keyboard \
      ros-jazzy-joy \
      ros-jazzy-cv-bridge \
      ros-jazzy-image-transport \
      ros-jazzy-pcl-conversions \
      ros-jazzy-rqt-image-view \
      libopencv-dev libpcl-dev libeigen3-dev libprotobuf-dev \
      python3-numpy python3-scipy python3-opencv python3-matplotlib \
      python3-psutil python3-simplejson python3-yaml python3-transforms3d \
      wget \
    && rm -rf /var/lib/apt/lists/*

# CUDA compiler and libraries from NVIDIA's repo. These packages carry no
# driver libraries: the host driver is injected at run time by the NVIDIA
# Container Toolkit (Ubuntu's nvidia-cuda-toolkit pulls in driver libs that
# clash with the host's and break GPU access).
RUN wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb \
    && dpkg -i cuda-keyring_1.1-1_all.deb && rm cuda-keyring_1.1-1_all.deb \
    && apt-get update && apt-get install -y --no-install-recommends \
      cuda-nvcc-12-6 cuda-cudart-dev-12-6 libcufft-dev-12-6 libcublas-dev-12-6 \
    && rm -rf /var/lib/apt/lists/*
ENV PATH=/usr/local/cuda/bin:${PATH}

# Anything else the package.xml files declare (keys with no rosdep rule are
# skipped; none of them are used by the EROAS demo)
COPY . /tmp/eroas_src
RUN find /tmp/eroas_src -name COLCON_IGNORE -printf '%h\n' | xargs rm -rf \
    && apt-get update \
    && rosdep install --from-paths /tmp/eroas_src --ignore-src -r -y \
         --skip-keys "casadi oberon4_control oberon7_control catkin" \
    ; rm -rf /tmp/eroas_src /var/lib/apt/lists/*

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
    && echo '[ -f /root/eroas_ws/install/local_setup.bash ] && source /root/eroas_ws/install/local_setup.bash' >> /root/.bashrc

ENV NVIDIA_DRIVER_CAPABILITIES=all \
    QT_X11_NO_MITSHM=1
WORKDIR /root/eroas_ws
CMD ["sleep", "infinity"]
