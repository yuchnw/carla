FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# -------------------------
# Core system utilities
# -------------------------
RUN apt-get update && apt-get install -y \
    sudo \
    ca-certificates \
    curl \
    wget \
    gnupg \
    lsb-release \
    software-properties-common \
    build-essential \
    pkg-config \
    libglib2.0-0 \
    libstdc++6 \
    libtbb2 \
    libjpeg-dev \
    libpng-dev \
    libgl1 \
    libx11-6 \
    libxrandr2 \
    libxi6 \
    libxinerama1 \
    libxcursor1 \
    git \
    rsync \
    ninja-build \
    cmake \
    clang-10 \
    lld-10 \
    autoconf \
    automake \
    libtool \
    m4 \
    unzip \
    libtiff-dev \
    fontconfig \
    && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL http://dist/apt.key | apt-key add - && \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/7fa2af80.pub | apt-key add - && \
    curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/3bf863cc.pub | apt-key add - && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /etc/apt/sources.list.d && \
    echo "deb [trusted=yes] http://dist/dists/master_ubuntu20.04/binary/ /" > /etc/apt/sources.list.d/plusai.list && \
    echo "deb [trusted=yes] http://dist/dists/common_base_ubuntu20.04/binary/ /" > /etc/apt/sources.list.d/plusai_common_base.list

# -------------------------
# CUDA
# -------------------------

# Add Nvidia cuda apt repos needed for following steps.
# Nvidia apt repos are having frequent outage.
# Limit the scope and remove after use to avoid the issues since this section
# won't change frequently and will be cached during docker image building.
RUN echo "deb [trusted=yes] https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64 /" > /etc/apt/sources.list.d/cuda.list && \
    echo "deb [trusted=yes] https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu2004/x86_64  /" > /etc/apt/sources.list.d/nvidia-ml.list

ENV CUDA_VERSION=11.8.0

# Ideally we should use CUDA 11.4 here which is same with V3NK, but it doesn't support 40xx GPUs.
# So we choose to use the closest version which support 40xx GPUs here.
# see https://gitlab.com/nvidia/container-images/cuda/-/tree/master/dist/11.8.0/ubuntu2004
# for more details
# This also install ptxas from the nvcc package without any of the dependencies by manually
# downloading and decompressing the package
ENV NV_CUDA_CUDART_VERSION=11.8.89-1
ENV NV_CUDA_LIB_VERSION=11.8.0-1
ENV NV_NVTX_VERSION=11.8.86-1
ENV NV_LIBNPP_VERSION=11.8.0.86-1
ENV NV_LIBCUSPARSE_VERSION=11.7.5.86-1
ENV NV_LIBCUBLAS_VERSION=11.11.3.6-1
ENV NV_LIBNCCL_PACKAGE_VERSION=2.16.2-1

RUN apt-get update && apt-get install -y --no-install-recommends \
        cuda-compat-11-8 \
        cuda-cudart-11-8=${NV_CUDA_CUDART_VERSION} \
        cuda-libraries-11-8=${NV_CUDA_LIB_VERSION} \
        libnpp-11-8=${NV_LIBNPP_VERSION} \
        cuda-nvtx-11-8=${NV_NVTX_VERSION} \
        libcusparse-11-8=${NV_LIBCUSPARSE_VERSION} \
        libcublas-11-8=${NV_LIBCUBLAS_VERSION} \
        libnccl2=${NV_LIBNCCL_PACKAGE_VERSION}+cuda11.8 && \
    apt-get download cuda-nvcc-11-8=${NV_CUDA_CUDART_VERSION} && \
    mkdir /tmp/ptxas && \
    dpkg-deb --extract cuda-nvcc-11-8_*.deb /tmp/ptxas && \
    rm cuda-nvcc-11-8_*.deb && \
    mkdir -p /usr/local/cuda-11.8/bin/ && \
    mkdir -p /usr/local/cuda-11.8/nvvm/libdevice && \
    cp /tmp/ptxas/usr/local/cuda-11.8/bin/ptxas /usr/local/cuda-11.8/bin/ && \
    cp /tmp/ptxas/usr/local/cuda-11.8/nvvm/libdevice/libdevice.10.bc /usr/local/cuda-11.8/nvvm/libdevice/ && \
    ln -s cuda-11.8 /usr/local/cuda && \
    rm -rf /var/lib/apt/lists/*

# cuDNN
ENV CUDNN_VERSION=8.9.2.26

ENV CUDNN_PKG_VERSION=${CUDNN_VERSION}-1+cuda11.8

RUN apt-get update && apt-get install -y --no-install-recommends \
        libcudnn8=${CUDNN_PKG_VERSION} && \
    rm -rf /var/lib/apt/lists/*

# ROS
ENV UBUNTU_VERSION=focal
ENV ROS_DISTRO=noetic
# hadolint ignore=DL3048
LABEL ROS_DISTRO="${ROS_DISTRO}"

# Official ROS repo
# dataspeed-can-usb is needed by dbw_mkz_ros repo
# diagnostic-updater is needed by plusai_novatel_span_driver repo
# serial is needed by advanced_navigation repo
# vision-msgs is needed by aeva-ros repo
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu '${UBUNTU_VERSION}' main" > /etc/apt/sources.list.d/ros-latest.list' && \
    mkdir -p /etc/ros/rosdep/sources.list.d/ && \
    sh -c 'echo "yaml http://packages.dataspeedinc.com/ros/ros-public-'${ROS_DISTRO}'.yaml '${ROS_DISTRO}'" > /etc/ros/rosdep/sources.list.d/30-dataspeed-public-'${ROS_DISTRO}'.list' && \
    sh -c 'echo "yaml http://dist/plusai-rosdep-'${ROS_DISTRO}'.yaml" > /etc/ros/rosdep/sources.list.d/10-plusai-'${ROS_DISTRO}'.list' && \
    apt-get update && apt-get -y --no-install-recommends install \
        ros-${ROS_DISTRO}-angles \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-dataspeed-can-usb \
        ros-${ROS_DISTRO}-diagnostic-updater \
        ros-${ROS_DISTRO}-geodesy \
        ros-${ROS_DISTRO}-pcl-msgs \
        ros-${ROS_DISTRO}-pcl-conversions \
        ros-${ROS_DISTRO}-pcl-ros \
        ros-${ROS_DISTRO}-ros-base \
        ros-${ROS_DISTRO}-ros-numpy \
        ros-${ROS_DISTRO}-serial \
        ros-${ROS_DISTRO}-tf \
        ros-${ROS_DISTRO}-vision-msgs \
        && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN wget https://www.python.org/ftp/python/3.10.13/Python-3.10.13.tgz && \
    tar -xzf Python-3.10.13.tgz && \
    cd Python-3.10.13 && \
    ./configure --enable-optimizations && \
    make && \
    make altinstall && \
    cd / && rm -rf Python-3.10.13*

# # -------------------------
# # Python 3.10 + pip
# # -------------------------

# Upgrade pip stack
RUN python3.10 -m ensurepip --upgrade && \
    python3.10 -m pip install --upgrade pip setuptools wheel protobuf==3.6.1 rospkg opencv-python

RUN mkdir -p /opt/packages
ARG RADAR_PKG
ARG COMMON_PROTOBUF_PKG
COPY ${RADAR_PKG} /opt/packages/${RADAR_PKG}
COPY ${COMMON_PROTOBUF_PKG} /opt/packages/${COMMON_PROTOBUF_PKG}

RUN dpkg -i /opt/packages/${RADAR_PKG}
RUN dpkg -i /opt/packages/${COMMON_PROTOBUF_PKG}

RUN rm -rf /opt/packages
RUN /usr/local/bin/python3.10 -m pip install "protobuf>=3.6.1,<4.0"
