ARG BASE
FROM ${BASE}

RUN rm -rf /tmp/plus_carla/Build/

# COPY Import /tmp/plus_carla/Import/
COPY LibCarla /tmp/plus_carla/LibCarla/
COPY PlusCarla /tmp/plus_carla/PlusCarla/
COPY PythonAPI /tmp/plus_carla/PythonAPI/
COPY Util /tmp/plus_carla/Util/
COPY CMakeLists.txt /tmp/plus_carla/
COPY Makefile /tmp/plus_carla/

RUN rm -rf /tmp/plus_carla/Build/
RUN rm -rf /tmp/plus_carla/PythonAPI/carla/build/
RUN rm -rf /tmp/plus_carla/PythonAPI/carla/dist/
RUN rm -rf /tmp/plus_carla/PythonAPI/carla/dependencies/
RUN rm -rf /tmp/plus_carla/PlusCarla/build/
RUN rm -rf /tmp/plus_carla/PlusCarla/devel/


WORKDIR /tmp/plus_carla/

RUN sudo apt-add-repository "deb http://archive.ubuntu.com/ubuntu focal main universe" && \
    sudo apt-get update && \
    sudo apt-get install -y build-essential clang-10 lld-10 g++-7 cmake ninja-build libvulkan1 libc++-dev libc++abi-dev libc6-dev \
    libpng-dev libtiff5-dev libjpeg-dev tzdata sed curl unzip autoconf libtool rsync libxml2-dev git git-lfs && \
    sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-10/bin/clang++ 180

RUN python3.10 -m pip install numpy==1.26.4
RUN python3.10 -m pip install grpcio==1.76.0
RUN python3.10 -m pip install protobuf==3.20.3
RUN python3.10 -m pip install pyproj==3.7.1
RUN python3.10 -m pip install scipy==1.15.3
RUN python3.10 -m pip install --upgrade -r /tmp/plus_carla/PythonAPI/carla/requirements.txt

RUN make PythonAPI

WORKDIR /tmp/plus_carla/
RUN mkdir -p /opt/plusai/carla
RUN cp -r PlusCarla /opt/plusai/carla/
RUN cp -r PythonAPI /opt/plusai/carla/
RUN rm -rf /tmp/plus_carla/


ENTRYPOINT ["/bin/bash"]