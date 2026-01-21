FROM dist:5000/plusai/drive:latest

RUN rm -rf /tmp/plus_carla/Build/

COPY Examples /tmp/plus_carla/Examples/
COPY Import /tmp/plus_carla/Import/
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

WORKDIR /tmp/plus_carla/

RUN sudo apt-get update && \
    sudo apt-get install wget software-properties-common && \
    sudo add-apt-repository ppa:ubuntu-toolchain-r/test && \
    wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key|sudo apt-key add

RUN sudo apt-add-repository "deb http://archive.ubuntu.com/ubuntu focal main universe" && \
    sudo apt-get update && \
    sudo apt-get install -y build-essential clang-10 lld-10 g++-7 cmake ninja-build libvulkan1 python python3 libc++-dev libc++abi-dev libc6-dev \
    python3-dev python3-pip libpng-dev libtiff5-dev libjpeg-dev tzdata sed curl unzip autoconf libtool rsync libxml2-dev git git-lfs && \
    sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-10/bin/clang++ 180 && \
    sudo update-alternatives --install /usr/bin/clang clang /usr/lib/llvm-10/bin/clang 180 && \
    sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 180 && \
    sudo update-alternatives --install /usr/bin/python python /usr/bin/python2.7 1 && \
    sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.8 2

RUN make clean
RUN make PythonAPI

WORKDIR /tmp/plus_carla/
RUN mkdir -p /opt/plusai/carla
RUN cp -r PlusCarla /opt/plusai/carla/
RUN cp -r PythonAPI /opt/plusai/carla/
#RUN pip3 install -r /opt/plusai/carla/PythonAPI/examples/requirements.txt
RUN rm -rf /tmp/plus_carla/

# COPY setup-plus-carla.sh /opt/plusai/carla/carla_client_entrypoint.sh
# COPY run_client.sh /opt/plusai/carla/run_client.sh
# RUN chmod +x /opt/plusai/carla/carla_client_entrypoint.sh
# RUN chmod +x /opt/plusai/carla/run_client.sh

ENTRYPOINT ["/bin/bash"]
