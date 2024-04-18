FROM pointcloudlibrary/env:20.04
LABEL maintainer="Qingwen Zhang <qingwen@kth.se>"

# Just in case we need it
ENV DEBIAN_FRONTEND noninteractive

# install zsh
RUN apt update && apt install -y wget git zsh tmux vim g++ curl

RUN sh -c "$(wget -O- https://raw.githubusercontent.com/Kin-Zhang/Kin-Zhang/main/Dockerfiles/latest_glog_gflag.sh)"
RUN git clone https://github.com/jbeder/yaml-cpp.git && \
    cd yaml-cpp && env CFLAGS='-fPIC' CXXFLAGS='-fPIC' cmake -Bbuild && \
    cmake --build build --config Release --target install

RUN apt update && apt install -y libpcl-dev libopencv-dev gcc-10 g++-10 libtbb-dev liblz4-dev

# since we will output pcd file, don't want to root to lock it. normally 1000 is the first user in our desktop also
RUN useradd -ms /bin/bash -u 1000 kin
USER kin
# setup oh-my-zsh 
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended
RUN printf "y\ny\ny\n\n" | bash -c "$(curl -fsSL https://raw.githubusercontent.com/Kin-Zhang/Kin-Zhang/main/scripts/setup_ohmyzsh.sh)"

RUN mkdir -p /home/kin/workspace && mkdir -p /home/kin/data
RUN git clone --recurse-submodules https://github.com/KTH-RPL/DynamicMap_Benchmark /home/kin/workspace/DynamicMap_Benchmark
WORKDIR /home/kin/workspace/DynamicMap_Benchmark