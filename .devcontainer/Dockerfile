FROM althack/ros2:foxy-dev 

# ** [Optional] Uncomment this section to install additional packages. **
#
RUN apt-get update \
   && apt-get -y install --no-install-recommends curl zip unzip tar \
    gdb rsync  git\
    # for development we use gcc-11 with C++20 support
    # for vcpkg:
    ninja-build pkg-config

# install Cmake 3.21
RUN arhitecture=$(uname -p) && tar_url="https://cmake.org/files/v3.21/cmake-3.21.0-linux-${arhitecture}.tar.gz" && \
    curl -o cmake.tar.gz ${tar_url} && \
    tar -xf cmake.tar.gz && \
    cp -r "cmake-3.21.0-linux-${arhitecture}/bin" /usr/ && \
    cp -r "cmake-3.21.0-linux-${arhitecture}/share"/ /usr/ && \
    cp -r "cmake-3.21.0-linux-${arhitecture}/man"/ /usr/share/ && \
    rm -rf cmak*

# here we have doenloaded and installed all that doesn't need anyting on disk
WORKDIR /
RUN rm -rf /setup

ENV VCPKG_FORCE_SYSTEM_BINARIES=0
# use  --branch 2021.05.12 to download with spesific tag(release) we should move to next 2021 relese
RUN git clone --depth 200 https://github.com/Microsoft/vcpkg.git && cd /vcpkg && \
    #git checkout -b our_pinned 8d5a4268a2b7cb079c3a37d70af75269c2f1f42d && \
    ./bootstrap-vcpkg.sh -disableMetrics
RUN cd /vcpkg && \
    ./vcpkg integrate install && \
    ./vcpkg integrate bash && \
    ./vcpkg \
    # list of librarries that we eill be using:
    install boost-atomic boost-interprocess boost-date-time boost-circulat-buffer

# add vcpkg to PATH, so that it can be called directly from terminal if needed
ENV PATH="/vcpkg:${PATH}"
   #
   # Clean up
RUN apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc