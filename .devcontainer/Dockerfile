FROM althack/ros2:foxy-dev 

# ** [Optional] Uncomment this section to install additional packages. **
#
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get -y install --no-install-recommends curl zip unzip tar git \
   #
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

RUN cd / && git clone --depth 200 https://github.com/Microsoft/vcpkg.git && cd /vcpkg && \
git checkout -b our_pinned 8d5a4268a2b7cb079c3a37d70af75269c2f1f42d && \
./bootstrap-vcpkg.sh -disableMetrics

RUN cd /vcpkg && \
    ./vcpkg integrate install && \
    ./vcpkg integrate bash && \
    ./vcpkg \
    # list of librarries that we eill be using:
    install boost-atomic boost-interprocess boost-date-time

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc