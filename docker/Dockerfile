# Base image is fully self-contained: every build dependency is apt-installed
# below, pywayne (gettool) comes from pip, and Ceres is built from source. The
# project only links Eigen3 + Ceres, so a plain Ubuntu base is enough -- no ROS
# / kalibr image required. Override BASE_IMAGE / BASE_IMAGE_PLATFORM to extend.
ARG BASE_IMAGE_PLATFORM=linux/amd64
ARG BASE_IMAGE=ubuntu:20.04
FROM --platform=${BASE_IMAGE_PLATFORM} ${BASE_IMAGE}

SHELL ["/bin/bash", "-lc"]

ARG BUILD_JOBS=2
ARG CERES_VERSION=2.2.0
ARG PYWAYNE_VERSION=1.0.1.6.3
ARG PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple

ENV DEBIAN_FRONTEND=noninteractive
ENV MPLBACKEND=Agg

RUN set -euo pipefail; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
        ca-certificates \
        cmake \
        git \
        build-essential \
        python3-pip \
        libatlas-base-dev \
        libeigen3-dev \
        libgflags-dev \
        libgoogle-glog-dev \
        libsuitesparse-dev; \
    rm -rf /var/lib/apt/lists/*

RUN set -euo pipefail; \
    # pywayne is installed with --no-deps to skip its heavy dependency tree, so
    # provide the modules gettool actually imports (pywayne.tools pulls these in
    # at import time; PIL/numpy come transitively via matplotlib).
    python3 -m pip install --no-cache-dir -i "${PIP_INDEX_URL}" \
        filelock watchdog matplotlib pyyaml tqdm; \
    python3 -m pip install --no-cache-dir -i "${PIP_INDEX_URL}" --no-deps "pywayne==${PYWAYNE_VERSION}"; \
    gettool ceres -t /tmp/ceres-solver -v "${CERES_VERSION}" -f; \
    cmake -S /tmp/ceres-solver -B /tmp/ceres-solver/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_TESTING=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DMINIGLOG=OFF; \
    cmake --build /tmp/ceres-solver/build -j "${BUILD_JOBS}"; \
    cmake --install /tmp/ceres-solver/build; \
    rm -rf /tmp/ceres-solver

WORKDIR /opt/kalibr-camimu-ceres

COPY CMakeLists.txt CMakeLists.txt
COPY apps apps
COPY include include
COPY src src
COPY tests tests

RUN set -euo pipefail; \
    cmake -S . -B build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCERES_CAM_IMU_BUILD_TESTS=OFF; \
    cmake --build build -j "${BUILD_JOBS}"

ENV PATH="/opt/kalibr-camimu-ceres/build:${PATH}"

CMD ["calibrate_cam_imu", "--help"]
