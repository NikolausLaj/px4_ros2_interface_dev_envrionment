#!/usr/bin/env bash
set -euo pipefail

CMAKE_FLAGS=" \
   -DCPACK_BINARY_DEB=ON \
   -DBUILD_EXAMPLES=OFF \
   -DBUILD_opencv_python2=OFF \
   -DBUILD_opencv_python3=ON \
   -DBUILD_opencv_java=OFF \
   -DCMAKE_BUILD_TYPE=RELEASE \
   -DCMAKE_INSTALL_PREFIX=/usr/local \
   -DOPENCV_EXTRA_MODULES_PATH=/OpenCV/opencv_contrib/modules \
   -DCUDA_FAST_MATH=ON \
   -DEIGEN_INCLUDE_PATH=/usr/include/eigen3 \
   -DWITH_EIGEN=ON \
   -DOPENCV_ENABLE_NONFREE=OFF \
   -DOPENCV_GENERATE_PKGCONFIG=ON \
   -DBUILD_PERF_TESTS=OFF \
   -DBUILD_TESTS=OFF"

OPENCV_VERSION=$1
BUILD_JOBS=${2:-2}

if ! [[ "${BUILD_JOBS}" =~ ^[1-9][0-9]*$ ]]; then
    echo "BUILD_JOBS must be a positive integer, got: ${BUILD_JOBS}" >&2
    exit 1
fi

mkdir /OpenCV && cd /OpenCV &&

git clone --depth 1 --branch ${OPENCV_VERSION} https://github.com/opencv/opencv.git
git clone --depth 1 --branch ${OPENCV_VERSION} https://github.com/opencv/opencv_contrib.git

cmake -S opencv -B build ${CMAKE_FLAGS} && \
cmake --build build --config Release --parallel "${BUILD_JOBS}" && \
cmake --install build --config Release --prefix ./install
