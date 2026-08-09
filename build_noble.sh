#!/bin/bash
# Release build of ORB_SLAM3, run *inside* the Noble container.
# Normally invoked by release_appimage.sh rather than directly.
#
# The repo must be mounted at the SAME absolute path as on the host, so the
# paths CMake bakes into build-noble/cmake_install.cmake resolve identically
# inside and outside the container.  Mounting elsewhere (e.g. /src) produces
# install rules pointing at container-only paths, which then fail when
# appimage_build.sh runs `cmake --install`.
#
# Builds into build-noble/ so the host's own build/ tree is left untouched.
set -e

REPO="${1:?usage: build_noble.sh <repo path (same inside and outside the container)>}"
cd "$REPO"

echo "=== Thirdparty/DBoW2 ==="
cmake -S Thirdparty/DBoW2 -B Thirdparty/DBoW2/build-noble -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build Thirdparty/DBoW2/build-noble

echo "=== Thirdparty/g2o ==="
cmake -S Thirdparty/g2o -B Thirdparty/g2o/build-noble -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build Thirdparty/g2o/build-noble

# Sophus is header-only for our purposes - nothing links it - so it is skipped.

echo "=== ORB_SLAM3 ==="
cmake -S . -B build-noble -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-noble

echo "=== DONE ==="
