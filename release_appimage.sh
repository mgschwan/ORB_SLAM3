#!/bin/bash
# End-to-end release build of ORBSlammer-latest-x86_64.AppImage.
#
# Everything runs inside the Noble container (see Dockerfile.noble), because
# the recipe deploys its runtime from noble and three separate things break
# when the build host is newer:
#
#   1. binaries link against the host's glibc, which the 2.39 bundle cannot
#      satisfy - and AppRun only engages its compat layer on hosts *older*
#      than the bundle, so even Ubuntu 24.04 users get "GLIBC_x.yz not found"
#   2. appimage_build.sh's glibc fixes would copy the host's ld-linux/libc.so.6
#      into a 2.39 bundle, mixing GLIBC_PRIVATE symbols across versions
#   3. appimage-builder hard-requires apt-key, which recent Ubuntu dropped
#
# Running the packaging step in the container fixes all three at once: inside
# it, appimage_build.sh's default GLIBC_LIB is already the correct 2.39 tree.
#
# Requires: docker, and the appimage-builder AppImage (any recent release from
# https://github.com/AppImageCrafters/appimage-builder/releases).
#
# Usage:
#   ./release_appimage.sh [path_to_appimage_builder]
#   APPIMAGE_BUILDER=/path/to/builder.AppImage ./release_appimage.sh
set -e

cd "$(dirname "$0")"
REPO="$PWD"

APPIMAGE_BUILDER="${1:-${APPIMAGE_BUILDER:-$HOME/Downloads/appimage-builder-x86_64.AppImage}}"
IMAGE="${IMAGE:-orbslam3-noble-build}"
WORK="$REPO/.release-work"

if [ ! -x "$APPIMAGE_BUILDER" ]; then
    echo "ERROR: appimage-builder not found at: $APPIMAGE_BUILDER" >&2
    echo "Pass its path as \$1 or set APPIMAGE_BUILDER." >&2
    exit 1
fi

DOCKER_RUN=(docker run --rm -u "$(id -u):$(id -g)" -v "$REPO:$REPO" -w "$REPO")

echo "### 1/4  Building the Noble build image"
docker build -f Dockerfile.noble -t "$IMAGE" .

echo "### 2/4  Compiling ORB_SLAM3 against glibc 2.39"
"${DOCKER_RUN[@]}" "$IMAGE" bash "$REPO/build_noble.sh" "$REPO"

# appimage-builder ships as an AppImage, which needs FUSE to self-mount.  There
# is no FUSE in the container, so extract it on the host and mount the result.
echo "### 3/4  Extracting appimage-builder"
mkdir -p "$WORK"
( cd "$WORK" && rm -rf squashfs-root && "$APPIMAGE_BUILDER" --appimage-extract >/dev/null )

# AppDir is always wiped: appimage-builder never removes files it did not just
# write, so a reused AppDir accumulates stale libraries from previous builds
# (including ones from older distro releases) and silently packages them.
echo "### 4/4  Packaging the AppImage"
rm -rf AppDir
"${DOCKER_RUN[@]}" \
    -v "$WORK/squashfs-root:/opt/appimage-builder:ro" \
    -e BUILD_DIR=build-noble \
    "$IMAGE" bash -c './appimage_build.sh /opt/appimage-builder/AppRun'

echo
echo "=== Release artifact ==="
ls -lh "$REPO"/ORBSlammer-latest-x86_64.AppImage*
echo
echo "Users need these on the host (OpenCV links libGLX; videoio pulls in X11/VA):"
echo "  libglx0 libx11-6 libxcb1 libxcb-render0 libxcb-shm0 libxcb-dri3-0 libx11-xcb1 libdrm2"
