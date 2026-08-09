#!/bin/bash
# Two-phase AppImage build.
#
# appimage-builder's libc integration has three bugs when targeting Noble:
#   1. Patches ELF interpreters to relative paths then removes lib64/ld-linux
#      → AppRun can't find the ld-linux and segfaults
#   2. Leaves a stale older ld-linux in runtime/compat/lib/ from a previous
#      Jammy build, mismatching the Noble libc.so.6
#   3. Patches (corrupts) the bundled libc.so.6 via RPATH tools, causing a
#      segfault in glibc init when the compat layer is used
#
# We fix all three by running the deploy phase, then restoring clean copies of
# the affected glibc files, then running the package phase.
#
# IMPORTANT: those glibc copies must come from the SAME glibc the recipe
# deploys (noble / 2.39), NOT from whatever the build host happens to run.
# This used to be implicit because the build host was Noble; once the host was
# upgraded to 26.04 (glibc 2.43) the copies started injecting 2.43 files into a
# 2.39 bundle, which breaks on every machine that isn't 26.04.  GLIBC_LIB now
# defaults to the host but is checked against the recipe below.
#
# Do not run this directly for a release - use ./release_appimage.sh, which
# compiles and packages inside the Noble container.  In there the default
# GLIBC_LIB is already the right 2.39 tree and the guard below passes.
#
# Running it on the host only works if the host *is* the release the recipe
# targets; otherwise point GLIBC_LIB at a matching glibc tree, e.g.
#
#   docker create --name noble ubuntu:24.04
#   docker cp noble:/usr/lib/x86_64-linux-gnu/libc.so.6            /tmp/noble-glibc/
#   docker cp noble:/usr/lib/x86_64-linux-gnu/ld-linux-x86-64.so.2 /tmp/noble-glibc/
#   docker rm noble
#   BUILD_DIR=build-noble GLIBC_LIB=/tmp/noble-glibc ./appimage_build.sh
#
# Usage: ./appimage_build.sh [path_to_appimage_builder]
set -e

APPIMAGE_BUILDER="${1:-/home/developer/Downloads/appimage-builder-x86_64.AppImage}"
APPDIR="AppDir"
BUILD_DIR="${BUILD_DIR:-build}"
GLIBC_LIB="${GLIBC_LIB:-/lib/x86_64-linux-gnu}"

cd "$(dirname "$0")"

# Install project libraries, binary and html into AppDir
cmake --install "$BUILD_DIR" --prefix "$APPDIR"

# Strip the installed copies only; the build tree keeps its debug info.
strip --strip-unneeded "$APPDIR"/usr/lib/*.so "$APPDIR"/localization_service_host

# Phase 1: deploy apt packages and patch AppDir
"$APPIMAGE_BUILDER" --skip-appimage --skip-tests

# Guard: the glibc we are about to copy in must match what phase 1 deployed.
# Mixing versions yields GLIBC_PRIVATE symbol errors on users' machines, and
# the failure only shows up on hosts older than the bundle - never here.
recipe_libc=$(sed -n 's/^APPDIR_LIBC_VERSION=//p' "$APPDIR/AppRun.env")
source_libc=$(objdump -T "$GLIBC_LIB/libc.so.6" \
              | grep -oE 'GLIBC_2\.[0-9]+' | sort -uV | tail -1 | sed 's/GLIBC_//')
if [ "$recipe_libc" != "$source_libc" ]; then
    echo "ERROR: glibc mismatch." >&2
    echo "  recipe deployed : $recipe_libc  (AppRun.env)" >&2
    echo "  GLIBC_LIB is    : $source_libc  ($GLIBC_LIB)" >&2
    echo "Point GLIBC_LIB at a glibc $recipe_libc tree - see the header." >&2
    exit 1
fi

# The exact compat layout varies between appimage-builder runs, so the fixes
# below adapt to whichever directories exist rather than hardcoding paths.  A
# long-lived AppDir that is never wiped accumulates directories from older
# builds (that is where runtime/compat/lib/x86_64-linux-gnu came from), and
# hardcoding those made this script fail against a clean AppDir.

# Fix 1: refresh every compat ld-linux that exists, so none is left behind at
# an older version than the deployed libc.
found=0
for d in runtime/compat/lib64 \
         runtime/compat/usr/lib64 \
         runtime/compat/usr/lib/x86_64-linux-gnu \
         runtime/compat/lib/x86_64-linux-gnu; do
    if [ -d "$APPDIR/$d" ]; then
        cp "$GLIBC_LIB/ld-linux-x86-64.so.2" "$APPDIR/$d/ld-linux-x86-64.so.2"
        found=1
    fi
done
if [ "$found" -eq 0 ]; then
    echo "ERROR: no compat lib dir under $APPDIR/runtime/compat" >&2
    exit 1
fi

# Fix 2: AppRun resolves APPDIR_LIBC_LINKER_PATH relative to the AppDir root,
# so lib64/ld-linux-x86-64.so.2 must exist there.  appimage-builder deletes it
# during deploy, so it is always restored here.  This also overrides
# CMakeLists' find_file(LD_LINUX_SO), which picks up the build *host's* linker.
mkdir -p "$APPDIR/lib64"
cp "$GLIBC_LIB/ld-linux-x86-64.so.2" "$APPDIR/lib64/ld-linux-x86-64.so.2"

# Fix 3: appimage-builder sometimes patches the bundled libc.so.6 (RPATH tools
# corrupt it), causing a segfault in glibc init.  Restore an unmodified copy
# wherever the deploy placed it.
libc_target=$(find "$APPDIR/runtime/compat" -name libc.so.6 -type f | head -1)
if [ -z "$libc_target" ]; then
    echo "ERROR: no libc.so.6 under $APPDIR/runtime/compat" >&2
    exit 1
fi
cp "$GLIBC_LIB/libc.so.6" "$libc_target"

# NOTE: do NOT try to patchelf --remove-needed libGLX.so.0 out of
# libopencv_core.  Ubuntu builds it with BIND_NOW (full RELRO), so every
# undefined symbol must resolve at load time even if never called; dropping the
# NEEDED entry makes it fail immediately with
#   undefined symbol: glXGetProcAddressARB
# on every host, headless or not.  Removing the GL dependency for real requires
# an OpenCV built with WITH_OPENGL=OFF, which Ubuntu does not package.

# Phase 2: package the AppImage from the current AppDir state (no re-deploy)
"$APPIMAGE_BUILDER" --skip-build --skip-tests
