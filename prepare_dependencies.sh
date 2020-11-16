#!/usr/bin/env bash

set -eux

# Update system packages.
#
# This might not be necessary since CMake seems to built custom packages. -h
(
    export DEBIAN_FRONTEND='noninteractive'
    apt-get -y update
    apt-get -y upgrade
    apt-get -y install build-essential cmake git
    apt-get -y install libpcl-dev libglfw3-dev libglade2-dev libeigen3-dev libusb-dev
)

# git-clone the libigl repo if not present.
[ -d libigl/ ] || git clone https://github.com/libigl/libigl.git libigl/

