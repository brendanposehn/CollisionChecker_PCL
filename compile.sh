#!/usr/bin/env bash

set -eux

(
    mkdir -pv build
    cd build
    cmake ../
    make -j 8
)


