#!/usr/bin/env bash

cd "$( dirname "${BASH_SOURCE[0]}" )"
source ../common.sh

export PIPPROXY="-i https://bbpteam.epfl.ch/repository/devpi/simple"

docker run --rm \
    -e http_proxy=$http_proxy \
    -e https_proxy=$https_proxy \
    -e PIPPROXY="$PIPPROXY" \
    -e UID=$UID \
    -v $BASE:/io:Z \
    bbpdocker.epfl.ch/brain_wheel \
    /bin/bash /io/packaging/python_wheels/linux/docker_build_wheel.sh
