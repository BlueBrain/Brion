#!/usr/bin/env bash

cd "$( dirname "${BASH_SOURCE[0]}" )"
source ../common.sh

docker build -t brain_wheel \
    --build-arg http_proxy=${HTTP_PROXY-$http_proxy} \
    --build-arg https_proxy=${HTTPS_PROXY-$https_proxy} \
    $BASE/packaging/python_wheels/linux | tee build.log

ID=$(tail -1 build.log | awk '{print $3;}')
upload2repo -t docker -i $ID -g latest -n brain_wheel
