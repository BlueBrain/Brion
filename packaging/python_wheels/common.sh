#!/bin/bash

pushd "$( dirname "${BASH_SOURCE[0]}" )"

function realpath
{
    python -c "import os; print(os.path.realpath('$1'))"
}


BASE=$(git rev-parse --show-toplevel)
WORKSPACE=$(realpath $BASE/..)

export http_proxy=${HTTP_PROXY-$http_proxy}
export https_proxy=${HTTPS_PROXY-$https_proxy}

NUMPY_VERSION=1.15.0
SPHINX_VERSION=1.3.6

popd
