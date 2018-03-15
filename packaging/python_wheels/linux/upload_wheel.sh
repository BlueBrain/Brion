#!/usr/bin/env bash

cd "$( dirname "${BASH_SOURCE[0]}" )"
source ../common.sh

if [ "$RELEASE" == "true" ]; then
    CHANNEL=release
else
    CHANNEL=dev
fi

for wheel in `ls $BASE/packaging/python_wheels/wheelhouse/brain*.whl`; do
    upload2repo -t python -r $CHANNEL -f $wheel
done

# cleanup afterwards
docker run --rm \
    -v $BASE:/io:Z \
    bbpdocker.epfl.ch/brain_wheel \
    /bin/rm -rf /io/packaging/python_wheels/linux/wheelhouse
