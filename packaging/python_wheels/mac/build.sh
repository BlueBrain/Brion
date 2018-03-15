#!/bin/bash

cd "$( dirname "${BASH_SOURCE[0]}" )"
source ../common.sh

# Cannot use an earlier version because C++11 features don't compile otherwise
export MACOSX_DEPLOYMENT_TARGET="10.9"
export PYTHON_CONFIGURE_OPTS="--enable-universalsdk=/ --with-universal-archs=intel"
export PYENV_ROOT=$WORKSPACE/pyenv
export PATH="$PYENV_ROOT/bin:$PATH"
PYTHON_VERSIONS="2.7.14 3.4.7 3.5.4 3.6.4"
HDF5=hdf5-1.8.17
BOOST=boost_1_59_0

function install_pyenv
{
    which pyenv && return 0
    git clone https://github.com/pyenv/pyenv.git $WORKSPACE/pyenv
    eval "$(pyenv init -)"
}

function install_python_versions
{
    pyenv install 2.7.14 -s
    pyenv install 3.4.7 -s
    pyenv install 3.5.4 -s
    pyenv install 3.6.4 -s

    export PATH=$(pyenv root)/shims:$PATH
}

function get_python_major_version
{
    $PYTHON -c "import sys; v = list(sys.version_info); print(v[0])"
}

function get_python_version
{
    $PYTHON -c "import sys; v = list(sys.version_info); print('%d.%d' % (v[0], v[1]))"
}

function get_cpython_version
{
    $PYTHON -c "import sys; v = list(sys.version_info); print('cp%d%d' % (v[0], v[1]))"
}

function get_cpython_abi
{
    $PYTHON -c "import sys; v = list(sys.version_info); print('cp%d%d%s' % (v[0], v[1], 'mu' if v[0] == 2 else 'm'))"
}

function get_python_include
{
    $PYTHON -c 'import distutils.sysconfig as s; print(s.get_python_inc())'
}

function get_python_library
{
    pyver=$(get_python_version)
    prefix=$(dirname $(dirname $PYTHON))
    if [ $pyver == 2.7 ]; then
        echo $prefix/lib/libpython$pyver.a
    else
        echo $prefix/lib/libpython${pyver}m.a
    fi
}

function download_hdf5
{
    pushd $WORKSPACE
    if [ -d hdf5-1.8.17 ]; then
        popd
        return
    fi

    curl -OL https://support.hdfgroup.org/ftp/HDF5/releases/hdf5-1.8/$HDF5/src/$HDF5.tar.bz2 && tar xfj $HDF5.tar.bz2
}

function build_hdf5
{
    download_hdf5

    if [ -f $WORKSPACE/$HDF5/build/include/hdf5.h ]; then
        return
    fi

    pushd $WORKSPACE/$HDF5
    ./configure CFLAGS=-fPIC CXXFLAGS=-fPIC --prefix=`pwd`/build --with-pic --disable-shared && make -j2 install
    popd
}

function download_boost
{
    pushd $WORKSPACE
    if [ -d $BOOST ]; then
        popd
        return
    fi

    curl -OL https://downloads.sourceforge.net/project/boost/boost/1.59.0/$BOOST.tar.gz && tar xfz $BOOST.tar.gz
    popd
}

function build_boost
{
    download_boost

    if [ -f $WORKSPACE/$BOOST/build/include/boost/version.hpp ]; then
        return
    fi

    pushd $WORKSPACE/$BOOST
    ./bootstrap.sh --with-libraries=date_time,iostreams,filesystem,program_options,regex,serialization,system,test

    ./b2 -j2 -q                                  \
        --build-type=minimal                     \
        cxxflags="-fPIC -arch i386 -arch x86_64"  \
        cflags="-fPIC -arch i386 -arch x86_64"   \
        threading=multi                          \
        link=static                              \
        install --prefix=`pwd`/build

    popd
}

function build_boost_python
{
    pushd $WORKSPACE/$BOOST

    local inc_dir=$(get_python_include)
    local version=$(get_cpython_version)-$(get_cpython_abi)

    ./bootstrap.sh --with-libraries=python --with-python=$version
    ./b2 --clean

    ./b2 -j2 -q                                  \
        --build-type=minimal                     \
        cxxflags="-fPIC -arch i386 -arch x86_64" \
        cflags="-fPIC -arch i386 -arch x86_64"   \
        threading=multi                          \
        link=static                              \
        include=$inc_dir                         \
        install --prefix=`pwd`/build

    popd
}

function create_venv
{
    # Creating a virtual environment to install version specific dependencies
    pyver=$(get_python_version)
    if [ $pyver == 2.7 ]; then
        pip install virtualenv
        virtualenv $1
    else
        # python -m venv should used for 3.5 and 3.6, but it doesn't bootstrap
        # pip
        pyvenv $1
        alias pip=pip3
    fi
}

function setup_venv
{
    create_venv .
    . bin/activate

    pip install wheel || return 1
    pip install numpy==$NUMPY_VERSION || return 1
    pip install sphinx==$SPHINX_VERSION || return 1
    pip install lxml || return 1
}

function build_brion
{
    rm -r build 2>/dev/null
    mkdir build
    pushd build

    local boost_prefix=$(realpath $WORKSPACE/$BOOST/build)
    local hdf5_prefix=$(realpath $WORKSPACE/$HDF5/build)

    # We have to provide the Python library explicitly because otherwise the
    # CMake code messes it up detecting the system Python framework when using
    # Python3.
    # We don't pass the Python executable from $PYTHON because we don't want
    # the original one, but that from the virtual enviroment where build
    # dependencies have been installed. Otherwise, CMake won't find numpy.
    # For HDF5, HDF5_ROOT is not enough to find h5cc, PATH is modified instead.
    # For boost, the only way to get it properly detected seems to be passing
    # Boost_INCLUDE_DIR and Boost_LIBRARY_DIR to the cmake command
    export HDF5_ROOT=$hdf5_prefix
    export PATH=$PATH:$hdf5_prefix/bin
    cmake $BASE  \
      -DBoost_INCLUDE_DIR=$boost_prefix/include                       \
      -DBoost_LIBRARY_DIR=$boost_prefix/lib                           \
      -DCMAKE_BUILD_TYPE=Release                                      \
      -DCLONE_SUBPROJECTS=ON                                          \
      -DUSE_PYTHON_VERSION=$(get_python_major_version)                \
      -DBoost_USE_STATIC_LIBS=ON                                      \
      -DHDF5_USE_STATIC_LIBRARIES=ON                                  \
      -DPYTHON_EXECUTABLE:FILEPATH=$(which python)                    \
      -DPYTHON_LIBRARY=$(get_python_library)                          \
      -DPYTHON_INCLUDE_DIR=$(get_python_include)                      \
      -DCOMMON_LIBRARY_TYPE=STATIC || { popd; return 1; }

    make -j2 brain_python || { popd; return 1; }
    popd
}

function build_brion_wheel
{
    setup_venv . || return 1
    . bin/activate || return 1

    build_brion || { deactivate; return 1; }

    cp build/packaging/python_wheels/setup.py build/lib
    pushd build/lib
    # Must use the python version from the virtualenv, which is the one
    # containing wheel
    python setup.py bdist_wheel -bdist-dir=/home
    popd
    mv build/lib/dist/brain*.whl .
    deactivate
    rm -r build
}

function test_wheel
{
    create_venv test
    . test/bin/activate

    pip install brain*.whl
    python -c "import brain; brain.Circuit;" || { deactivate; return 1; }

    deactivate

    # Once tested, the wheel is moved to wheelhouse
    mv brain*.whl ../wheelhouse
}

function build_wheel
{
    version=$1

    mkdir $version 2>/dev/null
    pushd $version

    pyenv global $version
    PYTHON=$(pyenv which python)

    echo -e "\nBuilding for Python $version: $PYTHON\n"

    build_boost_python

    build_brion_wheel || { popd; return 1; }
    test_wheel || { popd; return 1; }

    popd
}

build_boost
build_hdf5

install_pyenv
install_python_versions

mkdir -p wheelhouse
rm wheelhouse/*whl 2>/dev/null

for version in $PYTHON_VERSIONS; do
    build_wheel $version
done
