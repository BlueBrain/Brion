FROM quay.io/pypa/manylinux1_x86_64@sha256:b03e30ce7f306d711c6e60bf6cc3313fee40bfed84d2774deeb608fabc410031
RUN yum install -y zlib-devel bzip2-devel flex

############
# CMake
############

# Need to build this from source, since the binaries from Kitware don't
# run with the old version of the libraries in this Centos version
RUN curl -OL https://cmake.org/files/v3.7/cmake-3.7.2.tar.gz && tar xfz cmake-3.7.2.tar.gz && cd cmake-3.7.2 && ./configure && make -j2 install
RUN rm -rf cmake-3.7.2.tar.gz cmake-3.7.2


############
# HDF5
############

RUN curl -OL https://support.hdfgroup.org/ftp/HDF5/releases/hdf5-1.8/hdf5-1.8.17/src/hdf5-1.8.17.tar.bz2 && tar xfj hdf5-1.8.17.tar.bz2
RUN cd hdf5-1.8.17 && ./configure CFLAGS=-fPIC CXXFLAGS=-fPIC --prefix=/usr/local --with-pic --disable-shared --with-zlib=/usr/include/,/usr/lib64/ && make -j2 install

# --disable-shared doesn't add libz to the archive, so it
# must be done manually at the end, there aren't any collisions in
# the .a file, so currently safe
RUN cd hdf5-1.8.17 && ar x /usr/lib64/libz.a && ar r /usr/local/lib/libhdf5.a *.o
RUN rm -rf hdf5-1.8.17.tar.bz2 hdf5-1.8.17


############
# Boost
############

RUN curl -OL https://downloads.sourceforge.net/project/boost/boost/1.59.0/boost_1_59_0.tar.gz && tar xfz boost_1_59_0.tar.gz
RUN cd boost_1_59_0 && sed -i -e "s/_PyUnicode_AsString/(void*)_PyUnicode_AsString/" libs/python/src/converter/builtin_converters.cpp
RUN cd boost_1_59_0 && ./bootstrap.sh --with-libraries=date_time,iostreams,filesystem,program_options,regex,serialization,system,test && ./b2 -s NO_BZIP2=1 -j2 -q cxxflags=-fPIC cflags=-fPIC threading=multi link=static --build-type=minimal install
RUN cd boost_1_59_0 && ./b2 --clean
RUN rm -rf boost_1_59_0.tar.gz # don't delete build folder, needed to build boost.python for different python versions on-the-fly

############
# Doxygen
############

# Needed for docstrings generation and system doxygen is too old
# after 1.8.6, we can't use anymore system flex, see https://stackoverflow.com/questions/20844635

RUN curl -OL https://github.com/doxygen/doxygen/archive/Release_1_8_5.tar.gz && tar xfz Release_1_8_5.tar.gz && cd doxygen-Release_1_8_5 && ./configure && make -j2 && make install
RUN rm -rf Release_1_8_5 doxygen-Release_1_8_5

