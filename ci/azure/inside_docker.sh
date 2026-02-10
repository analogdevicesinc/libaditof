#!/bin/bash
git config --global --add safe.directory /libaditof
#git config --global --add safe.directory /libaditof/dependencies/third-party/protobuf
#git config --global --add safe.directory /libaditof/dependencies/third-party/benchmark
#git config --global --add safe.directory /libaditof/dependencies/third-party/googletest
#git config --global --add safe.directory /libaditof/dependencies/third-party/libzmq
#git config --global --add safe.directory /libaditof/dependencies/third-party/cppzmq
#git config --global --add safe.directory /libaditof/dependencies/third-party/gtest

project_dir=$1
pushd ${project_dir}

NUM_JOBS=4
ARGS="$2"

mkdir -p build
mkdir ../libs

pushd build
cmake .. ${ARGS}
make -j4
popd #build

popd # ${project_dir}
