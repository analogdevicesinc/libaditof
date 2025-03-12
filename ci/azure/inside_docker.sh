#!/bin/bash
git config --global --add safe.directory /libaditof
git config --global --add safe.directory /libaditof/glog
git config --global --add safe.directory /libaditof/libwebsockets
git config --global --add safe.directory /libaditof/protobuf
git config --global --add safe.directory /libaditof/cppzmq
git config --global --add safe.directory /libaditof/libzmq


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
