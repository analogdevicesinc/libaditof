#!/bin/bash

# Build the given dockerfile with the given name

dockername=$1
dockerfilepath=$2

docker build -t ${dockername} -f ${dockerfilepath} .

rm -rf temp_deps
