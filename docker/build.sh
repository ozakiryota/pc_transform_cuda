#!/bin/bash

image="pc_transform_cuda"
tag="latest"

docker build . \
    -t $image:$tag \
    --build-arg CACHEBUST=$(date +%s)