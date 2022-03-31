#!/bin/bash

image="pc_transform_cuda"
tag="latest"

docker build -t $image:$tag .