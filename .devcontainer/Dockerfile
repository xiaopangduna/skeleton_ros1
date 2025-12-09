FROM osrf/ros:noetic-desktop-full

# 避免在非交互模式下出现tzdata错误
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    git

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential    

RUN rm -rf /var/lib/apt/lists/*
#     build-essential \
#     cmake \
#     python3-pip \
#     vim \
#     net-tools \
#     iputils-ping \
#     && rm -rf /var/lib/apt/lists/*

