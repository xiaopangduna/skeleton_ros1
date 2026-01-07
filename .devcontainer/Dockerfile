FROM osrf/ros:noetic-desktop-full

# 避免在非交互模式下出现tzdata错误
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    build-essential \
    gdb 

RUN rm -rf /var/lib/apt/lists/*

