FROM ros:noetic

# 避免在非交互模式下出现tzdata错误
ARG DEBIAN_FRONTEND=noninteractive

# 安装常用工具
RUN apt-get update && apt-get install -y \
    git \
    vim \
    wget \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# 设置工作目录
WORKDIR /root/catkin_ws