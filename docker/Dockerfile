FROM osrf/ros:humble-desktop

# ====================== 正确国内加速（无报错） ======================
# 1. 替换 Ubuntu 系统源（清华）
RUN sed -i "s@http://archive.ubuntu.com@https://mirrors.tuna.tsinghua.edu.cn@g" /etc/apt/sources.list && \
    sed -i "s@http://security.ubuntu.com@https://mirrors.tuna.tsinghua.edu.cn@g" /etc/apt/sources.list

# 2. 添加 ROS 2 国内清华源（正确方式）
RUN echo "deb https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list && \
    apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-behaviortree-cpp \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /root/patrol_robot_ws
