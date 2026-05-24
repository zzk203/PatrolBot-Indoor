#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo ">>> 构建 PatrolBot 开发镜像..."
docker build -t patrolbot:latest -f "$SCRIPT_DIR/Dockerfile" "$PROJECT_DIR"

echo ">>> 启动容器..."
if docker ps -q -f name=patrolbot_dev | grep -q .; then
    docker exec -it patrolbot_dev bash
else
    docker run -it --rm --name patrolbot_dev \
        --gpus all \
        -e DISPLAY="$DISPLAY" \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v "$PROJECT_DIR/patrol_robot_ws:/root/patrol_robot_ws" \
        --network host \
        patrolbot:latest bash
fi
