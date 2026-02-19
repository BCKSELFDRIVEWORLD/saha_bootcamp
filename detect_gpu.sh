#!/bin/bash
# GPU detection and group ID resolution
# Usage: source detect_gpu.sh

detect_compose_file() {
    if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
        if dpkg -l 2>/dev/null | grep -q nvidia-container-toolkit; then
            echo "docker-compose.nvidia.yml"
            return 0
        fi
    fi

    if [ -d /dev/dri ]; then
        echo "docker-compose.amd.yml"
        return 0
    fi

    echo "docker-compose.amd.yml"
    return 1
}

# Get host video/render group IDs and export them
export VIDEO_GID=$(getent group video 2>/dev/null | cut -d: -f3)
export RENDER_GID=$(getent group render 2>/dev/null | cut -d: -f3)
VIDEO_GID=${VIDEO_GID:-44}
RENDER_GID=${RENDER_GID:-110}
