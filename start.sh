#!/bin/bash
# Saha Robotic Bootcamp - Container Start Script

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPT_DIR/detect_gpu.sh"

# Allow GUI applications
xhost +local:docker &>/dev/null

# Auto-detect GPU and pick the right compose file
COMPOSE_FILE=$(detect_compose_file)

echo "[GPU] $COMPOSE_FILE kullaniliyor."

# Stop existing container if running
docker compose -f "$SCRIPT_DIR/$COMPOSE_FILE" down &>/dev/null

# Start container
docker compose -f "$SCRIPT_DIR/$COMPOSE_FILE" up -d

if [ $? -eq 0 ]; then
    echo "[OK] Container basladi."
    echo ""
    docker exec -it turtlebot3_ros2 bash
else
    echo "[HATA] Container baslatilamadi."
    echo "Once ./setup.sh calistirdiginizdan emin olun."
    exit 1
fi
