#!/bin/bash
# Saha Robotic Bootcamp - Container Stop Script

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPT_DIR/detect_gpu.sh"

COMPOSE_FILE=$(detect_compose_file)

docker compose -f "$SCRIPT_DIR/$COMPOSE_FILE" down

echo "[OK] Container durduruldu."
