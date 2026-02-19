#!/bin/bash
# Saha Robotic Bootcamp - Docker Setup Script

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPT_DIR/detect_gpu.sh"

echo "============================================"
echo "  Saha Robotic Bootcamp - Docker Kurulum"
echo "============================================"
echo ""

# Check if docker is installed
if ! command -v docker &> /dev/null; then
    echo "[HATA] Docker kurulu degil!"
    echo "README.md dosyasindaki Docker kurulum adimlarini takip edin."
    exit 1
fi

# Allow GUI applications
xhost +local:docker &>/dev/null

# Detect GPU and select compose file
COMPOSE_FILE=$(detect_compose_file)

if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
    echo "[GPU] NVIDIA GPU tespit edildi."

    if ! dpkg -l 2>/dev/null | grep -q nvidia-container-toolkit; then
        echo "[UYARI] nvidia-container-toolkit kurulu degil!"
        echo "Kurmak icin:"
        echo "  sudo apt install -y nvidia-container-toolkit"
        echo "  sudo systemctl restart docker"
        echo ""
        echo "[INFO] nvidia-container-toolkit olmadan AMD/Intel modu kullanilacak."
        echo ""
    else
        echo "[INFO] nvidia-container-toolkit mevcut."
    fi
else
    echo "[GPU] AMD/Intel GPU tespit edildi."
fi

echo "[INFO] Compose: $COMPOSE_FILE"
echo ""
echo "[BUILD] Docker imaji olusturuluyor (ilk seferde 10-20 dk surebilir)..."
echo ""

docker compose -f "$SCRIPT_DIR/$COMPOSE_FILE" build

if [ $? -eq 0 ]; then
    echo ""
    echo "============================================"
    echo "  Kurulum tamamlandi!"
    echo "============================================"
    echo ""
    echo "Baslatmak icin:  ./start.sh"
    echo "Durdurmak icin:  ./stop.sh"
    echo ""
else
    echo ""
    echo "[HATA] Docker build basarisiz oldu. Hatalari kontrol edin."
    exit 1
fi
