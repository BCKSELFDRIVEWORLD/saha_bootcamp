# Saha Robotic Bootcamp - ROS2 Humble + TurtleBot3

Bu proje, ROS2 Humble ve TurtleBot3 simülasyonunu Docker ile calistirmani saglar. Bilgisayarina ROS2 kurmana gerek yok, her sey Docker icinde hazir gelir.

**GPU otomatik tespit edilir.** NVIDIA veya AMD/Intel farketmez, sistem kendisi anlar.

Docker Image: **saharobotic_bootcamp**

---

## Docker Nedir?

Docker, uygulamalari izole bir kutunun icinde calistiran bir aractir. Bu sayede bilgisayarina karmasik kurulumlar yapmadan, hazir bir ortami indirip kullanabilirsin. Herkes ayni ortamda calisir, "bende calisiyor sende calismiyor" sorunu olmaz.

---

## 1. Docker Kurulumu

Terminali ac ve sirasiyla su komutlari calistir:

```bash
sudo apt update
sudo apt install -y ca-certificates curl gnupg

sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin

# Docker'i sudo'suz kullanmak icin
sudo usermod -aG docker $USER
```

**Kurulumdan sonra bilgisayarini yeniden baslat.**

Dogrulama:
```bash
docker --version
# Docker version 2x.x.x gibi bir cikti gormalisin
```

---

## 2. GPU Ayarlari (Sadece Bir Kez)

Scriptler GPU'nu otomatik tespit eder. Ama once GPU suruculerinin dogru kuruldugundan emin ol.

### NVIDIA Ekran Karti Varsa

```bash
# NVIDIA GPU oldugundan emin ol
nvidia-smi

# nvidia-container-toolkit kur
sudo apt install -y nvidia-container-toolkit
sudo systemctl restart docker
```

### AMD / Intel Ekran Karti Varsa

```bash
sudo usermod -aG video $USER
sudo usermod -aG render $USER
# Bilgisayari yeniden baslat
```

> **Not:** Hangisine sahip oldugundan emin degilsen `nvidia-smi` calistir. Cikti veriyorsa NVIDIA'n var, hata veriyorsa AMD/Intel'dir.

---

## 3. Kurulum (Ilk Seferde Bir Kez)

```bash
cd ~/saha_bootcamp
chmod +x setup.sh start.sh stop.sh detect_gpu.sh
./setup.sh
```

Bu islem ilk seferde **10-20 dakika** surebilir (tum ROS2 paketlerini indirir). GPU otomatik tespit edilir.

---

## 4. Gunluk Kullanim

Sadece 3 komut bilmen yeterli:

```bash
# Container'i baslat ve baglan
./start.sh

# Container'dan cik
exit

# Container'i durdur
./stop.sh
```

`start.sh` her calistiginda GPU'nu otomatik tespit eder, xhost iznini verir ve container'a baglanir.

---

## 5. TurtleBot3 Simulasyonu

### Gazebo Simulasyonunu Baslat

Container icinde:
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Robotu Klavyeyle Kontrol Et

**Yeni bir terminal** ac ve container'a baglan:
```bash
docker exec -it turtlebot3_ros2 bash
ros2 run turtlebot3_teleop teleop_keyboard
```

| Tus | Hareket |
|-----|---------|
| `w` | Ileri   |
| `x` | Geri    |
| `a` | Sol     |
| `d` | Sag     |
| `s` | Dur     |

### SLAM ile Harita Olustur

**Yeni bir terminal** ac, container'a baglan:
```bash
docker exec -it turtlebot3_ros2 bash
ros2 launch slam_toolbox online_async_launch.py
```

### RViz ile Haritayi Goruntule

**Yeni bir terminal** ac, container'a baglan:
```bash
docker exec -it turtlebot3_ros2 bash
ros2 launch nav2_bringup rviz_launch.py
```

### Haritayi Kaydet

```bash
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/src/my_packages/my_map
```

Harita `workspace/` klasorune kaydedilir (bilgisayarindan da erisebilirsin).

---

## 6. Kendi Paketini Gelistirme

`workspace/` klasoru container ile paylasimli. Bilgisayarinda bu klasore koydugun dosyalar container icinde `/ros2_ws/src/my_packages/` yolunda gorunur.

```bash
# Container icinde yeni paket olustur
cd /ros2_ws/src/my_packages
ros2 pkg create --build-type ament_python my_robot_pkg

# Derle
cd /ros2_ws
colcon build
source install/setup.bash
```

---

## 7. Sik Karsilasilan Sorunlar

### Gazebo veya RViz acilmiyor

```bash
# Container icinde GPU test et
glxinfo | grep "OpenGL renderer"
# GPU adinizi gormalisiniz
```

Goremiyorsan container'i yeniden baslat:
```bash
./stop.sh
./start.sh
```

### "Permission denied" hatasi

```bash
sudo usermod -aG docker $USER
# Terminali kapat ve yeniden ac
```

### NVIDIA: "could not select device driver" hatasi

```bash
sudo apt install -y nvidia-container-toolkit
sudo systemctl restart docker
```

### AMD: "Cannot open /dev/dri" veya "drmGetDevice2 failed" hatasi

```bash
sudo usermod -aG video $USER
sudo usermod -aG render $USER
# Bilgisayari yeniden baslat
```

### Container'i sifirlamak istiyorum

```bash
./stop.sh
docker compose -f docker-compose.nvidia.yml build --no-cache  # veya docker-compose.amd.yml
./start.sh
```

### Disk alani doldu

```bash
docker system prune -a
```

---

## 8. Faydali ROS2 Komutlari

```bash
# Aktif topic'leri listele
ros2 topic list

# Bir topic'i dinle
ros2 topic echo /scan

# Aktif node'lari listele
ros2 node list

# Node bilgisi
ros2 node info /turtlebot3_ros
```

---

## Dosya Yapisi

```
saha_bootcamp/
├── Dockerfile                  # Docker imaj tarifi
├── docker-compose.nvidia.yml   # NVIDIA GPU ayarlari (otomatik secilir)
├── docker-compose.amd.yml      # AMD/Intel GPU ayarlari (otomatik secilir)
├── detect_gpu.sh               # GPU tespit scripti
├── ros_entrypoint.sh           # Container baslangic scripti
├── setup.sh                    # Ilk kurulum (bir kez calistir)
├── start.sh                    # Baslat ve baglan (her gun)
├── stop.sh                     # Durdur
├── workspace/                  # Senin paketlerin (container ile paylasimli)
└── README.md                   # Bu dosya
```
