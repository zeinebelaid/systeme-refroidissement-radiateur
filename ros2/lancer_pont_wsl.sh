#!/usr/bin/env bash
# A lancer DANS WSL, apres avoir source ROS 2 (ex. source /opt/ros/humble/setup.bash).
# Webots tourne sous Windows. Usage :  ./lancer_pont_wsl.sh [ip_windows]
#   - sans argument : IP de Windows detectee (mode WSL2 "NAT" par defaut)
#   - ./lancer_pont_wsl.sh 127.0.0.1 : si tu as active le reseau "mirrored" dans .wslconfig
set -e
python3 -c "import rclpy" 2>/dev/null || { echo "rclpy introuvable : fais d'abord 'source /opt/ros/<distro>/setup.bash'"; exit 1; }
HOTE="${1:-$(ip route show default | awk '{print $3}')}"
echo "Webots (Windows) = $HOTE:5005   |   ecoute MATLAB sur UDP 5006 (WSL)"
exec python3 "$(dirname "$0")/ros2_webots_bridge.py" --ros-args -p host:="$HOTE" -p udp_in:=5006
