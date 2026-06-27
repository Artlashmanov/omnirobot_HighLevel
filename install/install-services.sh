#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_OWNER="$(stat -c '%U' "${PROJECT_DIR}")"
OMNI_USER_VALUE="${OMNI_USER:-${SUDO_USER:-${PROJECT_OWNER}}}"
if [[ "${OMNI_USER_VALUE}" == "root" && "${PROJECT_OWNER}" != "root" ]]; then
  OMNI_USER_VALUE="${PROJECT_OWNER}"
fi

if [[ "${EUID}" -ne 0 ]]; then
  echo "Please run through sudo: sudo ./install/install-services.sh" >&2
  exit 1
fi

ensure_env_key() {
  local key="$1"
  local value="$2"
  grep -q "^${key}=" /etc/omni-robot/omni.env || printf '%s=%s\n' "${key}" "${value}" >> /etc/omni-robot/omni.env
}

install -d -m 0755 /etc/omni-robot
if [[ ! -f /etc/omni-robot/omni.env ]]; then
  sed \
    -e "s|^OMNI_HOME=.*|OMNI_HOME=${PROJECT_DIR}|" \
    -e "s|^OMNI_USER=.*|OMNI_USER=${OMNI_USER_VALUE}|" \
    "${PROJECT_DIR}/config/omni.env.example" > /etc/omni-robot/omni.env
  chmod 0644 /etc/omni-robot/omni.env
else
  cp -a /etc/omni-robot/omni.env "/etc/omni-robot/omni.env.bak.$(date +%Y%m%d-%H%M%S)"
  sed -i \
    -e "s|^OMNI_HOME=.*|OMNI_HOME=${PROJECT_DIR}|" \
    -e "s|^OMNI_USER=.*|OMNI_USER=${OMNI_USER_VALUE}|" \
    /etc/omni-robot/omni.env
  grep -q '^OMNI_HOME=' /etc/omni-robot/omni.env || echo "OMNI_HOME=${PROJECT_DIR}" >> /etc/omni-robot/omni.env
  grep -q '^OMNI_USER=' /etc/omni-robot/omni.env || echo "OMNI_USER=${OMNI_USER_VALUE}" >> /etc/omni-robot/omni.env
fi

ensure_env_key "ROBOT_PLATFORM" "omni4"
ensure_env_key "OMNI_PLATFORM_CONFIG" '${OMNI_HOME}/config/platforms/omni4.json'
ensure_env_key "CAN_IFACE" "can0"
ensure_env_key "CAN_BITRATE" "500000"
ensure_env_key "TELEOP_HOST" "0.0.0.0"
ensure_env_key "TELEOP_PORT" "8080"
ensure_env_key "OMNI_FETCH_ROS_DEPS" "1"
ensure_env_key "OMNI_ENABLE_LIDAR" "1"
ensure_env_key "OMNI_ROS_REPOS_FILE" '${OMNI_HOME}/src/ros2_ws/omni.repos'
ensure_env_key "OMNI_ODOM_PARAMS" '${OMNI_HOME}/src/ros2_ws/src/omni_bridge/config/omni_bridge.params.yaml'
ensure_env_key "OMNI_ENABLE_TF_LUNA" "1"
ensure_env_key "TF_LUNA_PARAMS" '${OMNI_HOME}/src/ros2_ws/src/omni_bridge/config/omni_bridge.params.yaml'
ensure_env_key "TF_LUNA_SERIAL_PORT" "/dev/ttyAMA0"
ensure_env_key "TF_LUNA_BAUDRATE" "115200"
ensure_env_key "TF_LUNA_FRAME_ID" "tf_luna_front"
ensure_env_key "TF_LUNA_RANGE_TOPIC" "/range/front"
ensure_env_key "TF_LUNA_STATUS_TOPIC" "/sensors/tf_luna/front"
ensure_env_key "LIDAR_MODEL" "rplidar_c1"
ensure_env_key "LIDAR_SERIAL_PORT" "/dev/rplidar"
ensure_env_key "LIDAR_FALLBACK_SERIAL_PORT" "/dev/ttyUSB0"
ensure_env_key "LIDAR_SERIAL_BAUDRATE" "460800"
ensure_env_key "LIDAR_FRAME_ID" "laser"
ensure_env_key "LIDAR_WAIT_TIMEOUT_SEC" "20"
ensure_env_key "LIDAR_SCAN_MODE" "Standard"
ensure_env_key "LIDAR_INVERTED" "false"
ensure_env_key "LIDAR_ANGLE_COMPENSATE" "true"
ensure_env_key "LIDAR_USB_SERIAL_SHORT" ""
ensure_env_key "OMNI_ENABLE_SLAM" "1"
ensure_env_key "SLAM_PARAMS" '${OMNI_HOME}/config/slam/slam_toolbox_online_async.yaml'
ensure_env_key "SLAM_USE_SIM_TIME" "false"

SERVICE_TMP_DIR="$(mktemp -d)"
trap 'rm -rf "${SERVICE_TMP_DIR}"' EXIT

for service_file in "${PROJECT_DIR}"/services/*.service; do
  service_name="$(basename "${service_file}")"
  sed \
    -e "s|^User=.*|User=${OMNI_USER_VALUE}|" \
    -e "s|/home/noob/omni-pi|${PROJECT_DIR}|g" \
    -e "s|/home/noob/omnirobot_HighLevel|${PROJECT_DIR}|g" \
    "${service_file}" > "${SERVICE_TMP_DIR}/${service_name}"
done

cp "${SERVICE_TMP_DIR}"/*.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable omni-can.service omni-bridge.service omni-odom.service omni-tfluna.service omni-mux.service teleop-web.service
systemctl enable omni-lidar.service || true
systemctl enable omni-slam.service || true

echo "Services installed. Runtime config: /etc/omni-robot/omni.env"
echo "Services run as user: ${OMNI_USER_VALUE}"
echo "Start core stack with: sudo systemctl start omni-can omni-bridge omni-odom omni-tfluna omni-mux teleop-web"
echo "Start mapping stack with: sudo systemctl start omni-lidar omni-slam"
