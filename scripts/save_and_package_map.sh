#!/usr/bin/env bash
# ==============================================================================
# AMR ROS 2 Humble - Industrial Map Saver & .zip Bundler for Map Studio
# Usage:
#   ./save_map.sh [nama_map]
#   Default: ./save_map.sh factory_map_baru
#
#   Atau bungkus ulang map yang sudah ada menjadi .zip:
#   ./save_map.sh --package-existing map_labv2_save
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
MAPS_DIR="${WS_ROOT}/maps"

mkdir -p "${MAPS_DIR}"

if [[ "$1" == "--package-existing" ]]; then
    EXISTING_PREFIX="${2:-map_labv2_save}"
    OUT_NAME="${EXISTING_PREFIX%_save}"
    echo "============================================================"
    echo " [AMR Map Packager] Mengemas peta existing: ${EXISTING_PREFIX}"
    echo "============================================================"

    YAML_FILE=""
    PGM_FILE=""
    if [[ -f "${WS_ROOT}/${EXISTING_PREFIX}.yaml" ]]; then
        YAML_FILE="${WS_ROOT}/${EXISTING_PREFIX}.yaml"
        PGM_FILE="${WS_ROOT}/${EXISTING_PREFIX}.pgm"
    elif [[ -f "${MAPS_DIR}/${EXISTING_PREFIX}.yaml" ]]; then
        YAML_FILE="${MAPS_DIR}/${EXISTING_PREFIX}.yaml"
        PGM_FILE="${MAPS_DIR}/${EXISTING_PREFIX}.pgm"
    else
        echo "[ERROR] File ${EXISTING_PREFIX}.yaml tidak ditemukan di ${WS_ROOT} maupun ${MAPS_DIR}."
        exit 1
    fi

    ZIP_OUT="${MAPS_DIR}/${OUT_NAME}.zip"
    rm -f "${ZIP_OUT}"

    FILES_TO_ZIP=("${YAML_FILE}" "${PGM_FILE}")

    # Sertakan posegraph & data jika tersedia
    SERIAL_PREFIX="${EXISTING_PREFIX%_save}_serial"
    [[ -f "${WS_ROOT}/${SERIAL_PREFIX}.posegraph" ]] && FILES_TO_ZIP+=("${WS_ROOT}/${SERIAL_PREFIX}.posegraph")
    [[ -f "${WS_ROOT}/${SERIAL_PREFIX}.data" ]] && FILES_TO_ZIP+=("${WS_ROOT}/${SERIAL_PREFIX}.data")

    # Sertakan waypoints.yaml jika tersedia di agv_hmi
    [[ -f "${WS_ROOT}/src/agv_hmi/config/waypoints.yaml" ]] && FILES_TO_ZIP+=("${WS_ROOT}/src/agv_hmi/config/waypoints.yaml")

    python3 -c '
import sys, zipfile, os
zip_path = sys.argv[1]
files = sys.argv[2:]
with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
    for f in files:
        if os.path.exists(f):
            zf.write(f, os.path.basename(f))
' "${ZIP_OUT}" "${FILES_TO_ZIP[@]}"

    echo "[SUCCESS] Paket .zip siap diunggah ke Map Studio:"
    echo "  -> ${ZIP_OUT}"
    exit 0
fi

MAP_NAME="${1:-factory_map_baru}"
TARGET_PATH="${MAPS_DIR}/${MAP_NAME}"
ZIP_PATH="${MAPS_DIR}/${MAP_NAME}.zip"

echo "============================================================"
echo " [AMR Map Saver] Menyimpan Peta ROS 2 Nav2: ${MAP_NAME}"
echo " Workspace : ${WS_ROOT}"
echo " Output    : ${TARGET_PATH}.pgm & ${TARGET_PATH}.yaml"
echo "============================================================"

# 1. Source ROS 2 Humble & Workspace
if [[ -f "/opt/ros/humble/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "/opt/ros/humble/setup.bash"
fi

if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${WS_ROOT}/install/setup.bash"
fi

# 2. Jalankan perintah standar industri Nav2 Map Saver CLI
cd "${WS_ROOT}"
echo "[1/3] Menjalankan: ros2 run nav2_map_server map_saver_cli -f maps/${MAP_NAME}"
if ! ros2 run nav2_map_server map_saver_cli -f "maps/${MAP_NAME}"; then
    echo "[INFO] Mencoba ulang dengan transient_local QoS..."
    ros2 run nav2_map_server map_saver_cli -f "maps/${MAP_NAME}" --ros-args -p map_subscribe_transient_local:=true
fi

# 3. Opsional: Simpan posegraph SLAM Toolbox jika node slam_toolbox sedang aktif
if ros2 service list 2>/dev/null | grep -q "/slam_toolbox/serialize_map"; then
    echo "[2/3] Menyimpan posegraph SLAM Toolbox (${MAP_NAME}_serial)..."
    ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '${TARGET_PATH}_serial'}" || true
fi

# 4. Buat paket .zip standar industri untuk di-upload ke Map Studio Web
echo "[3/3] Mengemas file peta ke dalam arsip .zip..."
rm -f "${ZIP_PATH}"

FILES_TO_ZIP=("${TARGET_PATH}.yaml" "${TARGET_PATH}.pgm")
[[ -f "${TARGET_PATH}_serial.posegraph" ]] && FILES_TO_ZIP+=("${TARGET_PATH}_serial.posegraph")
[[ -f "${TARGET_PATH}_serial.data" ]] && FILES_TO_ZIP+=("${TARGET_PATH}_serial.data")
[[ -f "${MAPS_DIR}/keepout_zones.png" ]] && FILES_TO_ZIP+=("${MAPS_DIR}/keepout_zones.png")
[[ -f "${WS_ROOT}/src/agv_hmi/config/waypoints.yaml" ]] && FILES_TO_ZIP+=("${WS_ROOT}/src/agv_hmi/config/waypoints.yaml")

python3 -c '
import sys, zipfile, os
zip_path = sys.argv[1]
files = sys.argv[2:]
with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
    for f in files:
        if os.path.exists(f):
            zf.write(f, os.path.basename(f))
' "${ZIP_PATH}" "${FILES_TO_ZIP[@]}"

echo "============================================================"
echo " [SELESAI] Peta berhasil disimpan dan dikemas!"
echo " - YAML : ${TARGET_PATH}.yaml"
echo " - PGM  : ${TARGET_PATH}.pgm"
echo " - ZIP  : ${ZIP_PATH}"
echo " Silakan pilih file ${ZIP_PATH} pada menu Map Studio."
echo "============================================================"
