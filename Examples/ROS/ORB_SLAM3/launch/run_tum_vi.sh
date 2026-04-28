#!/usr/bin/env bash

set -euo pipefail

# ORB-SLAM3 工程根目录
ROOT_DIR="/home/sherpinck_holomz/ORB_SLAM3"

# ORB 词袋文件路径
VOC_FILE="/home/sherpinck_holomz/ORB_SLAM3/Vocabulary/ORBvoc.txt"

# TUM-VI 双目惯性配置文件
SETTINGS_FILE="/home/sherpinck_holomz/ORB_SLAM3/Examples/Stereo-Inertial/TUM-VI.yaml"

# 可执行程序路径
EXECUTABLE="/home/sherpinck_holomz/ORB_SLAM3/Examples/Stereo-Inertial/stereo_inertial_tum_vi"

# TUM-VI dataset-room1_512_16 数据路径
# LEFT_IMAGE_DIR="/home/sherpinck_holomz/ORB_SLAM3/Dataset/TUM-VI/dataset-room1_512_16/mav0/cam0/data"
# RIGHT_IMAGE_DIR="/home/sherpinck_holomz/ORB_SLAM3/Dataset/TUM-VI/dataset-room1_512_16/mav0/cam1/data"
# TIMESTAMPS_FILE="/home/sherpinck_holomz/ORB_SLAM3/Examples/Stereo-Inertial/TUM_TimeStamps/dataset-room1_512.txt"
# IMU_FILE="/home/sherpinck_holomz/ORB_SLAM3/Dataset/TUM-VI/dataset-room1_512_16/mav0/imu0/data.csv"

# TUM-VI dataset-outdoors4_512_16 数据路径
LEFT_IMAGE_DIR="/home/sherpinck_holomz/ORB_SLAM3/Dataset/TUM-VI/dataset-outdoors4_512_16/mav0/cam0/data"
RIGHT_IMAGE_DIR="/home/sherpinck_holomz/ORB_SLAM3/Dataset/TUM-VI/dataset-outdoors4_512_16/mav0/cam1/data"
TIMESTAMPS_FILE="/home/sherpinck_holomz/ORB_SLAM3/Examples/Stereo-Inertial/TUM_TimeStamps/dataset-outdoors4_512.txt"
IMU_FILE="/home/sherpinck_holomz/ORB_SLAM3/Dataset/TUM-VI/dataset-outdoors4_512_16/mav0/imu0/data.csv"

# 可选输出前缀。留空则程序默认输出 CameraTrajectory.txt / KeyFrameTrajectory.txt
# OUTPUT_NAME="room1_512_16"
OUTPUT_NAME="outdoor4_512_16"

cd "${ROOT_DIR}"

echo "Running ORB-SLAM3 stereo inertial on TUM-VI dataset-outdoors4_512_16"
echo "Vocabulary: ${VOC_FILE}"
echo "Settings:   ${SETTINGS_FILE}"
echo "Left cam:   ${LEFT_IMAGE_DIR}"
echo "Right cam:  ${RIGHT_IMAGE_DIR}"
echo "Times:      ${TIMESTAMPS_FILE}"
echo "IMU:        ${IMU_FILE}"

if [[ -n "${OUTPUT_NAME}" ]]; then
  "${EXECUTABLE}" \
    "${VOC_FILE}" \
    "${SETTINGS_FILE}" \
    "${LEFT_IMAGE_DIR}" \
    "${RIGHT_IMAGE_DIR}" \
    "${TIMESTAMPS_FILE}" \
    "${IMU_FILE}" \
    "${OUTPUT_NAME}"
else
  "${EXECUTABLE}" \
    "${VOC_FILE}" \
    "${SETTINGS_FILE}" \
    "${LEFT_IMAGE_DIR}" \
    "${RIGHT_IMAGE_DIR}" \
    "${TIMESTAMPS_FILE}" \
    "${IMU_FILE}"
fi
