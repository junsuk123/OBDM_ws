#!/bin/bash
# OBDM 시뮬레이션 실행 스크립트

set -e

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 스크립트 디렉토리
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
ROS2_WORKSPACE="${HOME}/ardu_ws"

echo -e "${GREEN}========== OBDM Gazebo 시뮬레이션 시작 ==========${NC}"
echo "워크스페이스: $WORKSPACE_DIR"
echo "ROS2 워크스페이스: $ROS2_WORKSPACE"
echo ""

# ROS2 설치 확인
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}ERROR: ROS2가 설치되어 있지 않습니다.${NC}"
    echo "ROS2 설치: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

# Gazebo 설치 확인
if ! command -v gz &> /dev/null; then
    echo -e "${RED}ERROR: Gazebo가 설치되어 있지 않습니다.${NC}"
    echo "Gazebo 설치: https://gazebosim.org/docs/harmonic/install"
    exit 1
fi

# ROS2 워크스페이스 존재 확인
if [ ! -d "$ROS2_WORKSPACE" ]; then
    echo -e "${YELLOW}WARNING: ROS2 워크스페이스가 없습니다: $ROS2_WORKSPACE${NC}"
    echo "다음 명령어로 생성하세요:"
    echo "  mkdir -p $ROS2_WORKSPACE/src"
    echo "  cd $ROS2_WORKSPACE"
    echo "  vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src"
    exit 1
fi

# ROS2 환경 변수 설정
export ROS_DOMAIN_ID=0
export GZ_VERSION=harmonic
export ROS_LOCALHOST_ONLY=0

echo -e "${YELLOW}환경 변수:${NC}"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "  GZ_VERSION: $GZ_VERSION"
echo "  ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"
echo ""

# ROS2 워크스페이스 소스
echo -e "${YELLOW}ROS2 워크스페이스 소스 중...${NC}"
source $ROS2_WORKSPACE/install/setup.bash

# 사용 가능한 예제 확인
echo -e "${YELLOW}사용 가능한 시뮬레이션:${NC}"
echo "1. iris_runway      - 멀티콥터 (Iris) 예제"
echo "2. iris_maze        - 미로 환경 (Iris)"
echo "3. wildthumper      - 로버 예제"
echo "4. alti_transition  - Quadplane 예제"
echo "5. multiagent       - 다중 에이전트"
echo ""

# 사용자 입력
read -p "실행할 시뮬레이션 선택 (1-5, 기본값: 1): " CHOICE
CHOICE=${CHOICE:-1}

# 시뮬레이션 선택
case $CHOICE in
    1)
        LAUNCH_FILE="iris_runway.launch.py"
        VEHICLE_TYPE="Copter (Iris)"
        ;;
    2)
        LAUNCH_FILE="iris_maze.launch.py"
        VEHICLE_TYPE="Copter (Iris Maze)"
        ;;
    3)
        LAUNCH_FILE="wildthumper_playpen.launch.py"
        VEHICLE_TYPE="Rover (WildThumper)"
        ;;
    4)
        LAUNCH_FILE="alti_transition_runway.launch.py"
        VEHICLE_TYPE="Quadplane (Alti-Transition)"
        ;;
    5)
        LAUNCH_FILE="launch_multiagent.sh"
        VEHICLE_TYPE="Multi-Agent"
        ;;
    *)
        echo -e "${RED}Invalid choice. Using default (iris_runway).${NC}"
        LAUNCH_FILE="iris_runway.launch.py"
        VEHICLE_TYPE="Copter (Iris)"
        ;;
esac

echo -e "${GREEN}선택: $VEHICLE_TYPE${NC}"
echo "런치 파일: $LAUNCH_FILE"
echo ""

# 시뮬레이션 실행
echo -e "${GREEN}시뮬레이션 시작...${NC}"
echo "Ctrl+C로 종료할 수 있습니다."
echo ""

# If launching the alti transition, use wrapper to normalize args (accept '=' form)
if [ "${LAUNCH_FILE}" = "alti_transition_runway.launch.py" ]; then
    bash "${WORKSPACE_DIR}/simulation/launch/launch_alti_transition.sh" "$@"
else
    ros2 launch ardupilot_gz_bringup $LAUNCH_FILE
fi

echo ""
echo -e "${GREEN}========== 시뮬레이션 종료 ==========${NC}"
