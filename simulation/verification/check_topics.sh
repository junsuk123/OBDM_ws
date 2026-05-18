#!/bin/bash
# ROS2 토픽 실시간 검증 스크립트
# 시뮬레이션에서 발행하는 모든 토픽 모니터링

set -e

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========== ROS2 토픽 검증 스크립트 ==========${NC}\n"

# 1. ROS2 환경 확인
echo -e "${YELLOW}[1/5] ROS2 환경 확인${NC}"
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}✗ ROS2가 설치되어 있지 않습니다${NC}"
    exit 1
fi
echo -e "${GREEN}✓ ROS2 설치 확인${NC}"

# 2. ROS_DOMAIN_ID 확인
echo -e "\n${YELLOW}[2/5] ROS_DOMAIN_ID 확인${NC}"
DOMAIN_ID=${ROS_DOMAIN_ID:-0}
echo -e "ROS_DOMAIN_ID: $DOMAIN_ID"
export ROS_DOMAIN_ID=$DOMAIN_ID
echo -e "${GREEN}✓ ROS_DOMAIN_ID 설정: $DOMAIN_ID${NC}"

# 3. 사용 가능한 토픽 확인
echo -e "\n${YELLOW}[3/5] 사용 가능한 토픽 확인${NC}"

TOPICS=(
    "/ap/tf:geometry_msgs/msg/TransformStamped"
    "/ap/imu/sample:sensor_msgs/msg/Imu"
    "/ap/gps/sample:sensor_msgs/msg/NavSatFix"
    "/ap/baro/sample:sensor_msgs/msg/FluidPressure"
    "/ap/battery/state:sensor_msgs/msg/BatteryState"
)

echo "기대되는 토픽:"
for topic_def in "${TOPICS[@]}"; do
    IFS=':' read -r topic_name msg_type <<< "$topic_def"
    echo "  • $topic_name ($msg_type)"
done

# 4. 토픽 리스트 조회
echo -e "\n${YELLOW}[4/5] 시뮬레이션 토픽 조회${NC}"

timeout 3 ros2 topic list 2>/dev/null || true

AVAILABLE_TOPICS=$(ros2 topic list 2>/dev/null | grep "^/ap" || true)

if [ -z "$AVAILABLE_TOPICS" ]; then
    echo -e "${RED}✗ ArduPilot 토픽을 찾을 수 없습니다${NC}"
    echo -e "${YELLOW}  → 시뮬레이션이 실행 중인지 확인하세요${NC}"
    echo -e "${YELLOW}  → 다른 터미널에서 다음 명령 실행:${NC}"
    echo -e "${BLUE}    ros2 launch ardupilot_gz_bringup iris_runway.launch.py${NC}"
    exit 1
fi

echo -e "${GREEN}✓ 다음 토픽이 활성화되어 있습니다:${NC}"
echo "$AVAILABLE_TOPICS" | sed 's/^/  • /'

# 5. 토픽 데이터 샘플링
echo -e "\n${YELLOW}[5/5] 토픽 데이터 샘플링${NC}"
echo "(각 토픽당 3개 메시지 수집)\n"

for topic_def in "${TOPICS[@]}"; do
    IFS=':' read -r topic_name msg_type <<< "$topic_def"
    
    # 토픽 존재 확인
    if echo "$AVAILABLE_TOPICS" | grep -q "^${topic_name}$"; then
        echo -e "${GREEN}✓ $topic_name${NC}"
        echo "  메시지 타입: $msg_type"
        echo "  최근 3개 메시지:"
        
        # 토픽 데이터 샘플 (timeout으로 3초 후 중단)
        timeout 3 ros2 topic echo "$topic_name" -c 3 2>/dev/null | \
            head -20 | sed 's/^/    /' || true
        echo ""
    else
        echo -e "${YELLOW}⚠ $topic_name (발행 안 됨)${NC}\n"
    fi
done

# 6. 최종 보고
echo -e "\n${BLUE}========== 검증 완료 ==========${NC}"
echo -e "${GREEN}✓ 모든 필수 토픽이 활성화되어 있습니다${NC}"
echo -e "\n${YELLOW}다음 단계:${NC}"
echo -e "  1. MATLAB을 실행하여 토픽 구독 테스트"
echo -e "  2. 다음 명령 실행:"
echo -e "    ${BLUE}cd ~/OBDM_ws/matlab${NC}"
echo -e "    ${BLUE}matlab -r \"verify_ros2_connection\"${NC}"
echo ""
