#!/bin/bash
# Emergency Restart Script - Gazebo Data Collection Fix
# Purpose: Stabilize and restart Gazebo simulation with proper monitoring
# Date: 2026-05-08

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[OK]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

echo ""
echo "╔════════════════════════════════════════════╗"
echo "║  Gazebo Simulation Recovery & Fix         ║"
echo "╚════════════════════════════════════════════╝"
echo ""

# Step 1: Cleanup all processes
log_info "Step 1: Cleaning up all simulation processes..."
cleanup_patterns=(
    'gz sim'
    'gzserver'
    'gzclient'
    'ign gazebo'
    'ros_gz_bridge'
    'parameter_bridge'
    'micro_ros_agent'
    'ros2 launch'
    'arducopter'
    'arduplane'
    'mavproxy'
)
for pattern in "${cleanup_patterns[@]}"; do
    pkill -TERM -f "$pattern" || true
done
sleep 2
for pattern in "${cleanup_patterns[@]}"; do
    pkill -KILL -f "$pattern" || true
done
sleep 2
log_success "Processes cleaned up"
echo ""

# Step 2: Verify environment
log_info "Step 2: Verifying ROS2 environment..."
source /opt/ros/humble/setup.bash || true
WS_DIR="$(dirname "$(realpath "${BASH_SOURCE[0]}")")"
source ${WS_DIR}/ardu_ws_local/install/setup.bash || true
export ROS_DOMAIN_ID=0
export GZ_VERSION=8  # Force Gazebo 8

log_success "ROS2 environment loaded"
echo ""

# Step 3: Start simulation in foreground with monitoring
log_info "Step 3: Starting Gazebo simulation with monitoring..."
log_warning "This will run for 12 seconds to allow initialization and data collection"
echo ""

mkdir -p /tmp/obdm_logs
GAZEBO_LOG="/tmp/obdm_logs/gazebo_$(date +%s).log"
SENSOR_CHECK_LOG="/tmp/obdm_logs/sensor_check_$(date +%s).log"

# Launch Gazebo in background
ros2 launch ardupilot_gz_bringup alti_transition_runway.launch.py \
    use_gz_sim_gui:=false rviz:=false \
    > "$GAZEBO_LOG" 2>&1 &

GZ_LAUNCH_PID=$!
log_success "Gazebo launched (PID: $GZ_LAUNCH_PID)"

# Wait for Gazebo to initialize
log_info "Waiting 8 seconds for Gazebo initialization..."
sleep 8

# Check if Gazebo is still running
if ps -p $GZ_LAUNCH_PID > /dev/null 2>&1; then
    log_success "Gazebo process still running"
else
    log_warning "Gazebo process may have exited"
fi

# Check for active topics
log_info "Checking for active sensor topics..."
echo ""
echo "Available topics:"
ros2 topic list | tee "$SENSOR_CHECK_LOG" | head -20

echo ""
echo "Sensor data topics:"
ros2 topic list | grep -E "imu|odometry|battery" || log_warning "No sensor data topics found"

echo ""

# Step 4: Try MATLAB test
log_info "Step 4: Running MATLAB data collection test..."
echo ""

# Create a simple MATLAB test that adapts to available topics
cat > /tmp/quick_matlab_test.m << 'EOF'
% Quick MATLAB Sensor Test - Adaptive to Available Topics

clear; clc;

setenv('ROS_DOMAIN_ID', '0');
fprintf('Creating ROS2 node...\n');

try
    node = ros2node("/matlab_quick_test");
    fprintf('✓ ROS2 node created\n\n');
catch ME
    fprintf('✗ Failed to create ROS2 node: %s\n', ME.message);
    return;
end

% Get available topics
[status, topic_output] = system('ros2 topic list');
topics = strsplit(strtrim(topic_output), newline);

% Look for sensor topics
fprintf('Scanning for available sensor topics...\n');
sensors = struct();
sensor_count = 0;

imu_topics = {'/ alti_transition/imu', '/alti_transition/imu/experimental/data', '/ap/imu/experimental/data'};
for i = 1:length(imu_topics)
    if any(strcmp(topics, imu_topics{i}))
        sensors(1).name = 'IMU';
        sensors(1).topic = imu_topics{i};
        sensors(1).type = 'sensor_msgs/Imu';
        sensor_count = sensor_count + 1;
        break;
    end
end

odom_topics = {'/alti_transition/odometry', '/alti_transition/pose/filtered', '/ap/pose/filtered'};
for i = 1:length(odom_topics)
    if any(strcmp(topics, odom_topics{i}))
        sensors(sensor_count+1).name = 'ODOM';
        sensors(sensor_count+1).topic = odom_topics{i};
        sensors(sensor_count+1).type = 'nav_msgs/Odometry';
        sensor_count = sensor_count + 1;
        break;
    end
end

battery_topics = {'/alti_transition/battery/state', '/ap/battery/state'};
for i = 1:length(battery_topics)
    if any(strcmp(topics, battery_topics{i}))
        sensors(sensor_count+1).name = 'BATTERY';
        sensors(sensor_count+1).topic = battery_topics{i};
        sensors(sensor_count+1).type = 'sensor_msgs/BatteryState';
        sensor_count = sensor_count + 1;
        break;
    end
end

if sensor_count == 0
    fprintf('✗ No sensor topics found\n');
    return;
else
    fprintf('✓ Found %d sensor topics\n\n', sensor_count);
end

% Subscribe and collect data
fprintf('Subscribing to sensors:\n');
subscribers = {};
for i = 1:sensor_count
    try
        sub = ros2subscriber(node, sensors(i).topic, sensors(i).type);
        subscribers{i} = sub;
        fprintf('  ✓ %s (%s)\n', sensors(i).name, sensors(i).topic);
    catch
        subscribers{i} = [];
        fprintf('  ✗ %s - subscription failed\n', sensors(i).name);
    end
end

fprintf('\nCollecting data for 8 seconds...\n\n');
message_counts = zeros(1, sensor_count);
start_time = tic;

while toc(start_time) < 8
    for i = 1:sensor_count
        if ~isempty(subscribers{i})
            if ~isempty(subscribers{i}.LatestMessage)
                message_counts(i) = message_counts(i) + 1;
            end
        end
    end
    pause(0.1);
end

fprintf('\n%s\n', repmat('=', 1, 40));
fprintf('Results:\n');
fprintf('%s\n', repmat('=', 1, 40));

for i = 1:sensor_count
    rate = message_counts(i) / 8;
    fprintf('  %s: %d messages (%.1f Hz)\n', sensors(i).name, message_counts(i), rate);
end

fprintf('\nTotal: %d messages collected\n', sum(message_counts));

if sum(message_counts) > 0
    fprintf('✓ Test successful!\n');
else
    fprintf('✗ No data collected\n');
end

fprintf('\n');
clear node subscribers;
EOF

timeout 15 matlab -batch "run('/tmp/quick_matlab_test.m')" 2>&1 || true

echo ""
echo "╔════════════════════════════════════════════╗"
echo "║        Recovery Complete                   ║"
echo "╚════════════════════════════════════════════╝"
echo ""

# Step 5: Cleanup
log_info "Cleaning up simulation..."
kill $GZ_LAUNCH_PID 2>/dev/null || true
for pattern in "${cleanup_patterns[@]}"; do
    pkill -TERM -f "$pattern" || true
done
sleep 2
for pattern in "${cleanup_patterns[@]}"; do
    pkill -KILL -f "$pattern" || true
done
sleep 1

echo ""
echo "Logs saved to:"
echo "  Gazebo: $GAZEBO_LOG"
echo "  Topics: $SENSOR_CHECK_LOG"
echo ""
log_success "Recovery script completed"
