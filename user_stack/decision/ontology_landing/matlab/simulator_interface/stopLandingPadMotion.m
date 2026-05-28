function sim = stopLandingPadMotion(sim)
%STOPLANDINGPADMOTION Stop landing target helpers and command rover halt.

if isfield(sim, 'padMotion')
    sim.padMotion.enabled = false;
end

if isfield(sim, 'padModule') && ~isempty(sim.padModule)
    try
        sim.padModule.stop();
    catch ME
        fprintf('[simulator_interface] stopLandingPadMotion warning: %s\n', ME.message);
    end
    sim.padModule = [];
else
    try
        stop_rover_motion_local();
    catch
    end
end

function stop_rover_motion_local()
cmd = ['bash -lc ''source /opt/ros/humble/setup.bash 2>/dev/null || true; ' ...
    'export ROS_DOMAIN_ID=0; export GZ_VERSION=harmonic; ' ...
    'ros2 topic pub --once /model/r1_rover/cmd_vel geometry_msgs/msg/Twist ' ...
    '"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true; ' ...
    'for t in /model/r1_rover/joint/motor_0/cmd_force /model/r1_rover/joint/motor_1/cmd_force /model/r1_rover/joint/motor_2/cmd_force /model/r1_rover/joint/motor_3/cmd_force; do ' ...
    'gz topic -t "$t" -m gz.msgs.Double -p "data: 0.0" >/dev/null 2>&1 || true; done'''];
system(cmd);
end
end
