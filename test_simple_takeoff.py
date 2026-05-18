#!/usr/bin/env python3
"""
Simple takeoff sequence test:
1. Connect
2. Arm
3. Takeoff (without mode change)
"""

import sys
import time
from pymavlink import mavutil

print("Connecting to MAVLink (try udp:14550 then tcp:5760)...")
m = None
for uri in ("udp:127.0.0.1:14550", "tcp:127.0.0.1:5760"):
    try:
        print(f"  trying {uri}")
        m = mavutil.mavlink_connection(uri, source_system=251)
        break
    except Exception as e:
        print(f"  connect failed {uri}: {e}")
        m = None
if m is None:
    raise RuntimeError("Could not open MAVLink connection on udp:14550 or tcp:5760")

print("Waiting for heartbeat...")
hb = m.wait_heartbeat(timeout=20)
# Ensure target_system/component are set from the received heartbeat
if hb is not None:
    try:
        src_sys = hb.get_srcSystem()
        src_comp = hb.get_srcComponent()
        m.target_system = src_sys
        m.target_component = src_comp
    except Exception:
        pass
else:
    hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
if hb is not None:
    try:
        src_sys = hb.get_srcSystem()
        src_comp = hb.get_srcComponent()
    except Exception:
        src_sys = getattr(hb, 'sysid', 'NA')
        src_comp = getattr(hb, 'compid', 'NA')
    print(f"Heartbeat OK from SYSID={m.target_system} component={m.target_component} (msg src_sys={src_sys} src_comp={src_comp})")
    print(f"  HB fields: base_mode={hb.base_mode} custom_mode={hb.custom_mode} system_status={hb.system_status}")
else:
    print(f"Heartbeat OK (no message object) SYSID={m.target_system} component={m.target_component}")

def send_heartbeat():
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavutil.mavlink.MAV_STATE_ACTIVE, 3)

# === STEP 1: ARM ===
print("\n[STEP 1] Arming in STABILIZE mode...")
# Diagnostic dump: request streams and print key telemetry for 5s before arming
print("Performing pre-arm telemetry dump (5s): SYS_STATUS, GPS_RAW_INT, GLOBAL_POSITION_INT, STATUSTEXT")
try:
    m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)
except Exception:
    pass
t_diag = time.time() + 5
while time.time() < t_diag:
    msg = m.recv_match(blocking=True, timeout=1)
    if msg is None:
        continue
    t = msg.get_type()
    if t in ('SYS_STATUS', 'GPS_RAW_INT', 'GLOBAL_POSITION_INT', 'STATUSTEXT'):
        try:
            if t == 'STATUSTEXT':
                text = msg.text.decode() if isinstance(msg.text, bytes) else msg.text
                print(f"  {t}: {text}")
            else:
                print(f"  {t}: {msg}")
        except Exception:
            print(f"  {t}: (could not stringify)")
    else:
        # print only a few other messages for context
        pass
print("Pre-arm telemetry dump complete.")
# Request parameter list and capture ARMING_REQUIRE / ARMING_SKIPCHK
print("Requesting parameter list to check ARMING_REQUIRE / ARMING_SKIPCHK (3s)")
try:
    m.mav.param_request_list_send(m.target_system, m.target_component)
except Exception:
    pass
t_end = time.time() + 3
while time.time() < t_end:
    msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
    if msg is None:
        continue
    try:
        name = msg.param_id.decode().strip('\x00') if isinstance(msg.param_id, bytes) else msg.param_id
    except Exception:
        name = str(msg.param_id)
    if name in ('ARMING_REQUIRE', 'ARMING_SKIPCHK'):
        print(f"  PARAM {name} = {msg.param_value}")
print("Parameter check complete.")
for i in range(3):
    send_heartbeat()
    time.sleep(0.1)
send_heartbeat()
time.sleep(0.5)
m.mav.command_long_send(
    m.target_system,
    m.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0
)
print("Arm command sent (COMMAND_LONG ARM_DISARM)")

ack_deadline = time.time() + 3
while time.time() < ack_deadline:
    ack = m.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.5)
    if ack is None:
        continue
    if int(getattr(ack, 'command', -1)) == int(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM):
        print(f"  COMMAND_ACK ARM_DISARM: result={getattr(ack, 'result', 'NA')} progress={getattr(ack, 'progress', 'NA')}")
        break

arm_deadline = time.time() + 12
armed = False
while time.time() < arm_deadline:
    send_heartbeat()
    # receive any message type so we don't miss STATUSTEXT / COMMAND_ACK
    msg = m.recv_match(blocking=True, timeout=0.5)
    if msg is None:
        time.sleep(0.2)
        continue

    etype = msg.get_type()
    if etype == 'HEARTBEAT':
        is_armed = msg.base_mode & 128
        mode = m.flightmode
        print(f"  Check: armed={(is_armed!=0)} mode={mode}")
        if is_armed:
            print("✓ Vehicle ARMED")
            armed = True
            break
    elif etype == 'STATUSTEXT':
        try:
            text = msg.text.decode() if isinstance(msg.text, bytes) else msg.text
        except Exception:
            text = str(msg.text)
        print(f"  STATUSTEXT: {text}")
    elif etype == 'COMMAND_ACK':
        print(f"  COMMAND_ACK: command={getattr(msg, 'command', 'NA')} result={getattr(msg, 'result', 'NA')}")

if not armed:
    # Drain remaining STATUSTEXT messages for context
    print("No armed heartbeat received — collecting STATUSTEXT for diagnosis:")
    t0 = time.time()
    while time.time() - t0 < 3:
        msg = m.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
        if msg is None:
            break
        try:
            text = msg.text.decode() if isinstance(msg.text, bytes) else msg.text
        except Exception:
            text = str(msg.text)
        print(f"  STATUSTEXT: {text}")
    raise RuntimeError("ARM FAILED")

# === STEP 2: SWITCH TO GUIDED + TAKEOFF ===
print("\n[STEP 2] Switching to GUIDED mode...")
m.set_mode("GUIDED")
mode_deadline = time.time() + 10
guided = False
while time.time() < mode_deadline:
    send_heartbeat()
    hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
    if hb is None:
        continue
    mode = str(m.flightmode).upper() if m.flightmode is not None else "UNKNOWN"
    print(f"  Mode check: {mode}")
    if mode == 'GUIDED':
        guided = True
        break

if not guided:
    raise RuntimeError(f"GUIDED mode not confirmed (current={m.flightmode})")

print("[STEP 3] Takeoff command in GUIDED mode...")
target_alt = 3.0
time.sleep(0.5)
m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0,0,0,0,0,0, target_alt)
print(f"Takeoff command sent (target={target_alt} m AGL)")

t_start = time.time()
t_end = t_start + 30
climbing = False
max_alt = 0.0

while time.time() < t_end:
    send_heartbeat()
    msg = m.recv_match(type=["GLOBAL_POSITION_INT", "LOCAL_POSITION_NED"], blocking=True, timeout=0.5)
    if msg is None:
        continue
    
    elapsed = time.time() - t_start
    msg_type = msg.get_type()
    
    if msg_type == "GLOBAL_POSITION_INT":
        rel_alt_m = float(msg.relative_alt) / 1000.0
        max_alt = max(max_alt, rel_alt_m)
        vz = getattr(msg, 'vz', 0)
        if rel_alt_m > 0.2:
            if not climbing:
                print(f"  ✓ Climbing detected at t={elapsed:.1f}s: alt={rel_alt_m:.2f} m")
                climbing = True
        if rel_alt_m >= target_alt * 0.9:
            print(f"  ✓ Target altitude reached at t={elapsed:.1f}s: alt={rel_alt_m:.2f} m")
            break
    
    elif msg_type == "LOCAL_POSITION_NED":
        z = float(msg.z)
        if z < -0.2 and not climbing:
            print(f"  ✓ Climbing detected (LOCAL): z={z:.2f} m at t={elapsed:.1f}s")
            climbing = True

if not climbing:
    print(f"\nWARNING: No climb detected after 30s. Max altitude: {max_alt:.2f} m")
    sys.exit(1)
else:
    print(f"\nSUCCESS: Takeoff sequence completed. Max altitude: {max_alt:.2f} m")
    sys.exit(0)
