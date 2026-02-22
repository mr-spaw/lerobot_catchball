"""
python home_pos.py --arm arka_1
python home_pos.py --arm arka_2
python home_pos.py --arm arka_1 --port /dev/ttyACM0
python home_pos.py --arm arka_2 --port /dev/ttyACM1
    
"""
import argparse
import time
import threading
from lerobot.robots.so_follower import SO100Follower, SO100FollowerConfig

# ---------------------------------------------------------------------------
# Home poses 
# ---------------------------------------------------------------------------
HOME_POSES = {
    "arka_1": {
        "shoulder_pan.pos":  -9.00,
        "shoulder_lift.pos": -90.00,
        "elbow_flex.pos":    100.00,
        "wrist_flex.pos":     60.00,
        "wrist_roll.pos":      0.00,
        "gripper.pos":         1.00,
    },
    "arka_2": {
        "shoulder_pan.pos":  -6.00,
        "shoulder_lift.pos": -98.00,
        "elbow_flex.pos":     92.00,
        "wrist_flex.pos":     60.00,
        "wrist_roll.pos":    -65.00,
        "gripper.pos":         3.00,
    },
}

# Default ports per arm 
DEFAULT_PORTS = {
    "arka_1": "/dev/ttyACM0",
    "arka_2": "/dev/ttyACM1",
}

# ---------------------------------------------------------------------------
# Motion config
# ---------------------------------------------------------------------------

MOVE_TIME  = 5.0
STEPS      = 2000

# --- Stuck detection ---
MONITOR_HZ               = 30
STUCK_VELOCITY_THRESHOLD = 0.3   
STUCK_TIME_SEC           = 2.5   
STUCK_DECAY_FACTOR       = 4.0   

DEAD_ZONE_DEG            = 2.0    

# --- Detour ---
DETOUR_AMPLITUDE         = 5.0   
DETOUR_RAMP_SPEED        = 0.8

# --- Smoothing ---
POSE_EMA_ALPHA           = 0.3
OFFSET_SMOOTH_ALPHA      = 0.08
OFFSET_DECAY_ALPHA       = 0.05

WRAP_JOINTS = {"wrist_roll.pos"}

HELPER_MAP = {
    "shoulder_pan.pos":  "elbow_flex.pos",
    "elbow_flex.pos":    "shoulder_pan.pos",
    "wrist_roll.pos":    "wrist_flex.pos",
    "wrist_flex.pos":    "elbow_flex.pos",
    "shoulder_lift.pos": "shoulder_pan.pos",
}

# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------

state = {
    "ema_pose":       {},
    "actual_pose":    {},
    "stuck_joints":   set(),
    "target_offsets": {},
    "smooth_offsets": {},
    "lock":           threading.Lock(),
    "running":        True,
}

# ---------------------------------------------------------------------------
# Math helpers
# ---------------------------------------------------------------------------

def smooth_step(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * t * (t * (6.0 * t - 15.0) + 10.0)

def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t

def wrap_delta(a: float, b: float, wrap: float = 360.0) -> float:
    d = (b - a) % wrap
    if d > wrap / 2:
        d -= wrap
    return d

def apply_dead_zone(value: float, dead_zone: float) -> float:
    """Zero out values within the dead zone to prevent vibration on tiny errors."""
    if abs(value) < dead_zone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - dead_zone)

# ---------------------------------------------------------------------------
# Monitor thread
# ---------------------------------------------------------------------------

def monitor_thread():
    interval         = 1.0 / MONITOR_HZ
    stuck_timers     = {}
    detour_phase     = {}
    detour_direction = {}

    while state["running"]:
        time.sleep(interval)

        with state["lock"]:
            actual = dict(state["actual_pose"])
            ema    = dict(state["ema_pose"])

        if not actual or not ema:
            continue

        new_ema = {}
        for j, v in actual.items():
            prev = ema.get(j, v)
            if j in WRAP_JOINTS:
                new_ema[j] = prev + POSE_EMA_ALPHA * wrap_delta(prev, v)
            else:
                new_ema[j] = lerp(prev, v, POSE_EMA_ALPHA)

        new_target_offsets = {}
        new_stuck          = set()

        for joint in actual:
            if joint == "gripper.pos":
                continue

            if joint in WRAP_JOINTS:
                delta = abs(wrap_delta(ema.get(joint, new_ema[joint]), new_ema[joint]))
            else:
                delta = abs(new_ema[joint] - ema.get(joint, new_ema[joint]))

            velocity = delta / interval

            if velocity < STUCK_VELOCITY_THRESHOLD:
                stuck_timers[joint] = stuck_timers.get(joint, 0.0) + interval
            else:
                stuck_timers[joint] = max(
                    0.0,
                    stuck_timers.get(joint, 0.0) - interval * STUCK_DECAY_FACTOR
                )
                if stuck_timers[joint] <= 0.0 and joint in detour_phase:
                    detour_phase.pop(joint, None)
                    detour_direction.pop(joint, None)
                    print(f"[MONITOR]  {joint} free (v={velocity:.2f}°/s)")

            if stuck_timers.get(joint, 0.0) >= STUCK_TIME_SEC:
                new_stuck.add(joint)
                helper = HELPER_MAP.get(joint, "elbow_flex.pos")

                if helper not in detour_direction:
                    detour_direction[helper] = 1.0

                phase = min(1.0, detour_phase.get(helper, 0.0) + interval * DETOUR_RAMP_SPEED)
                detour_phase[helper] = phase

                raw_offset = DETOUR_AMPLITUDE * smooth_step(phase) * detour_direction[helper]
                offset = apply_dead_zone(raw_offset, DEAD_ZONE_DEG)
                if offset != 0.0:
                    new_target_offsets[helper] = offset

                print(f"[MONITOR]  {joint} STUCK (v={velocity:.2f}°/s, "
                      f"{stuck_timers[joint]:.1f}s) → detour {helper}: "
                      f"{raw_offset:+.1f}°")

                if phase >= 1.0:
                    detour_direction[helper] *= -1
                    detour_phase[helper]      = 0.0
                    print(f"[MONITOR]  Flipping detour on {helper}")

        with state["lock"]:
            state["ema_pose"]       = new_ema
            state["stuck_joints"]   = new_stuck
            state["target_offsets"] = new_target_offsets

# ---------------------------------------------------------------------------
# Safe disconnect
# ---------------------------------------------------------------------------

def safe_disconnect(robot, settle_secs: float = 2.0, retries: int = 3, retry_delay: float = 2.0):
    print(f"Waiting {settle_secs}s for motors to settle...")
    time.sleep(settle_secs)

    try:
        obs = robot.get_observation()
        joint_keys = [k for k in obs if k.endswith(".pos")]
        robot.send_action({k: obs[k] for k in joint_keys})
        time.sleep(0.4)
    except Exception:
        pass

    for attempt in range(1, retries + 1):
        try:
            robot.disconnect()
            print("Disconnected safely.")
            return
        except RuntimeError as e:
            print(f"[WARN] Disconnect attempt {attempt}/{retries} failed: {e}")
            if attempt < retries:
                time.sleep(retry_delay)
    print("[ERROR] Could not disconnect cleanly. Power cycle if needed.")

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Move SO100 arm to home pose.")
    parser.add_argument(
        "--arm",
        type=str,
        default="arka_1",
        choices=list(HOME_POSES.keys()),
        help=f"Which arm to home. Options: {list(HOME_POSES.keys())}",
    )
    parser.add_argument(
        "--port",
        type=str,
        default=None,
        help="Serial port (e.g. /dev/ttyACM0). Auto-selected per arm if not set.",
    )
    args = parser.parse_args()

    arm_id    = args.arm
    home_pose = HOME_POSES[arm_id]
    port      = args.port or DEFAULT_PORTS.get(arm_id, "/dev/ttyACM0")

    config = SO100FollowerConfig(
        port=port,
        id=arm_id,
        use_degrees=True,
    )

    robot = SO100Follower(config)
    robot.connect()
    print(f"Connected {arm_id} on {port}")
    print("(lerobot calibration loaded automatically from cache)\n")

    obs        = robot.get_observation()
    joint_keys = [k for k in obs if k.endswith(".pos")]
    start_pose = {k: obs[k] for k in joint_keys}

    print("  Start pose:")
    for k, v in start_pose.items():
        print(f"    {k:25s} {v:+.2f}°")

    print(f"\n  Target pose ({arm_id}):")
    for k, v in home_pose.items():
        print(f"    {k:25s} {v:+.2f}°")

    print(f"\n  Moving to home over {MOVE_TIME}s...\n")

    with state["lock"]:
        state["actual_pose"]    = dict(start_pose)
        state["ema_pose"]       = dict(start_pose)
        state["smooth_offsets"] = {k: 0.0 for k in joint_keys}

    monitor = threading.Thread(target=monitor_thread, daemon=True)
    monitor.start()

    step_duration = MOVE_TIME / STEPS
    t0 = time.perf_counter()

    for i in range(STEPS):
        alpha       = smooth_step((i + 1) / STEPS)
        base_action = {
            j: lerp(start_pose[j], home_pose.get(j, start_pose[j]), alpha)
            for j in joint_keys
        }

        with state["lock"]:
            target_offsets = dict(state["target_offsets"])
            stuck          = set(state["stuck_joints"])

            for j in joint_keys:
                prev = state["smooth_offsets"].get(j, 0.0)
                tgt  = target_offsets.get(j, 0.0)
                if tgt == 0.0:
                    state["smooth_offsets"][j] = lerp(prev, 0.0, OFFSET_DECAY_ALPHA)
                else:
                    state["smooth_offsets"][j] = lerp(prev, tgt, OFFSET_SMOOTH_ALPHA)

            smooth_offsets = dict(state["smooth_offsets"])

        action = {
            j: base_action[j] + apply_dead_zone(smooth_offsets.get(j, 0.0), DEAD_ZONE_DEG * 0.5)
            for j in joint_keys
        }
        robot.send_action(action)

        obs    = robot.get_observation()
        actual = {k: obs[k] for k in joint_keys if k in obs}
        with state["lock"]:
            state["actual_pose"] = actual

        if stuck and i % 30 == 0:
            offset_str = {
                k.replace(".pos", ""): f"{v:+.1f}°"
                for k, v in smooth_offsets.items() if abs(v) > DEAD_ZONE_DEG * 0.5
            }
            print(f"[MAIN] Stuck: {[j.replace('.pos','') for j in stuck]} "
                  f"| offsets: {offset_str}")

        elapsed  = time.perf_counter() - t0
        expected = (i + 1) * step_duration
        delay    = expected - elapsed
        if delay > 0:
            time.sleep(delay)

    state["running"] = False
    monitor.join(timeout=1.0)

    print(f"\nReached home pose for {arm_id}.")
    safe_disconnect(robot)


if __name__ == "__main__":
    main()