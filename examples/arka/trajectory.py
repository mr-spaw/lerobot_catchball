import time
import json
import numpy as np
from scipy.signal import savgol_filter
from lerobot.robots.so_follower import SO100Follower, SO100FollowerConfig

LOAD_FILE = "trajectory.json"
PLAYBACK_SPEED = 1.0  # 1.0 = real time, 0.5 = half speed (smoother), 2.0 = double speed


def load_and_smooth_trajectory(filepath: str) -> list[dict]:
    """Load recorded trajectory and apply Savitzky-Golay smoothing to remove hand jitter."""
    with open(filepath, "r") as f:
        data = json.load(f)

    print(f"Loaded {len(data)} samples from '{filepath}'")

    joint_keys = list(data[0]["joints"].keys())
    timestamps = [entry["elapsed_ms"] for entry in data]

    # Extract per-joint position arrays
    raw = {k: np.array([entry["joints"][k] for entry in data]) for k in joint_keys}

    # --- Savitzky-Golay filter: smooths jitter while preserving motion shape ---
    # window_length: must be odd, larger = smoother (but less sharp transitions)
    # polyorder: 3 = cubic fit inside window (good balance)
    window_length = min(151, len(data) if len(data) % 2 != 0 else len(data) - 1)
    smoothed = {}
    for k in joint_keys:
        smoothed[k] = savgol_filter(raw[k], window_length=window_length, polyorder=3)

    # Rebuild trajectory list with smoothed values
    smooth_traj = []
    for i, entry in enumerate(data):
        smooth_traj.append({
            "elapsed_ms": timestamps[i],
            "joints": {k: float(smoothed[k][i]) for k in joint_keys},
        })

    print(f"Smoothing applied (window={window_length}, polyorder=3)")
    return smooth_traj


def main():
    # Load and smooth trajectory
    trajectory = load_and_smooth_trajectory(LOAD_FILE)
    joint_keys = list(trajectory[0]["joints"].keys())
    total_duration_s = trajectory[-1]["elapsed_ms"] / 1000.0
    print(f"Trajectory duration: {total_duration_s:.2f}s → playback at {PLAYBACK_SPEED}x = {total_duration_s / PLAYBACK_SPEED:.2f}s")

    # Connect robot
    config = SO100FollowerConfig(
        port="/dev/ttyACM0",
        id="my_awesome_follower_arm",
        use_degrees=True,
    )
    robot = SO100Follower(config)
    robot.connect()
    print("Connected.")

    # --- Move smoothly to the FIRST waypoint before starting replay ---
    print("Moving to start position...")
    obs = robot.get_observation()
    current_pose = {k: obs[k] for k in obs if k.endswith(".pos")}
    start_target = trajectory[0]["joints"]
    WARMUP_STEPS = 200
    WARMUP_TIME = 3.0

    for i in range(WARMUP_STEPS):
        t = (i + 1) / WARMUP_STEPS
        alpha = t * t * t * (t * (6.0 * t - 15.0) + 10.0)  # quintic smoothstep
        action = {k: current_pose[k] + alpha * (start_target[k] - current_pose.get(k, start_target[k]))
                  for k in joint_keys}
        robot.send_action(action)
        time.sleep(WARMUP_TIME / WARMUP_STEPS)

    print("At start position. Replaying trajectory...\n")

    # --- Replay loop ---
    playback_start = time.perf_counter()

    for i, waypoint in enumerate(trajectory):
        target_time = (waypoint["elapsed_ms"] / 1000.0) / PLAYBACK_SPEED
        action = waypoint["joints"]

        # Wait until it's time for this waypoint
        now = time.perf_counter() - playback_start
        wait = target_time - now
        if wait > 0:
            time.sleep(wait)

        robot.send_action(action)

        if i % 1000 == 0:
            print(f"  [{i}/{len(trajectory)}] t={target_time:.2f}s | {action}")

    print("\n✅ Trajectory complete.")
    robot.disconnect()
    print("Disconnected safely.")


if __name__ == "__main__":
    main()