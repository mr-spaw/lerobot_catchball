import time
import math
from lerobot.robots.so_follower import SO100Follower, SO100FollowerConfig

# Target joint configuration (degrees)
TARGET_POSE = {
    "shoulder_pan.pos": -9.00,
    "shoulder_lift.pos": -90.00,
    "elbow_flex.pos": 100.00,
    "wrist_flex.pos": 60.00,
    "wrist_roll.pos": -75.00,
    "gripper.pos": 1.00,
}

MOVE_TIME = 5.0   
STEPS = 1000     

def smooth_step(alpha: float) -> float:
    """
    Formula: 6t^5 - 15t^4 + 10t^3  (quintic smoothstep)
    """
    t = max(0.0, min(1.0, alpha))
    return t * t * t * (t * (6.0 * t - 15.0) + 10.0)


def main():
    config = SO100FollowerConfig(
        port="/dev/ttyACM0",
        id="my_awesome_follower_arm",
        use_degrees=True,
    )

    robot = SO100Follower(config)
    robot.connect()
    print("Connected. Reading current pose...")

    # Get current pose
    obs = robot.get_observation()
    joint_keys = [k for k in obs.keys() if k.endswith(".pos")]
    current_pose = {k: obs[k] for k in joint_keys}

    print(f"Moving to target pose over {MOVE_TIME}s with smooth S-curve profile...")

    step_duration = MOVE_TIME / STEPS

    for i in range(STEPS):
        linear_alpha = (i + 1) / STEPS
        eased_alpha = smooth_step(linear_alpha)  #key: remap with S-curve

        action = {}
        for joint in joint_keys:
            start = current_pose[joint]
            target = TARGET_POSE.get(joint, start)  # fallback: hold position
            action[joint] = start + eased_alpha * (target - start)

        robot.send_action(action)

        
        time.sleep(step_duration)

    print("Reached target pose smoothly.")
    robot.disconnect()
    print("Disconnected safely.")


if __name__ == "__main__":
    main()