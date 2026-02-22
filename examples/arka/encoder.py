import sys
import time
from lerobot.robots.so_follower import SO100Follower, SO100FollowerConfig

REFRESH_RATE = 0.05  # 20 Hz

JOINT_KEYS = [
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_roll.pos",
    "gripper.pos",
]


def main():
    config = SO100FollowerConfig(
        port="/dev/ttyACM0",
        id="my_awesome_follower_arm",
        use_degrees=True,
    )

    robot = SO100Follower(config)
    robot.connect()
    robot.bus.disable_torque()

    try:
        while True:
            obs = robot.get_observation()

            sys.stdout.write("\033[2J\033[H")  # clear screen
            print("=== Live Encoder Values ===\n")
            for k in JOINT_KEYS:
                print(f"  {k:<25} {obs[k]:>8.2f}°")
            print("\nCTRL+C to stop.")
            sys.stdout.flush()

            time.sleep(REFRESH_RATE)

    except KeyboardInterrupt:
        print("\nStopped.")

    finally:
        robot.disconnect()
        print("Disconnected safely.")


if __name__ == "__main__":
    main()