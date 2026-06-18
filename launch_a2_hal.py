import shlex
import subprocess
import time
from pathlib import Path


DIR_PATH = Path(__file__).resolve().parent
ROS_WS = DIR_PATH / "ros2_ws"
EXECUTABLE = ROS_WS / "install" / "a2_hal" / "bin" / "a2_hal"

CYCLONEDDS_SETUP = DIR_PATH / "unitree_ros2" / "cyclonedds_ws" / "install" / "setup.bash"
if not CYCLONEDDS_SETUP.exists():
    CYCLONEDDS_SETUP = DIR_PATH / "cyclonedds_ws" / "install" / "setup.bash"


def run_bash(command):
    subprocess.run(["bash", "-c", command], check=True)


def quoted(path):
    return shlex.quote(str(path))


print("submodule_path:", ROS_WS / "install")
if not EXECUTABLE.exists():
    print("Building the A2 HAL/msgs first..")
    run_bash(
        f"source {quoted(CYCLONEDDS_SETUP)} && "
        f"cd {quoted(ROS_WS)} && "
        "colcon build --packages-select dls2_interface a2_hal && "
        f"cd {quoted(DIR_PATH)} && "
        "source unitree_ros2_connect.bash && "
        "source ros2_ws/install/setup.bash && "
        f"{quoted(EXECUTABLE)}"
    )
else:
    print("\n\n")
    print("A2 HAL/msgs already built - if you have any modifications, please delete the build folder in the submodule")
    print("\n\n")
    time.sleep(2)
    run_bash(
        f"source {quoted(CYCLONEDDS_SETUP)} && "
        f"cd {quoted(DIR_PATH)} && "
        "source unitree_ros2_connect.bash && "
        "source ros2_ws/install/setup.bash && "
        f"{quoted(EXECUTABLE)}"
    )
