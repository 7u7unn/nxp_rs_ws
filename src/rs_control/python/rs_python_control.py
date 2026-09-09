#!/usr/bin/env python3
"""Run the Python RobStride ROS 2 controller."""

from pathlib import Path
import sys

import rclpy

# CMake installs this wrapper without the .py suffix. Explicitly add the
# sibling package directory because some ros2-run launchers do not preserve
# the script directory as sys.path[0].
sys.path.insert(0, str(Path(__file__).resolve().parent))

from rs_control_py.node import PythonRobstrideNode


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PythonRobstrideNode()
        rclpy.spin(node.node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"[rs_python_control] {exc}")
        raise SystemExit(1) from exc
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
