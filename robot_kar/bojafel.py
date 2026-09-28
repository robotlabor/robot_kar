#!/usr/bin/env python3
import rclpy
import time
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import argparse
import math
from rclpy.node import Node
from dsr_msgs2.srv import DrlStart, GetRobotState, GetWorkpieceWeight
from std_srvs.srv import Trigger
from sensor_msgs.msg import PointCloud2
from .onrobot import RG
from std_msgs.msg import Int32MultiArray


class DrlStartNode(Node):
    def __init__(self, args):
        super().__init__('drl_script_run_node')

        # Gripper
        self.rg = RG(args.gripper, args.ip, args.port)

        # Clients
        self.drl_cli = self.create_client(DrlStart, '/dsr01/drl/drl_start')
        self.state_cli = self.create_client(GetRobotState, '/dsr01/system/get_robot_state')
        self.workpiece_cli = self.create_client(GetWorkpieceWeight, '/dsr01/force/get_workpiece_weight')

        while not self.drl_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for drl_start...')
        while not self.state_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for robot_state...')
        while not self.workpiece_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for workpiece_weight...')

        # Trigger
        self.trigger_service = self.create_service(
            Trigger, 'step_trigger', self.trigger_callback
        )
        self.step_triggered = False

        # PointCloud
        self.subscription = self.create_subscription(
            PointCloud2, '/tyr/detection/cone_top_pc2', self.store_latest_msg, 10
        )
        self.last_msg = None

        # Column heights subscriber
        self.column_heights = None
        self.column_heights_sub = self.create_subscription(
            Int32MultiArray, '/column_heights', self.column_heights_callback, 10
        )

    # ROS CALLBACKS
    def trigger_callback(self, request, response):
        self.step_triggered = True
        response.success = True
        response.message = "Step executed"
        return response

    def store_latest_msg(self, msg):
        self.last_msg = msg

    def column_heights_callback(self, msg):
        if len(msg.data) == 6:
            self.column_heights = list(msg.data)
            self.get_logger().info(f"Updated column heights: {self.column_heights}")
        else:
            self.get_logger().warn("Column heights msg must have 6 values")

    # FUNCTIONS
    def run_drl_script(self, robot_system, drl_code):
        request = DrlStart.Request()
        request.robot_system = robot_system
        request.code = drl_code
        future = self.drl_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error("Service call failed.")
            return None

    def is_robot_in_standby(self):
        request = GetRobotState.Request()
        future = self.state_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result().robot_state == 1
        else:
            self.get_logger().error("Failed to get robot state.")
            return False

    def update_marker_position(self, num_samples=50, sample_delay=0.01):
        collected_points = []
        for _ in range(num_samples):
            points = list(pc2.read_points(self.last_msg, field_names=("x", "y", "z"), skip_nans=True))
            if len(points) == 0:
                rclpy.spin_once(self, timeout_sec=sample_delay)
                continue
            pt = points[0]
            collected_points.append(np.array([pt[0], pt[1], pt[2]], dtype=np.float32))
            rclpy.spin_once(self, timeout_sec=sample_delay)

        if len(collected_points) == 0:
            return False

        avg_point = np.mean(collected_points, axis=0)
        self.latest_x = float(avg_point[0])
        self.latest_y = float(avg_point[1])
        self.latest_z = float(avg_point[2])
        return True

    def get_workpiece_weight(self):
        request = GetWorkpieceWeight.Request()
        future = self.workpiece_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            return None
        return future.result().weight

    def run_and_wait(self, robot_system_mode, code):
        self.run_drl_script(robot_system_mode, code)
        time.sleep(0.1)
        while not self.is_robot_in_standby():
            time.sleep(0.1)
            rclpy.spin_once(self)

    def camera_to_robot(self):
        if not self.update_marker_position():
            return None
        return (self.latest_y*1000, self.latest_z*-1000, self.latest_x*-1000)

    # MAIN LOGIC
    def run(self):
        robot_system_mode = 0

        # Várakozás a kezdő column_heights-re
        while self.column_heights is None:
            self.get_logger().info("Waiting for starting column heights...")
            rclpy.spin_once(self, timeout_sec=0.1)

        num_rows = 2
        num_cols = 3
        num_levels = 5
        x_start = -439
        y_start = 386
        z_start = 413
        x_step = -270
        y_step = -300
        z_step = 33
        home_position = "posx(-527, 35, 847, -90, 180, 0)"
        watch_position = "posx(-101, 567, 760, 0, 180, 0)"
        help_pos = "posx(-320, 559, 765, 45, 180, 0)"

        drl_code_template = """set_velj(25); set_velx(40); set_accj(25); set_accx(40);"""

        commands = []
        for level in range(num_levels):
            for col in reversed(range(num_cols)):
                for row in reversed(range(num_rows)):
                    column_index = row * num_cols + col
                    if self.column_heights[column_index] < level + 1:
                        x_pos = x_start + row * x_step
                        y_pos = y_start + col * y_step
                        z_pos = z_start + level * z_step
                        commands.append({
                            "place_position": f"posx({x_pos}, {y_pos}, {z_pos}, -90, 180, 0)",
                            "place_offset": f"posx({x_pos}, {y_pos}, {z_pos+400}, -90, 180, 0)",
                            "home_position": home_position,
                            "watch_position": watch_position
                        })

        max_series = 2

        for command in commands:
            while True:
                self.step_triggered = False
                while not self.step_triggered:
                    rclpy.spin_once(self)

                retry = 0
                succes = False
                weight = self.get_workpiece_weight()

                while not succes:
                    self.rg.open_gripper()
                    drl_code = drl_code_template + f"movejx({command['watch_position']}, sol=7);\n"
                    self.run_and_wait(robot_system_mode, drl_code)

                    marker = self.camera_to_robot()
                    if marker is None:
                        retry += 1
                        continue

                    x, y, z = marker
                    if math.sqrt(x*x + y*y) > 300 or y > 300:
                        boja_position_offset = f"posx({x/1.5}, {y/1.5}, {z/5}, 0, 0, 0)"
                        drl_code = drl_code_template + f"movejx({boja_position_offset}, sol=7, mod=DR_MV_MOD_REL);\n"
                        self.run_and_wait(robot_system_mode, drl_code)

                    # Marker correction loop
                    marker = self.camera_to_robot()
                    if marker is None:
                        retry += 1
                        continue

                    x, y, z = marker
                    boja_position_offset = f"posx({x}, {y}, {z/3}, 0, 0, 0)"
                    drl_code = drl_code_template + f"movejx({boja_position_offset}, sol=7, mod=DR_MV_MOD_REL);\n"
                    self.run_and_wait(robot_system_mode, drl_code)

                    marker = self.camera_to_robot()
                    if marker is None:
                        retry += 1
                        continue

                    x, y, z = marker
                    boja_position_offset = f"posx({x+65}, {y-131}, {z/5}, 0, 0, 0)"
                    drl_code = drl_code_template + f"movejx({boja_position_offset}, sol=7, mod=DR_MV_MOD_REL);\n"
                    self.run_and_wait(robot_system_mode, drl_code)

                    marker = self.camera_to_robot()
                    if marker is None:
                        retry += 1
                        continue

                    x, y, z = marker
                    drl_code = drl_code_template + f"movejx(posx(0, 0, {z+221}, 0, 0, 0), mod=DR_MV_MOD_REL, sol=7);\n"
                    drl_code += f"movel(posx(0, 0, -50, 0, 0, 0), mod=DR_MV_MOD_REL);\n"
                    self.run_and_wait(robot_system_mode, drl_code)

                    self.rg.close_gripper()
                    time.sleep(0.5)
                    drl_code = drl_code_template + "movel(posx(0, 0, 100, 0, 0, 0), mod=DR_MV_MOD_REL);\n"
                    drl_code += f"movejx({help_pos}, radius=100, sol=7);\n"
                    drl_code += f"movejx({command['place_offset']}, sol=7);\n"
                    self.run_and_wait(robot_system_mode, drl_code)
                    time.sleep(0.5)

                    weight2 = self.get_workpiece_weight()
                    if weight2 > weight + 0.5:
                        succes = True
                        retry -= 1
                    elif retry == max_series:
                        break
                    else:
                        retry += 1
                        continue

                if retry == max_series:
                    self.rg.open_gripper()
                    drl_code = drl_code_template + f"movejx({command['home_position']}, sol=7);\n"
                    self.run_and_wait(robot_system_mode, drl_code)
                    continue

                drl_code = drl_code_template + f"movel({command['place_position']});\n"
                self.run_and_wait(robot_system_mode, drl_code)
                self.rg.open_gripper()
                time.sleep(0.5)
                drl_code = drl_code_template + f"movel({command['place_offset']});\n"
                drl_code += f"movejx({command['home_position']}, sol=7);\n"
                self.run_and_wait(robot_system_mode, drl_code)
                break

        self.get_logger().info("Platform is full")


def get_options():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gripper', default="rg6")
    parser.add_argument('--ip', default="192.168.1.1")
    parser.add_argument('--port', default="502")
    return parser.parse_args()


def main():
    rclpy.init()
    args = get_options()
    node = DrlStartNode(args)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
