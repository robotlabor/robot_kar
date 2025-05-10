#!/usr/bin/env python3

import rclpy
import time
import argparse
import threading

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from dsr_msgs2.srv import DrlStart, GetRobotState
from robot_kar.onrobot import RG

from std_srvs.srv import Trigger
from std_msgs.msg import Int32MultiArray, Int32, Bool


class DrlStartNode(Node):
    def __init__(self, gripper, ip, port):
        super().__init__('drl_script_run_node')

        self.drl_cli = self.create_client(DrlStart, '/dsr01/drl/drl_start')
        self.state_cli = self.create_client(GetRobotState, '/dsr01/system/get_robot_state')

        while not self.drl_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /dsr01/drl/drl_start service...')
        while not self.state_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /dsr01/system/get_robot_state service...')

        self.trigger_service = self.create_service(Trigger, 'step_trigger', self.trigger_callback)
        self.sub_column_heights = self.create_subscription(Int32MultiArray, '/column_heights', self.column_heights_callback, 10)
        self.reset_subscriber = self.create_subscription(Bool, '/reset_remaining_markers', self.reset_remaining_markers_callback, 10)
        self.remaining_boja_pub = self.create_publisher(Int32, '/remaining_markers', 10)

        self.gripper = RG(gripper, ip, port)
        self.column_heights = []
        self.current_layout = []
        self.step_triggered = False
        self.total_boja = 0
        self.is_running = False
        self.needs_restart = False
        self.lock = threading.Lock()

    def trigger_callback(self, request, response):
        self.step_triggered = True
        response.success = True
        response.message = "Step executed"
        return response

    def column_heights_callback(self, msg):
        with self.lock:
            self.column_heights = list(msg.data)
            self.get_logger().info(f"[column_heights_callback] Received: {self.column_heights}")
            if not self.is_running:
                self.needs_restart = True
                self.start_marker_thread()
            else:
                self.get_logger().info("[column_heights_callback] Robot is busy.")

    def reset_remaining_markers_callback(self, msg):
        if msg.data:
            with self.lock:
                self.get_logger().info("[reset_remaining_markers] Triggered.")
                if not self.is_running and self.column_heights:
                    self.needs_restart = True
                    self.start_marker_thread()
                elif self.is_running:
                    self.get_logger().info("[reset_remaining_markers] Still running.")
                else:
                    self.get_logger().warn("[reset_remaining_markers] No layout received yet.")

    def start_marker_thread(self):
        self.get_logger().info("[start_marker_thread] Launching marker thread.")
        threading.Thread(target=self.process_markers, daemon=True).start()

    def run_drl_script(self, robot_system, drl_code):
        request = DrlStart.Request()
        request.robot_system = robot_system
        request.code = drl_code
        future = self.drl_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error("DRL service call failed.")
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

    def wait_for_standby(self, context_msg=""):
        while not self.is_robot_in_standby():
            self.get_logger().info(f"{context_msg} — waiting for robot standby...")
            time.sleep(2)
            rclpy.spin_once(self)

    def generate_commands(self):
        layout = list(self.current_layout)
        commands = []

        num_rows, num_cols, num_levels = 2, 3, 5
        x_start, y_start, z_start = -439, 386, 413
        x_step, y_step, z_step = -270, -300, 33
        drop_position = "posx(42, 735, -52, -90, 180, 0)"
        drop_offset1 = "posx(42, 735, 853, -90, 180, 0)"
        drop_offset = "posx(42, 735, 50, -90, 180, 0)"
        home_position = "posx(-527, 35, 847, -90, 180, 0)"

        for level in reversed(range(num_levels)):
            for col in range(num_cols):
                for row in range(num_rows):
                    idx = row * num_cols + col
                    if idx >= len(layout):
                        continue
                    if layout[idx] > level:
                        x = x_start + row * x_step
                        y = y_start + col * y_step
                        z = z_start + level * z_step
                        commands.append({
                            "pick_position": f"posx({x}, {y}, {z}, -90, 180, 0)",
                            "drop_position": drop_position,
                            "home_position": home_position,
                            "pick_offset": f"posx({x}, {y}, {z+100}, -90, 180, 0)",
                            "pickup_position": f"posx({x}, {y}, 870, -90, 180, 0)",
                            "drop_offset": drop_offset,
                            "drop_offset1": drop_offset1
                        })

        self.get_logger().info(f"Generated {len(commands)} marker commands.")
        return commands

    def process_markers(self):
        with self.lock:
            if not self.needs_restart:
                self.get_logger().info("[process_markers] No restart needed. Exiting.")
                return
            if self.is_running:
                self.get_logger().info("[process_markers] Already running")
                return

            self.is_running = True
            self.needs_restart = False
            self.current_layout = list(self.column_heights)

        self.get_logger().info("[process_markers] BEGIN placement.")

        try:
            commands = self.generate_commands()
            self.total_boja = len(commands)

            if self.total_boja == 0:
                self.get_logger().warn("[process_markers] No marker commands generated.")
                return

            robot_system_mode = 0
            drl_code_template = """
set_velj(30);
set_velx(30);
set_accj(30);
set_accx(30);
"""

            for idx, command in enumerate(commands):
                self.step_triggered = False
                while not self.step_triggered:
                    self.get_logger().info(f"Waiting for step_trigger... ({idx+1}/{self.total_boja})")
                    rclpy.spin_once(self, timeout_sec=0.5)

                self.gripper.open_gripper()

                drl = drl_code_template + f"""
movejx({command['home_position']}, sol=7);
movejx({command['pick_offset']}, sol=7);
movel({command['pick_position']});
"""
                self.run_drl_script(robot_system_mode, drl)
                time.sleep(2)
                self.wait_for_standby("After pick")

                self.gripper.close_gripper()
                time.sleep(2)

                drl = drl_code_template + f"""
movel({command['pickup_position']}, v=150, a=90);
movejx({command['drop_offset1']}, sol=7);
movejx({command['drop_offset']}, sol=7);
movel({command['drop_position']});
"""
                self.run_drl_script(robot_system_mode, drl)
                time.sleep(2)
                self.wait_for_standby("After drop")

                self.gripper.open_gripper()
                time.sleep(2)

                self.total_boja -= 1
                msg = Int32()
                msg.data = self.total_boja
                self.remaining_boja_pub.publish(msg)

                drl = drl_code_template + f"""
movel({command['drop_offset']});
movejx({command['home_position']}, sol=7);
"""
                self.run_drl_script(robot_system_mode, drl)
                time.sleep(2)
                self.wait_for_standby("Return home")

                time.sleep(2)

            self.get_logger().info("All markers placed.")

        except Exception as e:
            self.get_logger().error(f"[process_markers] Exception: {e}")

        finally:
            with self.lock:
                self.is_running = False
            self.get_logger().info("[process_markers] DONE. is_running=False.")


def get_options():
    parser = argparse.ArgumentParser(description='Set options.')
    parser.add_argument('--gripper', dest='gripper', type=str, default="rg6", choices=['rg2', 'rg6'], help='Gripper type')
    parser.add_argument('--ip', dest='ip', type=str, default="192.168.1.1", help='IP address')
    parser.add_argument('--port', dest='port', type=str, default="502", help='Port number')
    return parser.parse_args()

def main():
    args = get_options()
    rclpy.init()
    node = DrlStartNode(args.gripper, args.ip, args.port)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
