from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class ServoStartClient(Node):
    def __init__(self) -> None:
        super().__init__("servo_start_client_node")
        self.declare_parameter("service_name", "/servo_node/start_servo")
        self.declare_parameter("timeout_sec", 30.0)

        self._service_name = str(self.get_parameter("service_name").value)
        self._timeout_sec = float(self.get_parameter("timeout_sec").value)

        self._cli = self.create_client(Trigger, self._service_name)
        self._timer = self.create_timer(0.2, self._tick)
        self._sent = False

        self.get_logger().info(f"Will call Trigger service: {self._service_name}")

    def _tick(self) -> None:
        if self._sent:
            return
        if not self._cli.wait_for_service(timeout_sec=0.0):
            return

        req = Trigger.Request()
        future = self._cli.call_async(req)
        self._sent = True

        def _done_cb(fut):
            try:
                resp = fut.result()
                if resp is None:
                    self.get_logger().error("start_servo: no response")
                else:
                    self.get_logger().info(f"start_servo: success={resp.success} message='{resp.message}'")
            except Exception as e:
                self.get_logger().error(f"start_servo call failed: {e}")
            # Exit after the first attempt (success or failure)
            rclpy.shutdown()

        future.add_done_callback(_done_cb)


def main() -> None:
    rclpy.init()
    node = ServoStartClient()
    # Optional watchdog timeout: if never got service, exit
    node.create_timer(node._timeout_sec, lambda: rclpy.shutdown())
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


