#!/usr/bin/env python3
"""TODO"""
import time
import subprocess

# ROS imports
import rclpy
import rclpy.node
from std_msgs.msg import String
from std_srvs.srv import Empty


class TrifingerLauncherNode(rclpy.node.Node):
    """TODO"""

    def __init__(self, name):
        super().__init__(name)

        self.data_node_ready = False
        self.backend_node_ready = False

        self._shutdown_srv = self.create_service(
            Empty, 'shutdown', self._shutdown_callback
        )

        self._sub_data_node_status = self.create_subscription(
            String,
            '/trifinger_data/status',
            self._data_node_status_callback,
            10,
        )
        self._sub_data_node_status  # prevent unused variable warning

        self._sub_backend_node_status = self.create_subscription(
            String,
            '/trifinger_backend/status',
            self._backend_node_status_callback,
            10,
        )
        self._sub_data_node_status  # prevent unused variable warning

        self.data_shutdown = self.create_client(
            Empty, '/trifinger_data/shutdown')
        self.backend_shutdown = self.create_client(
            Empty, '/trifinger_backend/shutdown')

    def _data_node_status_callback(self, msg):
        if msg.data == "READY":
            self.data_node_ready = True
            self.get_logger().info("Data node is ready")

    def _backend_node_status_callback(self, msg):
        if msg.data == "READY":
            self.backend_node_ready = True
            self.get_logger().info("Backend node is ready")

    def _shutdown_callback(self, request, response):
        # TODO request nodes to shutdown
        return response

    def run(self):
        max_number_of_actions = 20000

        # run data node
        self.get_logger().info("Launch data node.")
        cmd = [
            "python3",
            "./trifinger_data_backend.py",
            "--max-number-of-actions",
            str(max_number_of_actions),
        ]
        data_node = subprocess.Popen(cmd)

        # wait for data node to be ready
        while not self.data_node_ready:
            rclpy.spin_once(self, timeout_sec=3)

        # run backend node
        self.get_logger().info("Launch backend node.")
        cmd = [
            "python3",
            "./trifinger_robot_backend.py",
            "--max-number-of-actions",
            str(max_number_of_actions),
        ]
        backend_node = subprocess.Popen(cmd)

        # wait for backend node to be ready
        while not self.backend_node_ready:
            if backend_node.poll() is not None:
                raise RuntimeError("Backend failed.")
            rclpy.spin_once(self, timeout_sec=3)

        # run user code
        self.get_logger().info("Run user code")
        cmd = [
            "ros2",
            "run",
            "robot_fingers",
            "demo_trifingerpro",
            "--multi",
        ]
        user_node = subprocess.Popen(cmd)

        # monitor running nodes
        self.get_logger().info("Monitor nodes...")
        while True:
            time.sleep(3)

            if data_node.poll() is not None:
                self.get_logger().info("Data node terminated.")

                # TODO better tear down?
                self.backend_shutdown.call_async(Empty.Request())
                user_node.kill()

                break

            if backend_node.poll() is not None:
                self.get_logger().info("Backend node terminated.")
                # FIXME this should actually first wait a bit and then
                # terminate the user code before the data node is killed
                time.sleep(5)
                self.get_logger().info("Kill user node.")
                user_node.kill()
                self.get_logger().info("Shut down data node.")
                self.data_shutdown.call_async(Empty.Request())
                break

            if user_node.poll() is not None:
                self.get_logger().warning("User code terminated")
                # call backend shutdown service
                self.backend_shutdown.call_async(Empty.Request())
                time.sleep(10)
                self.data_shutdown.call_async(Empty.Request())
                break

        self.get_logger().info("Done.")


def main(args=None):
    rclpy.init(args=args)

    node = TrifingerLauncherNode("trifinger_launcher")
    node.run()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
