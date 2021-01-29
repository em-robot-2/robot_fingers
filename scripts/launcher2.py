#!/usr/bin/env python3
"""Experimental launcher node."""
import time
import subprocess

# ROS imports
import rclpy
import rclpy.node
from std_msgs.msg import String
from std_srvs.srv import Empty


class DataRunner:

    def __init__(self, node):
        self.is_ready = False

        self._logger = node.get_logger()

        self._sub_status = node.create_subscription(
            String,
            '/trifinger_data/status',
            self._status_callback,
            10,
        )
        self._sub_status  # prevent unused variable warning

    def _status_callback(self, msg):
        if msg.data == "READY":
            self.is_ready = True
            self._logger.info("Data node is ready")

    def start(self):
        ...

    def request_shutdown(self):
        # FIXME
        ...

    def kill(self):
        # TODO enough?
        self._proc.kill()

    def is_running(self):
        self.returncode = self._proc.poll()
        return self.returncode is None


class BackendRunner:

    def __init__(self, node):
        self.is_ready = False

        self._logger = node.get_logger()

        self._sub_status = node.create_subscription(
            String,
            '/trifinger_backend/status',
            self._status_callback,
            10,
        )
        self._sub_status  # prevent unused variable warning

    def _status_callback(self, msg):
        if msg.data == "READY":
            self.is_ready = True
            self._logger.info("Backend node is ready")

    def start(self, first_action_timeout: int, episode_length: int):
        self._logger.info("Launch backend node.")
        cmd = [
            "python3",
            "./trifinger_robot_backend.py",
            "--max-number-of-actions",
            str(episode_length),
            "--first-action-timeout",
            str(first_action_timeout),
        ]
        self._proc = subprocess.Popen(cmd)

    def request_shutdown(self):
        # FIXME
        ...

    def terminate(self):
        self.request_shutdown()
        self.wait(30)
        if self.is_running():
            self._logger.error("Backend did not shut down in time.  Kill it.")
            self.kill()

    def kill(self):
        # TODO kill process group
        self._proc.kill()
        # TODO wait

    def is_running(self):
        self.returncode = self._proc.poll()
        return self.returncode is None


class TrifingerLauncherNode(rclpy.node.Node):
    """TODO"""

    def __init__(self, name):
        super().__init__(name)

        self._shutdown_srv = self.create_service(
            Empty, 'shutdown', self._shutdown_callback
        )

    def _shutdown_callback(self, request, response):
        # TODO request nodes to shutdown
        return response

    def run(self):
        max_number_of_actions = 20000

        data_node = DataRunner(self)
        backend_node = BackendRunner(self)

        # sleep to give subscribers some time (not sure if this is still needed
        # in ROS 2)
        time.sleep(3)

        # run data node
        data_node.start()
        # wait for data node to be ready
        while not data_node.is_ready:
            if not data_node.is_running():
                raise RuntimeError("Data node failed.")
            rclpy.spin_once(self, timeout_sec=3)

        # run backend node
        backend_node.start()
        # wait for backend node to be ready
        while not backend_node.is_ready:
            if not backend_node.is_running():
                raise RuntimeError("Backend failed.")
            rclpy.spin_once(self, timeout_sec=3)

        # run user code
        self.get_logger().info("Run user code")
        # FIXME
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

            if user_node.poll() is not None:
                self.get_logger().info("User node terminated unexpectedly.")

                backend_node.terminate()
                data_node.terminate()

                # TODO: this is an error case, report this

                break

            if not data_node.is_running():
                self.get_logger().error("Data node terminated unexpectedly.")
                backend_node.terminate()
                user_node.kill()

                # TODO: this is an error case, report this

                break

            if not backend_node.is_running():
                if backend_node.returncode == 0:
                    # all good, clean shutdown
                    self.get_logger().info("Backend node terminated.")

                    # FIXME this should actually first wait a bit and then
                    # terminate the user code before the data node is killed
                    self.get_logger().info("Shut down data node.")
                    time.sleep(5)
                    data_node.kill()  # FIXME request shutdown via service
                    break

                else:
                    self.get_logger().error("Backend node terminated with code %d.",
                                            backend_node.returncode)

                # Give the user code some time to wrap up (e.g. save logs)
                # TODO do a user_node.wait() here
                time.sleep(30)
                user_node.kill()

                data_node.terminate()


        self.get_logger().info("Done.")


def main(args=None):
    rclpy.init(args=args)

    node = TrifingerLauncherNode("trifinger_launcher")
    node.run()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
