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
            Empty, "shutdown", self._shutdown_callback
        )

        self._sub_data_node_status = self.create_subscription(
            String,
            "/trifinger_data/status",
            self._data_node_status_callback,
            10,
        )
        self._sub_data_node_status  # prevent unused variable warning

        self._sub_backend_node_status = self.create_subscription(
            String,
            "/trifinger_backend/status",
            self._backend_node_status_callback,
            10,
        )
        self._sub_data_node_status  # prevent unused variable warning

        self.data_shutdown = self.create_client(Empty, "/trifinger_data/shutdown")
        self.backend_shutdown = self.create_client(Empty, "/trifinger_backend/shutdown")

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


def state_machine_draft():
    import copy
    import enum

    # to avoid warnings
    def shutdown_data():
        pass

    def shutdown_backend():
        pass

    def terminate_user():
        pass

    class State(enum.Enum):
        """Process state."""

        #: Terminated cleanly (i.e. returncode == 0)
        GOOD = 0

        #: Terminated with error (returncode != 0)
        BAD = 1

        #: Still running
        RUNNING = 2

        #: Terminated (no matter which returncode)
        GOOD_OR_BAD = 3

    class ComparableState:
        """Wrapper around State to provide proper ``__eq__`` handling of GOOD_OR_BAD.

        Wraps around the State enum and provides a custom ``__eq__`` where
        ``GOOD == GOOD_OR_BAD`` and ``BAD == GOOD_OR_BAD`` evaluate to True.  This helps
        to simplify the state machine in cases where it only matters that a process has
        terminated, no matter if good or bad.
        """

        def __init__(self, state: State):
            self.state = state

        def __eq__(self, other):
            if self.state == other.state:
                return True
            elif self.state == State.GOOD_OR_BAD and other.state in (
                State.GOOD,
                State.BAD,
            ):
                return True
            elif other.state == State.GOOD_OR_BAD and self.state in (
                State.GOOD,
                State.BAD,
            ):
                return True
            else:
                return False

        def __str__(self):
            # hide the wrapper when converting to string
            return str(self.state)

    RUNNING = ComparableState(State.RUNNING)
    GOOD = ComparableState(State.GOOD)
    BAD = ComparableState(State.BAD)
    GOOD_OR_BAD = ComparableState(State.GOOD_OR_BAD)

    # initially all processes are running
    data_state = backend_state = user_state = RUNNING

    error = False
    last_state = None
    while True:
        time.sleep(3)
        state = copy.copy((data_state, backend_state, user_state))

        # only take action if state changes
        if state != last_state:
            print("State %s --> %s" % (last_state, state))
            last_state = state

            if state == (RUNNING, RUNNING, RUNNING):
                pass

            elif state == (RUNNING, RUNNING, GOOD_OR_BAD):
                shutdown_backend()

            elif state == (RUNNING, GOOD, RUNNING):
                time.sleep(10)
                terminate_user()

            elif state == (RUNNING, GOOD, GOOD_OR_BAD):
                shutdown_data()

            elif state == (RUNNING, BAD, RUNNING):
                error = True
                time.sleep(10)
                terminate_user()

            elif state == (RUNNING, BAD, GOOD_OR_BAD):
                error = True
                shutdown_data()

            elif state == (GOOD_OR_BAD, RUNNING, RUNNING):
                error = True
                terminate_user()

            elif state == (GOOD_OR_BAD, RUNNING, GOOD_OR_BAD):
                error = True
                shutdown_backend()

            elif state == (GOOD_OR_BAD, GOOD_OR_BAD, RUNNING):
                error = True
                terminate_user()

            # terminal states

            elif state == (GOOD, GOOD, GOOD_OR_BAD):
                # end with success :)
                break

            elif state in ((BAD, GOOD, GOOD_OR_BAD), (GOOD_OR_BAD, BAD, GOOD_OR_BAD)):
                # end with failure
                error = True
                break

            else:
                raise RuntimeError("Unexpected state %s" % state)

    print(error)


def main(args=None):
    rclpy.init(args=args)

    node = TrifingerLauncherNode("trifinger_launcher")
    node.run()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
