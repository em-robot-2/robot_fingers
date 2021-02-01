#!/usr/bin/env python3
"""TODO"""
import enum
import time
import subprocess

# ROS imports
import rclpy
import rclpy.node
from std_msgs.msg import String
from std_srvs.srv import Empty


class ProcessState(enum.Enum):
    """Process state."""

    #: Terminated cleanly (i.e. returncode == 0)
    GOOD = 0

    #: Terminated with error (returncode != 0)
    BAD = 1

    #: Still running
    RUNNING = 2

    #: Terminated (no matter which returncode)
    GOOD_OR_BAD = 3


class ProcessStateCompareWrapper:
    """Wrapper around ProcessState to provide proper ``__eq__`` handling of GOOD_OR_BAD.

    Wraps around the ProcessState enum and provides a custom ``__eq__`` where
    ``GOOD == GOOD_OR_BAD`` and ``BAD == GOOD_OR_BAD`` evaluate to True.  This helps
    to simplify the state machine in cases where it only matters that a process has
    terminated, no matter if good or bad.
    """

    def __init__(self, state: ProcessState):
        self.state = state

    def __eq__(self, other):
        if self.state == other.state:
            return True
        elif self.state == ProcessState.GOOD_OR_BAD and other.state in (
            ProcessState.GOOD,
            ProcessState.BAD,
        ):
            return True
        elif other.state == ProcessState.GOOD_OR_BAD and self.state in (
            ProcessState.GOOD,
            ProcessState.BAD,
        ):
            return True
        else:
            return False

    def __repr__(self):
        # only print the plain state name
        return str(self.state.name)


class LauncherState:
    """Represents the combined state of all nodes monitored by the launcher."""

    def __init__(self, data_state, backend_state, user_state):
        self.data_state = data_state
        self.backend_state = backend_state
        self.user_state = user_state

    def __repr__(self):
        return "(Data: {}, Robot: {}, User: {})".format(
            self.data_state, self.backend_state, self.user_state
        )

    def __eq__(self, other):
        # allow comparison with tuple by converting it
        if type(other) is tuple:
            try:
                other = type(self)(*other)
            except Exception:
                return False

        return (
            (self.data_state == other.data_state)
            and (self.backend_state == other.backend_state)
            and (self.user_state == other.user_state)
        )


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

        # state constants to simplify the code below
        RUNNING = ProcessStateCompareWrapper(ProcessState.RUNNING)
        GOOD = ProcessStateCompareWrapper(ProcessState.GOOD)
        BAD = ProcessStateCompareWrapper(ProcessState.BAD)
        GOOD_OR_BAD = ProcessStateCompareWrapper(ProcessState.GOOD_OR_BAD)

        # some helper functions
        def get_state(poll_result):
            if poll_result is None:
                return RUNNING
            elif poll_result == 0:
                return GOOD
            else:
                return BAD

        def shutdown_data():
            self.get_logger().info("Shut down data node.")
            self.data_shutdown.call_async(Empty.Request())

        def shutdown_backend():
            self.get_logger().info("Shut down robot backend node.")
            self.backend_shutdown.call_async(Empty.Request())

        def terminate_user():
            self.get_logger().info("Kill user node.")
            user_node.kill()

        # monitor running nodes and handle shutdown using a state machine
        self.get_logger().info("Monitor nodes...")
        error = False
        previous_state = LauncherState(RUNNING, RUNNING, RUNNING)
        while True:
            time.sleep(3)

            # update state
            state = LauncherState(
                get_state(data_node.poll()),
                get_state(backend_node.poll()),
                get_state(user_node.poll()),
            )

            # only take action if state changes
            if state != previous_state:
                self.get_logger().info("State %s --> %s" % (previous_state, state))
                previous_state = state

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

                elif state in (
                    (BAD, GOOD, GOOD_OR_BAD),
                    (GOOD_OR_BAD, BAD, GOOD_OR_BAD),
                ):
                    # end with failure
                    error = True
                    break

                else:
                    raise RuntimeError("Unexpected state %s" % state)

        if error:
            self.get_logger().error("Finished with error.")
        else:
            self.get_logger().info("Done.")


def main(args=None):
    rclpy.init(args=args)

    node = TrifingerLauncherNode("trifinger_launcher")
    node.run()

    # cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
