#!/usr/bin/env python3
"""Run TriFinger back-end using multi-process robot data."""
import argparse
import sys

# ROS imports
import rclpy
import rclpy.node
from std_msgs.msg import String
from std_srvs.srv import Empty

import robot_interfaces


class TriFingerDataNode(rclpy.node.Node):
    """Simple ROS node for communication with other processes."""

    def __init__(self):
        super().__init__("trifinger_data")
        self._status_publisher = self.create_publisher(String, "~/status", 1)

        self.shutdown_requested = False
        self._shutdown_srv = self.create_service(
            Empty, '~/shutdown', self.shutdown_callback
        )

    def shutdown_callback(self, request, response):
        self.shutdown_requested = True
        return response

    def publish_status(self, status: str):
        msg = String()
        msg.data = status
        self._status_publisher.publish(msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--max-number-of-actions",
        "-a",
        type=int,
        required=True,
        help="""Maximum numbers of actions that are processed.  After this the
            backend shuts down automatically.
        """,
    )
    camera_group = parser.add_mutually_exclusive_group()
    camera_group.add_argument(
        "--cameras",
        "-c",
        action="store_true",
        help="Run camera backend.",
    )
    camera_group.add_argument(
        "--cameras-with-tracker",
        action="store_true",
        help="Run camera backend with integrated object tracker.",
    )
    parser.add_argument(
        "--robot-logfile",
        type=str,
        help="""Path to a file to which the robot data log is written.  If not
            specified, no log is generated.
        """,
    )
    parser.add_argument(
        "--camera-logfile",
        type=str,
        help="""Path to a file to which the camera data is written.  If not
            specified, no log is generated.
        """,
    )
    args = parser.parse_args()

    rclpy.init()
    node = TriFingerDataNode()

    logger = node.get_logger()

    cameras_enabled = False
    if args.cameras:
        cameras_enabled = True
        from trifinger_cameras import tricamera
    elif args.cameras_with_tracker:
        cameras_enabled = True
        import trifinger_object_tracking.py_tricamera_types as tricamera

    if cameras_enabled:
        logger.info("Start camera data")

        # make sure camera time series covers at least one second
        CAMERA_TIME_SERIES_LENGTH = 15

        camera_data = tricamera.MultiProcessData(
            "tricamera", True, CAMERA_TIME_SERIES_LENGTH
        )

    logger.info("Start robot data")

    # Storage for all observations, actions, etc.
    history_size = args.max_number_of_actions + 1
    robot_data = robot_interfaces.trifinger.MultiProcessData(
        "trifinger", True, history_size=history_size
    )

    if args.robot_logfile:
        robot_logger = robot_interfaces.trifinger.Logger(robot_data)

    if cameras_enabled and args.camera_logfile:
        camera_fps = 10
        robot_rate_hz = 1000
        # make the logger buffer a bit bigger as needed to be on the safe side
        buffer_length_factor = 1.5

        episode_length_s = args.max_number_of_actions / robot_rate_hz
        # Compute camera log size based on number of robot actions plus a
        # 10% buffer
        log_size = int(camera_fps * episode_length_s * buffer_length_factor)

        logger.info("Initialize camera logger with buffer size %d" % log_size)
        camera_logger = tricamera.Logger(camera_data, log_size)

    logger.info("Data backend is ready")

    # send ready signal
    node.publish_status("READY")

    if cameras_enabled and args.camera_logfile:
        # FIXME backend.wait_until_first_action()
        camera_logger.start()
        logger.info("Start camera logging")

    while not node.shutdown_requested:
        logger.info("spin")  # TODO
        rclpy.spin_once(node)

    logger.debug("Received shutdown signal")

    if cameras_enabled and args.camera_logfile:
        logger.info(
            "Save recorded camera data to file %s" % args.camera_logfile
        )
        camera_logger.stop_and_save(args.camera_logfile)

    if args.robot_logfile:
        logger.info("Save robot data to file %s" % args.robot_logfile)
        if args.max_number_of_actions:
            end_index = args.max_number_of_actions
        else:
            end_index = -1

        robot_logger.write_current_buffer_binary(
            args.robot_logfile, start_index=0, end_index=end_index
        )

    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
