#!/usr/bin/env python3
"""Record data while the user is manually moving the robot."""
import argparse
import curses
import numpy as np

# %%


# %%

import robot_fingers
robot = robot_fingers.Robot.create_by_name('trifingerpro')
robot.initialize()


def record(trajectory):
    trajectory[:] = []
    t = 0
    while True:
        t = robot.frontend.append_desired_action(robot.Action())
        robot.frontend.wait_until_timeindex(t)
        trajectory += [robot.frontend.get_observation(t).position]

        if len(trajectory) > 3000:
            if np.abs(trajectory[-1] - trajectory[-2000]).sum() < 0.01:
                robot.frontend.append_desired_action(robot.Action())
                return


def play(trajectory):
    for position in trajectory:
        action = robot.Action(position=position)
        t = robot.frontend.append_desired_action(action)
        robot.frontend.wait_until_timeindex(t - 10)


if __name__ == "__main__":
    trajectory = []
    while True:
        print('enter key')
        key = input()
        if key == 'r':
            print('recording')
            record(trajectory)
        elif key == 'p':
            print('playing')
            play(trajectory)
        else:
            print('invalid key')
