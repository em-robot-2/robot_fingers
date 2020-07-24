#!/usr/bin/env python3
"""Record data while the user is manually moving the robot."""
import argparse
import curses

# %%


# %%

import robot_fingers
robot = robot_fingers.Robot.create_by_name('trifingerpro')
robot.initialize()


def record(trajectory):
    trajectory[:] = []
    t = 0
    for _ in range(10**4):
        t = robot.frontend.append_desired_action(robot.Action())
        robot.frontend.wait_until_timeindex(t)
        trajectory += [robot.frontend.get_observation(t).position]
        

def play(trajectory):
    for position in trajectory:    
        action = robot.Action(position=position)
        robot.frontend.append_desired_action(action)


if __name__ == "__main__":
    trajectory = []
    while True:
        print('enter key')
        key = input()
        if key == 'r':
            print('recording')
            record(trajectory)
            print(trajectory)
        elif key == 'p':
            print('playing')
            play(trajectory)
        else:
            print('invalid key')
    

