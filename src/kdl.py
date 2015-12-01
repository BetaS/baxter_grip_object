
from src.importer import *

import math

def main():
    joints = baxter._arms[Defines.LEFT].get_joint_angle()
    th, dist = robotlib.inv_kin("left", joints, [0.8, 0.2, 0.0], [math.pi, 0, 0])

    print dist

    baxter._arms[Defines.LEFT].set_joint_angle(th, time=10000)

import time
if __name__ == "__main__":
    baxter.start(False)
    main()
