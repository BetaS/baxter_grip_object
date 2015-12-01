import rospy
import baxter_interface

import baxter_kdl.kdl_kinematics as pykdl

import numpy as np
import cv2, cv_bridge

class ClassProperty(property):
    def __get__(self, cls, owner):
        return self.fget.__get__(None, owner)()

class Defines:
    LEFT = "left"
    RIGHT = "right"
    ARMS = [LEFT, RIGHT]
    ARMS_IDX = {LEFT: 0, RIGHT: 10}

from baxter_interface import CHECK_VERSION

import src.core.mathlib as mathlib
import src.core.graphiclib as graphiclib
import src.core.rvizlib as rvizlib
import src.core.markerlib as markerlib

from src.core.baxter import Baxter

baxter = Baxter("py")