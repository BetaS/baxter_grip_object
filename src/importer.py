import rospy
import baxter_interface

import numpy as np
import cv2, cv_bridge

class ClassProperty(property):
    def __get__(self, cls, owner):
        return self.fget.__get__(None, owner)()

from baxter_interface import CHECK_VERSION

import src.core.mathlib as mathlib
import src.core.rvizlib as rvizlib
import src.core.markerlib as markerlib

from src.core.baxter import Baxter

baxter = Baxter("py")