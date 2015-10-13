import rospy
import baxter_interface

import numpy as np
import cv2, cv_bridge

from baxter_interface import CHECK_VERSION

from src.core.baxter import Baxter
import src.core.mathlib as mathlib
import src.core.markerlib as markerlib

baxter = Baxter("py")