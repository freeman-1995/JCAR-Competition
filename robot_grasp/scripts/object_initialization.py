#!/usr/bin/env python
import numpy as np
import pickle
import rospy
import roslib
from pcl_helper import *
from training_helper import delete_model
from training_helper import initial_setup
from training_helper import capture_sample
from training_helper import capture_image_sample
from training_helper import multi_obj_image_sample
from geometry_msgs.msg import Pose

NUMBER_OF_SPAWNS = 1
NUMBER_OF_ATTEMPTS = 5
combine_num = 12
index = 0
models = [\
    'cube',
    'cylinder',
    'cone',
    'fourPrism',
    'fourPrismoid',
    'fourPyramid',
    'halfSphere',
    'ShortCylinder',
    'sixPrism',
    'threePyramid',
   ]
if __name__ == '__main__':
    rospy.init_node('obj_init')
    initial_setup()
    labeled_features = []
    multi_obj_image_sample(models)
