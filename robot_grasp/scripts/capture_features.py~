#!/usr/bin/env python
import numpy as np
import pickle
import rospy

from pcl_helper import *
from training_helper import spawn_model
from training_helper import delete_model
from training_helper import initial_setup
from training_helper import capture_sample
from robot_grasp.srv import GetNormals
from robot_grasp.srv import GetVfh
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2

'''
'halfSphere',
'ShortCylinder',
'sixPrism',
'threePyramid',
'threePrism',
'sphere'
'''

NUMBER_OF_SPAWNS = 30
NUMBER_OF_ATTEMPTS = 5

def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

def get_vfh(cloud):
    get_vfh_prox = rospy.ServiceProxy('/feature_extractor/get_vfh', GetVfh)
    return get_vfh_prox(cloud).VFH_data

if __name__ == '__main__':
    rospy.init_node('capture_node')
    models = [\
        'cube',
        'cone',
        'cylinder',
        'fourPrism',
        'fourPrismoid',
        'fourPyramid',
        'halfSphere',
        'ShortCylinder',
        'sixPrism',
        'sphere',
        'threePrism',
        'threePyramid'
       ]
    initial_setup()
    labeled_features = []
    for model_name in models:
        spawn_model(model_name)
        for i in range(NUMBER_OF_SPAWNS):
            print(model_name, "#", i)
            sample_was_good = False
            try_count = 0
            while not sample_was_good and try_count < NUMBER_OF_ATTEMPTS:
                sample_cloud = capture_sample()
                sample_cloud_arr = ros_to_pcl(sample_cloud).to_array()
                # Check for invalid clouds.
                if sample_cloud_arr.shape[0] == 0:
                    print('Invalid cloud detected')
                    try_count += 1
                else:
                    sample_was_good = True
            vfhs = get_vfh(sample_cloud)
            feature = np.array(vfhs)
            labeled_features.append([feature, model_name])
        delete_model()
    pickle.dump(labeled_features, open('model.sav', 'wb'))

