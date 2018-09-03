import math
import random
import rospy
import rospkg
import tf
import time
from robot_grasp.srv import GetNormals
from gazebo_msgs.srv import GetPhysicsProperties
from gazebo_msgs.srv import SetPhysicsProperties
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetModelStateRequest
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image

models_dict = {\
   'cube':0.733392,
   'cylinder':0.745890,
   'cone':0.727391,
   'fourPrism':0.745890,
   'fourPrismoid':0.723033,
   'fourPyramid':0.726741,
   'halfSphere':0.717761,
   'ShortCylinder':0.733389,
   'sixPrism':0.745891,
   'threePyramid':0.727304
   }
models_training = {\
    'cube':'cube_training_model',
    'cylinder':'cylinder_training_model',
    'cone':'cone_training_model',
    'fourPrism':'fourPrism_training_model',
    'fourPrismoid':'fourPrismoid_training_model',
    'fourPyramid':'fourPyramid_training_model',
    'halfSphere':'halfSphere_training_model',
    'ShortCylinder':'ShortCylinder_training_model',
    'sixPrism':'sixPrism_training_model',
    'threePyramid':'threePyramid_training_model'
}
model_coord = {
    'cube':(0,0),
    'cylinder':(0,0),
    'cone':(0,0),
    'fourPrism':(0,0),
    'fourPrismoid':(0,0),
    'fourPyramid':(0,0),
    'halfSphere':(0,0),
    'ShortCylinder':(0,0),
    'sixPrism':(0,0),
    'threePyramid':(0,0)
}
'''
pos_x = random.uniform(0.144415, 0.396688)
pos_y = random.uniform(-0.157532, -0.314466)
'''
temp_models_dist={}
def get_kinect_pose():
    get_model_state_prox = rospy.ServiceProxy('gazebo/get_model_state',GetModelState)
    model_state = get_model_state_prox('kinect','world')
    kinect_pose = {model_state.pose.position.x,model_state.pose.position.y,model_state.pose.position.z,model_state.pose.orientation.x.model_state.pose.orientation.y,model_state.pose.orientation.z,model_state.pose.orientation.w}
    return kinect_pose

def calc_pos_x_y(index,pos_x,pos_y):
    for k in range(index):
        threshold = (temp_models_dist[str(k)][0]-pos_x)**2 + (temp_models_dist[str(k)][1]-pos_y)**2
        if threshold < 0.0038:
            return False
        else:
            print("threshold: %f",threshold)
    return True

def get_random_coor_list(model_name,index):
    pos_z = models_dict[model_name]
    while True:
        pos_x = random.uniform(0.1445, 0.3966)
        pos_y = random.uniform(-0.1576, -0.3144)
        if index == 0:
            temp_models_dist[str(index)] = (pos_x,pos_y,pos_z)
            break
        else:
            flag = calc_pos_x_y(index,pos_x,pos_y)
            if flag == True:
                temp_models_dist[str(index)] = (pos_x,pos_y,pos_z)
                model_coord[model_name] = (pos_x,pos_y)
                break
def capture_image_sample(obj_list):
    initial_pose = Pose()
    for k in range(len(temp_models_dist)):
        model_name = obj_list[k]
        print(model_name)
        initial_pose.position.x = temp_models_dist[str(k)][0]
        initial_pose.position.y = temp_models_dist[str(k)][1]
        initial_pose.position.z = temp_models_dist[str(k)][2]

        # Spawn the new model #
        model_path = rospkg.RosPack().get_path('robot_grasp')+'/models/grasp_object/'
        model_xml = ''

        with open (model_path + model_name + '/'+model_name+'.sdf', 'r') as xml_file:
            model_xml = xml_file.read().replace('\n', '')

        spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_prox(models_training[model_name], model_xml, '', initial_pose, 'world')
        get_model_state_prox = rospy.ServiceProxy('gazebo/get_model_state',GetModelState)
        model_state = get_model_state_prox(models_training[model_name],'world')
        set_model_state_prox = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
        #time.sleep(0.5)
    return rospy.wait_for_message('/camera/rgb/image_rect_color', Image)
def multi_obj_image_sample(obj_list):
    index = 0
    for obj_name in obj_list:
        get_random_coor_list(obj_name,index)
        index = index +1
    capture_image_sample(obj_list)
    temp_models_dist.clear()
    return rospy.wait_for_message('/camera/rgb/image_rect_color', Image)

def multi_obj_load(obj_list):
    index = 0
    for obj_name in obj_list:
        get_random_coor_list(obj_name,index)
        index = index +1
    capture_image_sample(obj_list)
    temp_models_dist.clear()

def capture_sample():
    get_model_state_prox = rospy.ServiceProxy('gazebo/get_model_state',GetModelState)
    model_state = get_model_state_prox('training_model','world')

    set_model_state_prox = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)

    roll = random.uniform(0, 2*math.pi)
    pitch = random.uniform(0, 2*math.pi)
    yaw = random.uniform(0, 2*math.pi)
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    model_state.pose.orientation.x = quaternion[0]
    model_state.pose.orientation.y = quaternion[1]
    model_state.pose.orientation.z = quaternion[2]
    model_state.pose.orientation.w = quaternion[3]
    sms_req = SetModelStateRequest()
    sms_req.model_state.pose = model_state.pose
    sms_req.model_state.twist = model_state.twist
    sms_req.model_state.model_name = 'training_model'
    sms_req.model_state.reference_frame = 'world'
    set_model_state_prox(sms_req)

    return rospy.wait_for_message('/camera/depth_registered/points', PointCloud2)


def initial_setup():
    rospy.wait_for_service('gazebo/get_model_state')
    rospy.wait_for_service('gazebo/set_model_state')
    rospy.wait_for_service('gazebo/get_physics_properties')
    rospy.wait_for_service('gazebo/set_physics_properties')
    rospy.wait_for_service('gazebo/spawn_sdf_model')

    get_physics_properties_prox = rospy.ServiceProxy('gazebo/get_physics_properties', GetPhysicsProperties)
    physics_properties = get_physics_properties_prox()
 
    physics_properties.gravity.z = 0

    set_physics_properties_prox = rospy.ServiceProxy('gazebo/set_physics_properties', SetPhysicsProperties)
    set_physics_properties_prox(physics_properties.time_step,
                                physics_properties.max_update_rate,
                                physics_properties.gravity,
                                physics_properties.ode_config)


    delete_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    delete_model_prox('ground_plane')

                 
def spawn_model(model_name,):
    initial_pose = Pose()
    initial_pose.position.x = 0.5
    initial_pose.position.y = 0
    initial_pose.position.z = 0.7

    model_path = rospkg.RosPack().get_path('training_station')+'/training_models/'
    model_xml = ''
    with open (model_path + model_name + '/'+model_name+'.sdf', 'r') as xml_file:
        model_xml = xml_file.read().replace('\n', '')
    spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_prox('training_model', model_xml, '', initial_pose, 'world')

def delete_model():
    delete_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    delete_model_prox('training_model')

