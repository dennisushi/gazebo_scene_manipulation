#!/usr/bin/python

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

from gazebo_scene_manipulation.gazebo_scene_manipulation import *
from fusion_server.srv import CaptureScene

import os, sys
from glob import glob
import numpy as np

rospy.init_node('gsm_wrapper')


# Area
# x -> [ 0.45, 0.95]
# y -> [-0.45, 0.45]
# z -> [0.3]

def upright_pose(x,y,z, q):
  pose = Pose()
  pose.position.x = x
  pose.position.y = y
  pose.position.z = z
  pose.orientation.x = q[0]
  pose.orientation.y = q[1]
  pose.orientation.z = q[2]
  pose.orientation.w = q[3]
  return pose

def random_pose_in_area(xlims, ylims, zlims,
                        roll_lims, pitch_lims, yaw_lims):
  x = np.random.uniform(xlims[0], xlims[1])
  y = np.random.uniform(ylims[0], ylims[1])
  z = np.random.uniform(zlims[0], zlims[1])
  roll  = np.random.uniform(roll_lims[0], roll_lims[1])
  pitch = np.random.uniform(pitch_lims[0], pitch_lims[1])
  yaw   = np.random.uniform(yaw_lims[0], yaw_lims[1])
  q = quaternion_from_euler(roll, pitch, yaw)
  pose = upright_pose(x,y,z, q)
  return pose

xlims = [0.6,0.90]
ylims = [-0.35,0.35]
zlims = [0.15,0.25]
roll_lims  = [-np.pi, np.pi]
pitch_lims = [-np.pi, np.pi]
yaw_lims   = [-np.pi, np.pi]

models = ["coke_can", "beer"]

models = ["donut_frost_1", "coffee_box",
          "can_sprite", "can_coke", "can_fanta", "can_pepsi",
          "tea_box", "camera_fujica", "light", "mouse",
          "book_1", "book_2","book_3","book_4",
          "glue_bottle"]



gsm = GSM()

for i in range(100):
  model_name = np.random.choice(models)
  instance_name = "object"

  pose = random_pose_in_area(xlims, ylims, zlims,
                        roll_lims, pitch_lims, yaw_lims)

  success = gsm.spawn(model_name,
                      instance_name,
                      pose)
                       
  if success:
    rospy.sleep(rospy.Duration(3))
    rospy.wait_for_service('capture_scene')
    capture_scene = rospy.ServiceProxy('capture_scene', CaptureScene)
    try:
      bagpath = capture_scene().bag_filepath
      rospy.loginfo("Scene captured in "+bagpath)
      # Where the bag is stored
      dirname = os.path.dirname(bagpath)
      with open(dirname+"/scene_context.yaml","wb") as f:
        f.write("object_name: "+model_name)

    except rospy.ServiceException as exc:
      rospy.logerr("Service did not process request: " + str(exc))

  rospy.sleep(rospy.Duration(1))
  gsm.despawn(instance_name)
  rospy.sleep(rospy.Duration(1))

