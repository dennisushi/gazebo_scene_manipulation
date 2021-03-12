import rospy, rospkg
import gazebo_ros
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import GetModelState
#from gazebo_msgs.msg import WorldState

from geometry_msgs.msg import Pose, PoseStamped

import os
from glob import glob
"""
Spawn online model

rosrun gazebo_ros spawn_model -database coke_can -sdf -model coke_can -y 1.2 -x -0.0

Move existing model 

rosservice call /gazebo/set_model_state '{model_state: 
  { model_name: coke_can, 
    pose: { 
      position: { x: 0.3, y: 0.2 ,z: 0 }, 
      orientation: {x: 0, y: 0.491983115673, z: 0, w: 0.870604813099 } 
    }, 
    twist: { 
      linear: {x: 0.0 , y: 0 ,z: 0 } , 
      angular: { x: 0.0 , y: 0 , z: 0.0 } 
    } ,
    reference_frame: world } 
  }'

"""

"""
spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
spawn_model_client(
    model_name='ground_plane',
    model_xml=open('/usr/share/gazebo-9/models/ground_plane/model.sdf', 'r').read(),
    robot_namespace='/foo',
    initial_pose=Pose(),
    reference_frame='world'
)
"""

def InitPose():
  return


class GSM(object):
  def __init__(self, extra_model_dirs = []):
    self.init_model_dirs(extra_model_dirs)

    self.model_names = set()
    for mdir in self.models_dirs:
      subfolders = os.listdir(mdir)
      self.model_names |= set(subfolders)
    return

  def init_model_dirs(self, extra_model_dirs):
    self.models_dirs = []

    # Using the model directory passed by launch file
    # to allow easy integration with other nodes
    self.models_dir = rospy.get_param("~models_dir", "/home/$(optenv USER)/.gazebo/models/")
    self.models_dirs.append(self.models_dir)

    # Using the model directory passed by python file
    # to allow easy integration with new launch files
    self.models_dirs += extra_model_dirs

    # Using the default directories

    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    rospath = rospack.get_path("gazebo_scene_manipulation")
    pkg_models_path = rospath+"/models/"
    category_paths = [ path for path in glob(pkg_models_path+"/*") if os.path.isdir(path) ]
      
    self.models_dirs += category_paths

    rospy.loginfo("Model libraries used by gazebo_scene_manipulation: ")
    for mdir in self.models_dirs:
      rospy.loginfo(mdir)
    return

  def find_model(self, model_name):
    if model_name not in self.model_names:
      rospy.logwarn("Requested model_name %s is not found"%(model_name))
      return -1

    for mdir in self.models_dirs :
      glob_query  = mdir + "/"+model_name+"/*.sdf"
      model_paths = glob(glob_query)
      if len(model_paths)>0:
        break

    if len(model_paths)==0:
      rospy.logwarn("No .sdf found for model %s"%(model_name))
      return -1

    path_to_model = model_paths[0]

    return path_to_model


  def spawn(self, model_name, instance_name, pose):
    if model_name not in self.model_names:
      rospy.logwarn("Requested model_name %s is not found"%(model_name))
      return -1

    for mdir in self.models_dirs :
      glob_query  = mdir + "/"+model_name+"/*.sdf"
      model_paths = glob(glob_query)
      if len(model_paths)>0:
        break

    if len(model_paths)==0:
      rospy.logwarn("No .sdf found for model %s"%(model_name))
      return -1

    path_to_model = model_paths[0]

    rospy.loginfo("Spawning model of type %s"%model_name)

    sdff = open(path_to_model).read()

    rospy.wait_for_service('gazebo/spawn_sdf_model')
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_client(
        model_name=instance_name,
        model_xml=sdff,
        robot_namespace='/gsm',
        initial_pose=pose,
        reference_frame='world'
    )
    return 1

  def get_model_state(self, name, relname):
    get_model_state_srv = rospy.ServiceProxy('/gazebo/get_model_state',
                                              GetModelState)
    resp = get_model_state_srv(name, relname)
    return resp

  def despawn(self, instance_name):

    delete_model = rospy.ServiceProxy('/gazebo/delete_model', 
                                              DeleteModel)
    delete_model(instance_name)