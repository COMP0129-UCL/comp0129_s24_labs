#!/usr/bin/env python3

import rospy, rospkg
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState, GetWorldProperties
from geometry_msgs.msg import Pose

import os
from glob import glob
from time import sleep

def vectors_to_ros_pose(pos, q):
  pose = Pose()
  pose.position.x = pos[0]
  pose.position.y = pos[1]
  pose.position.z = pos[2]
  pose.orientation.x = q[0]
  pose.orientation.y = q[1]
  pose.orientation.z = q[2]
  pose.orientation.w = q[3]
  return pose

class Model(object):
  def __init__(self, model_name, instance_name, position, 
               orientation=[0,0,0,1], scale=None):
    self.mdict = self.create_model_dict(model_name, instance_name, position, 
                                        orientation, scale)
    return

  def create_model_dict(self, model_name, instance_name, position,
                        orientation=[0,0,0,1], scale=None):
    pose = vectors_to_ros_pose(position, orientation)
    mdict = dict(mtype = "sdf", model_name = model_name, 
                instance_name=instance_name, pose=pose)
    return mdict

  def get_model_state(self, relname="world"):
    get_model_state_srv = rospy.ServiceProxy('/gazebo/get_model_state',
                                              GetModelState)
    resp = get_model_state_srv(self.mdict["instance_name"], relname)
    return resp

  def despawn(self):
    delete_model = rospy.ServiceProxy('/gazebo/delete_model', 
                                              DeleteModel)
    delete_model(self.mdict["instance_name"])

    
class WorldSpawner(object):
  def __init__(self, model_directory):
    self.models_dirs = [model_directory]
    self.init_model_names()
    return

  def init_model_names(self):
    self.model_names = set()
    for mdir in self.models_dirs:
      subfolders = os.listdir(mdir)
      self.model_names |= set(subfolders)
    return

  def find_model(self, model_name):
    if model_name not in self.model_names:
      rospy.logwarn("Requested model_name %s is not found"%(model_name))
      return None

    # Search for model name in all model directories
    for mdir in self.models_dirs :
      glob_query  = mdir + "/"+model_name+"/*.sdf"
      model_paths = glob(glob_query)
      if len(model_paths)>0:
        break 
    else:  # For/else clause means that the loop has finished without breaking
      rospy.logwarn("No .sdf found for model %s"%(model_name))
      return None

    path_to_model = model_paths[0]

    return path_to_model

  def spawn(self, model):
    model_dict = model.mdict
    if model_dict['mtype'] == 'sdf':
      model_name = model_dict['model_name']
      instance_name = model_dict['instance_name']
      pose = model_dict['pose']
      return self.spawn_model(model_name, instance_name, pose)
    elif model_dict['mtype'] == 'primitive':
      pass
      return False
    else:
      rospy.logerr("Unsupported model spawn type %s"%model_dict['mtype'])
      return False

  def spawn_model(self, model_name, instance_name, pose):
    path_to_model =  self.find_model(model_name)
    if path_to_model is None:
      return False

    rospy.logdebug("Spawning model of type %s"%model_name)
    sdff = open(path_to_model).read()

    rospy.wait_for_service('gazebo/spawn_sdf_model')
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_client(
        model_name=instance_name,
        model_xml=sdff,
        robot_namespace='/WorldSpawner',
        initial_pose=pose,
        reference_frame='world'
    )
    return True

  def get_model_state_by_name(self, name, relname="world"):
    get_model_state_srv = rospy.ServiceProxy('/gazebo/get_model_state',
                                              GetModelState)
    resp = get_model_state_srv(name, relname)
    return resp

  def despawn_by_name(self, instance_name):
    delete_model = rospy.ServiceProxy('/gazebo/delete_model', 
                                              DeleteModel)
    resp = delete_model(instance_name)
    return resp
  
  def get_world_properties(self):
    world_props = rospy.ServiceProxy('/gazebo/get_world_properties', 
                                      GetWorldProperties)
    resp = world_props()
    return resp

  def despawn_all(self, keyword = 'object', exceptions ='exception'):
    props = self.get_world_properties().model_names
    [ self.despawn_by_name(name) for name in props if keyword in name and exceptions not in name] 
    return
    
if __name__ == "__main__":

  # sleep to give gazebo time to load
  sleep(5)

  # get an instance of RosPack with the default search paths
  rospack = rospkg.RosPack()
  rospath = rospack.get_path("moveit_tutorial")
  pkg_models_path = rospath + "/spawn_gazebo_model/models/"

  # spawn a cube into gazebo at x=0.6, y=0
  world_spawner = WorldSpawner(pkg_models_path)
  model = Model(model_name="cube", instance_name="cube", position=[0.6, 0, 0.1])
  world_spawner.spawn(model)

  rospy.loginfo(
    "Cube (side length 40mm) successfully spawned at x = 0.6m, y = 0.0m"
  )
