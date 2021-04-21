import os
import subprocess
import rospy
from std_msgs.msg import Float32MultiArray, String

MODEL_DIR = "models"

def chunk_path_callback(path):
  print("callback")
  path = str(path.data)
  MODEL_NAME = path.split("/")
  MODEL_NAME = MODEL_NAME[-1]
  temp = path.split("/")
  temp.pop(-1)
  OLD_MODEL_DIR  = "/".join(temp)
  COORDINATES = MODEL_NAME[:MODEL_NAME.index(".obj")] #get everything to left of .obj substring

  MAT_NAME = "material0.mtl"
  TEX_NAME = "material0.png"

  sdf_text = f"""<?xml version="1.0"?>
  <sdf version="1.5">
    <model name="osm_1">
      <static>true</static>
      <link name="l1">
        <collision name="c1">
          <geometry>
            <box>
              <size>0 0 0</size>
            </box>
          </geometry>
        </collision>     
        <visual name="m1">
          <cast_shadows>true</cast_shadows>
          <geometry>
            <mesh>
              <uri>model://{MODEL_NAME}</uri>
              <scale>1000 1000 1000</scale>
            </mesh>
          </geometry>
        </visual>
      </link>
    </model>
  </sdf>

  """

  #where generation node saves models
  OLD_MESH_PATH = path#os.path.dirname(os.path.realpath(__file__)) + "/" + "cache"
  #where gazebo wants them to be
  NEW_MESH_PATH = os.path.dirname(os.path.realpath(__file__)) + "/" + "models"
  #move mesh file over
  os.system(f"cp {OLD_MESH_PATH} {NEW_MESH_PATH}/{MODEL_NAME}")
  #also copy material/texture over
  os.system(f"cp {OLD_MODEL_DIR}/{MAT_NAME} {NEW_MESH_PATH}/{MAT_NAME}")
  os.system(f"cp {OLD_MODEL_DIR}/{TEX_NAME} {NEW_MESH_PATH}/{TEX_NAME}")
  #make sdf file
  sdf_file = open(f"{NEW_MESH_PATH}/{COORDINATES}.sdf", "w")
  sdf_file.write(sdf_text)
  sdf_file.close()
  #can repeatedly call this command with new model names at runtime
  os.system(f"rosrun gazebo_ros spawn_model -sdf -file {NEW_MESH_PATH}/{COORDINATES}.sdf -model {MODEL_NAME}")

if __name__ == "__main__":
    subscriber = rospy.Subscriber("chunk_path", String, chunk_path_callback, queue_size=10) #1d arrays of size 2
    rospy.init_node("simulation_node")

    #this runs as subprocess in background
    subprocess.run(["roslaunch", "gazebo_ros", "empty_world.launch"])