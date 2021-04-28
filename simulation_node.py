import os
import subprocess
import rospy
from std_msgs.msg import Float32MultiArray, String

MODEL_DIR = "models"

def chunk_path_callback(chunk_name):
  chunk_name = str(chunk_name.data)
  print("simulation node received: " + chunk_name)
  MODEL_NAME = chunk_name + ".obj"

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
              <uri>model://chunks/{chunk_name}/{MODEL_NAME}</uri>
              <scale>1000 1000 1000</scale>
            </mesh>
          </geometry>
        </visual>
      </link>
    </model>
  </sdf>

  """

  #where gazebo wants them to be
  NEW_MESH_PATH = os.path.dirname(os.path.realpath(__file__)) + f"/models/chunks/{chunk_name}"
  #move mesh file over
  # os.system(f"cp {OLD_MESH_PATH} {NEW_MESH_PATH}/{MODEL_NAME}")
  #also copy material/texture over #TODO: commented out for now while texture generation WIP
  # os.system(f"cp {OLD_MODEL_DIR}/{MAT_NAME} {NEW_MESH_PATH}/{MAT_NAME}")
  # os.system(f"cp {OLD_MODEL_DIR}/{TEX_NAME} {NEW_MESH_PATH}/{TEX_NAME}")
  #make sdf file
  sdf_file = open(f"{NEW_MESH_PATH}/{chunk_name}.sdf", "w")
  sdf_file.write(sdf_text)
  sdf_file.close()
  #can repeatedly call this command with new model names at runtime
  os.system(f"rosrun gazebo_ros spawn_model -sdf -file {NEW_MESH_PATH}/{chunk_name}.sdf -model {MODEL_NAME}")

if __name__ == "__main__":
    subscriber = rospy.Subscriber("chunk_path", String, chunk_path_callback, queue_size=10) #1d arrays of size 2
    rospy.init_node("simulation_node")

    #add car
    # URDF_PATH = os.path.dirname(os.path.realpath(__file__)) + "/" + "urdf"
    # os.system(f"rosrun gazebo_ros spawn_model -file {URDF_PATH}/prius.urdf -urdf -z 1 -model prius")

    #this runs as subprocess in background
    subprocess.run(["roslaunch", "gazebo_ros", "empty_world.launch"])