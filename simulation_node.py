import os
import subprocess
import rospy
from std_msgs.msg import Float32MultiArray, String

import networkx as nx

MODEL_DIR = "models"

first_chunk = True

def chunk_path_callback(chunk_name):
    global first_chunk
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
                        <mesh>
                            <uri>model://chunks/{chunk_name}/{MODEL_NAME}</uri>
                            <scale>1000 1000 1000</scale>
                        </mesh>
                    </geometry>
                        <surface>
                            <friction>
                                <ode>
                                    <mu>0.99</mu>
                                    <mu2>0.99</mu2>
                                </ode>
                            </friction>
                        </surface>
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

    #read road graph
    road_graph = nx.readwrite.gpickle.read_gpickle(f"{NEW_MESH_PATH}/{chunk_name}.roads")

    #make sdf file
    sdf_file = open(f"{NEW_MESH_PATH}/{chunk_name}.sdf", "w")
    sdf_file.write(sdf_text)
    sdf_file.close()
    
    #can repeatedly call this command with new model names at runtime
    os.system(f"rosrun gazebo_ros spawn_model -sdf -file {NEW_MESH_PATH}/{chunk_name}.sdf -model {MODEL_NAME}")

    #add car if this is the first time a spawn occurs
    if first_chunk:

        #get starting location
        start = list(road_graph.nodes)
        if len(start) != 0:
            start = start[0]
            start = list((x*1000 for x in start))#convert units from km to m
        else:
            print("no road to spawn car in this chunk")
            return
        first_chunk = False
        URDF_PATH = os.path.dirname(os.path.realpath(__file__)) + "/" + "urdf"
        os.system(f"rosrun gazebo_ros spawn_model -file {URDF_PATH}/prius.urdf -urdf -x {start[0]} -y {start[1]} -z {start[2]+1} -model prius")
        os.system("roslaunch ECE_573_FINAL_PROJECT car.launch")
        os.system("gz camera -c gzclient_camera -f prius")

if __name__ == "__main__":
    subscriber = rospy.Subscriber("chunk_path", String, chunk_path_callback, queue_size=10) #1d arrays of size 2
    rospy.init_node("simulation_node")
    
    #reset gazebo if it's already running
    os.system('rosservice call /gazebo/reset_simulation "{}"')
    #this runs as subprocess in background
    subprocess.run(["roslaunch", "gazebo_ros", "empty_world.launch"])