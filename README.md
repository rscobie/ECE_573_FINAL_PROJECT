# ECE_573_FINAL_PROJECT

# Installation Instructions

## install ROS noetic

follow instructions at this link: http://wiki.ros.org/Installation/Ubuntu

## install GDAL
`sudo apt-get install gdal-bin`

## install python dependencies
`pip install trimesh shapely OSMPythonTools elevation GDAL numpy networkx Pillow pyglet`

## initialize ROS workspace
To initialize the workspace, run: 

```
cd ~
mkdir -p ece573_ws/src
cd ece573_ws/src
catkin_init_workspace
cd ..
catkin_make
```

## clone repo

Now that the workspace is initialized, clone this repo into the ece573_ws/src directory

```
cd src
git clone https://github.com/rscobie/ECE_573_FINAL_PROJECT.git
```

## build (or rebuild) workspace

Build the package contained in the cloned repo by going back to the workspace root directory:
```
cd ../../
catkin_make
```

## source workspace

Assuming workspace is in the home directory, run
`source ~/ece573_ws/devel/setup.bash`

Add this to your .bashrc file as well to avoid having to do this every time
a new shell is opened. Now, our ECE_573_FINAL_PROJECT should be runnable using rosrun

# start nodes

make sure roscore is running first:
`roscore`

to launch the project in its entirety, run
`roslaunch ECE_573_FINAL_PROJECT dynamic_world.launch`

# run tests
run the python script `run_tests.py` found in the package's root directory

## usage
### gui node
Enter the desired Latitude and Longitude in the appropriate box, then press Start
### generation node
Publish a Float32MultiArray with format [latitude, longitude] to the chunk_coordinate topic. The output .obj file will be found under ECE_573_FINAL_PROJECT/cache with a name indicated its north, south, east, and west bounds in degrees. 

Note: originally colors were saved per-vertex. However, Gazebo doesn't support this. We are in the process of generating textures for the models instead, but this is a work in progress. As a result, the accompanying .mtl and .png files may cause some 3d viewers to reject the model. We use meshlab to preview generated models and have found it to open the models properly.
### simulation node
Publish a String with the path of the chunk to add to the simulation on the chunk_path topic. The chunk will then be loaded into gazebo

# Spawning Vehicle:
A launch file called vehicle_test.launch will spawn the vehicle in an empty world for testing purposes. Just type the command: roslaunch ECE_573_FINAL_PROJECT vehicle_test.launch to show the URDF model in gazebo. Currently movement, sensors and other implementations are being modified on the vehicle, which is why it is in this test world. Once the vehicle modifications are complete, it will be put in the dynamic world.

# simulations_v1.zip:
A zip file (version 1) that contains two packages, where one package is primarily used for now (simulation_node_gazebo). This package has sdf models of three main environments the group wants to test the vehicle in (neighborhood, city block, freeway). To use these packages, put them in any workspace of choice, in the src directory, then run a catkin_make on that workspace. Use roslaunch simulation_node_gazebo [launch file you want to run] to make gazebo spawn the world with the model area in that world. There are two test worlds and launch files that are just showing the beginnings of figuring out how to spawn models in gazebo. Test 1 could not be included in github because that test included a tutorial urdf model from the github repository that was too big to include. The world consisted of the urdf file, a generated model from gazebo and a chair model from 3DWarehouse to show that they could all be included together. test2.launch will just launch the 3DWarehouse chair model found to show that an sdf model can be put into gazebo. The other launch files are self explanatory, generating the name that's on the launch file into gazebo.

# simulations_v2.zip:
The Alpha Release simulation package which holds four worlds. Three of these worlds are test worlds for when a URDF vehicle is made. The osm_1.world is for a 1000x1000 chunk generated from OSM and put into gazebo as a model. The previous version had worlds that were not needed, so they were taken out in this version. One can spawn these test worlds by putting this package into a src folder of a workspace, running a catkin_make, and then typing roslaunch simulation_node_gazebo [name of launch file]. The video will focus on launching osm_1.launch to show that a world chunk was successfully created in gazebo. 

# simulations_v3.zip:
An updated simulation zip file that includes a simulation_node_description package that has a URDF vehicle model inside it. An extra test world and launch file called vehicle_test was also made to simulate the vehicle in an empty world. One can spawn these test worlds by putting this package into a src folder of a workspace, running a catkin_make, and then typing roslaunch simulation_node_gazebo [name of launch file].  
