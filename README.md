# ECE_573_FINAL_PROJECT

# Installation Instructions

## install ROS noetic

follow instructions at this link: http://wiki.ros.org/Installation/Ubuntu

## install GDAL
`sudo apt-get install gdal-bin`

## install python dependencies
`pip install trimesh shapely OSMPythonTools elevation GDAL numpy networkx Pillow`

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

## start nodes

make sure roscore is running first:
`roscore`

TODO: add launch file to do all of this
For now, to start generation node:

`rosrun ECE_573_FINAL_PROJECT osm_to_building.py`

To test out this node, run

`rostopic echo -c chunk_path`

in one one shell, then

`rostopic pub /chunk_coordinate std_msgs/Float32MultiArray "{layout: {}, data: [43.7393, 7.4202]}" -1`

in another shell.


# simulations_v1.zip:
A zip file (version 1) that contains two packages, where one package is primarily used for now (simulation_node_gazebo). This package has sdf models of three main environments the group wants to test the vehicle in (neighborhood, city block, freeway). To use these packages, put them in any workspace of choice, in the src directory, then run a catkin_make on that workspace. Use roslaunch simulation_node_gazebo [launch file you want to run] to make gazebo spawn the world with the model area in that world. There are two test worlds and launch files that are just showing the beginnings of figuring out how to spawn models in gazebo. Test 1 could not be included in github because that test included a tutorial urdf model from the github repository that was too big to include. The world consisted of the urdf file, a generated model from gazebo and a chair model from 3DWarehouse to show that they could all be included together. test2.launch will just launch the 3DWarehouse chair model found to show that an sdf model can be put into gazebo. The other launch files are self explanatory, generating the name that's on the launch file into gazebo.

# simulations_v2.zip:
The Alpha Release simulation package which holds four worlds. Three of these worlds are test worlds for when a URDF vehicle is made. The osm_1.world is for a 1000x1000 chunk generated from OSM and put into gazebo as a model. The previous version had worlds that were not needed, so they were taken out in this version. One can spawn these test worlds by putting this package into a src folder of a workspace, running a catkin_make, and then typing roslaunch simulation_node_gazebo [name of launch file]. The video will focus on launching osm_1.launch to show that a world chunk was successfully created in gazebo. 
