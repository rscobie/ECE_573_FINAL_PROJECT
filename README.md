# ECE_573_FINAL_PROJECT

# Installation Instructions
## install GDAL
`sudo apt-get install gdal-bin`

## python dependencies
`pip install trimesh, shapely, OSMPythonTools, elevation, GDAL, numpy`

# simulations_v1.zip:
A zip file (version 1) that contains two packages, where one package is primarily used for now (simulation_node_gazebo). This package has sdf models of three main environments the group wants to test the vehicle in (neighborhood, city block, freeway). To use these packages, put them in any workspace of choice, in the src directory, then run a catkin_make on that workspace. Use roslaunch simulation_node_gazebo [launch file you want to run] to make gazebo spawn the world with the model area in that world. There are two test worlds and launch files that are just showing the beginnings of figuring out how to spawn models in gazebo. Test 1 could not be included in github because that test included a tutorial urdf model from the github repository that was too big to include. The world consisted of the urdf file, a generated model from gazebo and a chair model from 3DWarehouse to show that they could all be included together. test2.launch will just launch the 3DWarehouse chair model found to show that an sdf model can be put into gazebo. The other launch files are self explanatory, generating the name that's on the launch file into gazebo.
