# ABatRe-Sim

> The Simulation Framework for automated battery recycling robot.

This depository contains source code for ABatRe-Sim, including robots, models, perceptions.

![](header.png)

## Installation
### Robot Environment Installation:
Ubuntu 16.04:
Create a ws and git clone this directory
Install dependency
```sh
rosdep install -r --from-path src --ignore-src
```
Make
```sh
catkin_make
```
### Tensorflow Environment Installation:
The perception framework is located in this repository.Please create a seperate workspace and follow the instruction there to install. https://github.com/hwan30/Object-Detetion-ABatRe-Sim-.git

## Usage example
Battery recycle robot user guide:

There are two launch files, one with finger gripper for picking bolts and cables, one with vacuum gripper for modules.run the second launch file when bolts and cable picking are finished.

Roslaunch unbolting_example main_unbolting.launch 

Roslaunch lifting_example ur10_remove.launch


Then launch the vision node to broadcast pixel u,v


Then start transform node u,v-> x,y,z

Roslaunch unbolting_example frame_transform.launch 


Then, pick scripts for bolts and cables, and pick scripts for module.

Rosrun unbolting_example pick_all.py

Rosrun lifting_example removing_module5.py


## Video Demo

## Citation


## Reference




## Contributing




