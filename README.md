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
Delete the Devel and Build folder

Then Make
```sh
catkin_make
```
### Tensorflow Environment Installation:
The perception framework is located in this repository.Please create a seperate workspace and follow the instruction there to install. https://github.com/hwan30/Object-Detetion-ABatRe-Sim-.git

## Usage example
Battery recycling robot user guide:

There are two launch files, one with finger gripper for picking bolts and cables, one with vacuum gripper for modules.Only run the second launch file when bolts and cable picking are finished.

To run the finger gripper environment to remove bolts and cables:

Roslaunch unbolting_example main_unbolting.launch 

To run the vaccum gripper environment to remove modules:

Roslaunch lifting_example ur10_remove.launch


Then launch the vision node to broadcast pixel u,v. Follow the rosrun instructions on the vision node github repo.


Then start transform node u,v-> x,y,z

Roslaunch unbolting_example frame_transform.launch 


Then, run pick scripts for bolts and cables, and run pick scripts for module.

Rosrun unbolting_example pick_all.py

Rosrun lifting_example removing_module5.py


## Video Demo

## Citation


## Reference




## Contributing




