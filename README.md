# Robot Crowd Navigation through Predictive Pedestrian Modeling
The control stack for this robot has been built based on this [repo](https://github.com/kylevedder/ServiceRobotControlStack). The obstacle detector package is provided by this [repo](https://github.com/tysik/obstacle_detector) and the pedestrian simulator is provided by this [repo](https://github.com/srl-freiburg/pedsim_ros).

We present a local path planning algorithm, deployable on a service robot, that generates probabilistic models about the future as outlined in our paper in `paper/`. The implementation can be found in `control_stack/include/cs/path_finding/rrt`. This algorithm can be deployed on a service robot in different simulation environments and demonstration videos can be found in `demos/`.

## Requirements

- ROS Melodic

## Installation
- Clone this repo to your \<catkin workspace folder\>/src

## Usage
- Ensure that you are in the root directory of your catkin workspace

`$ source ./devel/setup.bash`

- To run a single obstacle environment:

`$ roslaunch pedsim_simulator 700_1obst.launch`

- To run a hallway with pedestrian traffic against the service robot:

`$ roslaunch pedsim_simulator 700_against_1way.launch`

- To run a hallway with two way pedestrian traffic:

`$ roslaunch pedsim_simulator 700_2way_hallway.launch`

- To run a four way interection:

`$ roslaunch pedsim_simulator 700_4way.launch`

- Note that for each scenerio you want to run, you will need to update the map path in corresponding nav_config_700_*.lua file in `control_stack/config` to match your absolute path

- Analytics for each run will be automatically printed on the terminal
