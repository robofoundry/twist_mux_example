### Main steps to complete all installation

### pre-requisites
1. ROS2 foxy is installed
2. You have twist_mux and xterm packages installed [don't worry if you don't you can do this by following next steps]
3. clone the repo and follow steps below

### next steps - get the code and build the workspace
1. Get the code from github repo for dynabot 
``` 
git clone https://github.com/robofoundry/twist_mux_example
```
2. while in root workspace folder run the following command to execute shell script that will get code from all external github repos installed in workspace and run rosdep to install any depedencies that are missing [on both host and robot computer]
``` 
rosdep install --from-path src --ignore-src -y 
```

3. while in root workspace folder run the following commands to build and launch the nodes 
``` 
colcon build
source install/setup.bash
ros2 launch twist_mux_test twist_mux_test.launch.py
```

