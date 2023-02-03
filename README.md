# Project 2 - Experimental Robotics (MSc Robotics Engineering, Unige)
This project is an extention of the work done in the assignment 1 of the experimental robotics course which can be found here:https://github.com/elh4m/exprimental_robotics_1 . In the previous project, a ROS package was developed for a toy simulation of Clauedo game in which a robot explore the environment to collect hints and deduced an hypothesis about who can be the killer. 

Building upon this architectural theme, the project contains some environment simulation and task-motion planning level upgrades. For environment simulation we have developed a scene in Gazebo simulator which contains a custom made robot model with an arm attached to its base. There are four hovering points (x,y,z) in the environment with the following 'x' and 'y' coordinates (-3,0), (3,0), (0,-3), (0,3) while the position of 'z' coordinate may be either 0.75 or 1.25 which is chosen randomly everytime. These four points depicts the locations of the four rooms where robot needs to place its arm's end-effector in order to collect the hints. 



Besides this there are small walls in the simulation environment which can been seen in the above picture. These walls restraint the robot to reach the points coordinates with its mobile base, therefore robot plan its arm motion to placed it over the point coordinates in order to collect the hints. Similar to the previous project the deduced hypotheses has to be consistent and correct which means it has to be based on three different types of hints and its ID needs to match the ID of the correct hypotheses. The hnts are of following types:

1. who: Robot can find a name of a person as a hint which can be a killer e.g: Prof. Plum.
2. what: Robot can find a name of a weapon as a hint which killer might have used e.g: Dagger.
3. where: Robot can find a name of random place where the crime might have been committed e.g: Hall.

Statement of a consistent hypothesis will be something like this: “Prof. Plum with the Dagger in the Hall”. Incase the deduced hypotheses is wrong then the robot will visit the rooms again for new hints until it forms a consistent hypotheses. Similar to the previous project, ARMOR package has been used to deduced the hypothesis which is developed by researchers at University of Genova. ARMOR is versatile management system that can handle single or multiple-ontology archetectures under ROS. Please find more details regarding ARMOR from here: https://github.com/EmaroLab/armor

In addition to this, we have also used ROSPlan to plan the behaviour of the our robot. ROSPlan is a framework which provides collection of tools for AI Planning in a ROS system. Its variety of nodes encapsulate planning, problem generation, and plan execution in itself. In order to used ROSPlan, problem statement of the project that we diccussed above has been translated into PDDL problem file which contains the required objects like 'robot' and 'waypoint', initial condition and goals which describe the final desire state of the environment and domain file which contains the actions that robot can opt for in order to achieve the goal like 'goto_waypoint'. At the start of the simulation we execute the planning loop services of the ROSPlan which includes problem generation, planning, parsing and dispatching. During the execution of the simulation its very likely that the robot will fail to complete the goals in the first attempt. Therefore, the archieture of the project is developed in the way that can sense the failure duiring execution and goes into replaning. After replanning, the robot starts the execution of the new plan and it keeps doing it until the goals are achieved.

## Project Installation:

This project requires ROS with ARMOR and ROSplan packages to be install in the system. Please make sure you have alrady install it before following the instructions. 

For installing ARMOR package please follow the instructions available in this respository: https://github.com/EmaroLab/armor and for ROSPlan package please follow the instructions in this repository: https://github.com/KCL-Planning/ROSPlan.

1. Code available in **Main** branch is a ROS package which should be place in ROS workspace {ros1_ws}/src after downloading.
2. To successfully deploy and build the package run the following command.
```
catkin_make
source devel/setup.bash
```
3. In order to use the python modules contained in armor_py_api package run the following command to add the path of the armor python modules to your PYTHONPATH environmental variable.
``` 
export PYTHONPATH=$PYTHONPATH:/root/ros_ws/src/armor/armor_py_api/scripts/armor_api/
```
4. Download the 'cluedo_ontology.owl' file provided in this repository and place it in your system '/root/Desktop/' directory. 

## Running the Project Simulation:

1. After successfully installing the python package and other dependecies open a command window and start ROS master by using the following command:
```
roscore&
```
2. After that start the ARMOR service by using the following command:
```
rosrun armor execute it.emarolab.armor.ARMORMainService
```
3. Open the new tab in command terminal and run the ROS launch file to start the simulation by using the following command: 
```
roslaunch erl2 assignment.launch
```
After running the command wait for the system to load all the files. Once all nodes are loaded open another terminal and execute 'assignment_services' launch file by running the following command:
```
roslaunch erl2 assignment_services.launch
```
Once all the service nodes are completely loaded then run the rosplan_start.sh bash file in another terminal to start the planning and dispatching process. Use the following command:
```
rosrun erl2 rosplan_start.sh
```

## Software Architecture of the Project:

The project architecture is based on the following main nodes. 

1. simulation.cpp

The simulation node implements the 'oracle' and visualization of the four hints position. The oracle is implemented in a way that it randomly generate hints of IDs 0 to 5 which may and may not generate inconsistent hypothesis. It also randomly generates a trustable ID which generates the consistent and correct hypothesis. This trustable ID can be requested from the 'oracle_solution' service that is also initialized in this node. The generated hints by this node are published in '/oracle_hint' topic.

2. my_action.cpp

This node implements an action client which work as a call function for the 'goto_waypoint' action in PDDL domain file. Hence, working as a low-level controller for the PDDL action abstraction. Upon call it request the robot in the Gazebo simulation to visit all waypoints (wp1,wp2,wp3,wp4). When the robot reached a waypoint location then this node call the service 'request_set_orientation' to set robot in appropiate orientation to move its arm suitably. Once this is done then robot request the 'request_move_arm' service to move its arm. After this it calls the '/request_hint_collector' service to collect the hints. If hint(s) collected successfully then the service returns 'true' and robot continue visiting other waypoints. 

3. hint_collector.cpp 

This node provides the '/request_hint_collector' service which collects and store the hints from the '/oracle_hint' topic as they are being published. Once three hints are collected, it checks their consistency and load them in the 'ARMOR' ontology knowlegde base. It start the ontology reasoner, and check if the deduced hypothesis is complete and correct. If the hypothesis is inconsistent, incomplete or incorrect then it return 'false' otherwise it return 'true'.

4. replan.cpp

This node provides the '/request_replan' service which upon request signal the 'replan_sub' node to start the replanning process by publishing a string message "replan" in the topic 'replan'.

5. replan_sub.cpp

This node initializes a subscriber for '/replan' topic. The callback funtion of this subscriber execute the 'rosplan_start.sh' bash file and starts the replanning. The bash file execute the following services which are the part of replanning loop. 

* /rosplan_problem_interface/problem_generation_server
* /rosplan_planner_interface/planning_server
* /rosplan_parsing_interface/parse_plan
* /rosplan_plan_dispatcher/dispatch_plan 

6. move_arm.cpp

This node plans and executes robot's arm motion using ROS Moveit library. It initializes a service through which a client can request to move robot arm's end-effector to a desired point in Gazebo environment.

7. hint_loader.py

This node waits for 'hint_loader_service' service's request from the 'hintcollector' node. Based on the request recieved, it loads the hint in the ARMOR reasonser, start the reasoner to deduced a hypotheses based on the previously loaded hints and request ARMOR reasoner for the list of 'COMPLETE' hypotheses. If the recently deduced hypothesis is 'COMPLETE' then it checks its 'CORRECTNESS' and return the appropiate response to the 'hintcollector' node. 

8. set_orientation.py

This node recieves the desired orientation goal coordinate as a '/request_set_orientation' service's request from the 'my_action' node. Based on the recieved goal coordinates it computes the required angular velocity of the robot to reach the desired orientation and then publishes it in 'cmd_vel' topic.

## Project Simulation Demo:



## Code Documentation:

The code documentation is done using Doxygen tool. Please find the doxygen documentation in the **main** branch .

## Contant Info: 
1. Author: Elham Mohammadi
2.Email:elhmohamadii@gmail.com
