### Introduction and Objectives: 

In this final assignment we implemented and simulated three behaviors of MIRO Robot. Those three behaviors are:

• Normal 
• Sleep  
• Play  
• Find

These three behaviors were implemented inside a finite state machine which was built using ROS library called SMACH. This node is named as assignment3.py file. This assignment is built up on the assignment 1 and 2. The difference is that the assignment 1 was a high level assignment, we did not have the real simulation of the robot, we just had to simulate the behaviors on the terminal.

Whereas, assignment number 3 is a low level assignment, which mean  that we actually deal with the real robot in simulation and deal with velocity commands as well.   

Following are main tasks done in this assignment and will be discussed in detail in this readme file, later on. 

    • Spawned a two wheeled robot having a hokuyo laser sensor mounted on its front. This robot was already provided to us as part of some other exercises done during the course. 
    • Added additional links and joint for mounting a camera on the top of the robot. First a box was mounted on the base of the robot and on the front of this box, the camera was mounted. 
    • Made a custom message for this assignment. This message is called as Ball. This means that the type of this custom message is Ball. 
    • Built a User Interface so that the human can interact with the robot. This interface is built as a cpp file and is called Speak_node.cpp. 
    • Used gmapping package for building the map of the entire environment. 
    • Used move_base package for moving the robot. Move_base takes into account the world environment/map for planning trajectory. 
    • Used explore_lite package for publishing targets to mobe_base server. 
    • Loaded the important parameters in the rviz namely, RobotModel, LaserScan, Map,  Path and, Camera.  
    • Build the complete ROS architecture and simulate it on Gazebo.
    • Launched the complete assignment using a launch file. 

### Software Architecture:

Below is the image of my architecture. 

![alt text](https://github.com/laibazahid26/FinalAssignment_expRob/blob/master/architecture.png?raw=true)

The Speech processor node sets some parameters using ROS parameters server. And also gets the parameters from ROS parameter server. The Command Manager node, in which our finite state machine lies, subscribes to camera, laserScan and Odometry data.  These parameters are responsible for  controlling the flow of the entire code and hence result in switching of the robot from one state to another. 
The Odometry and Laser Scan data are used by SLAM gmapping algorithm to build the map of the unknown environment and in turn generates the map information. The map information together with the frontiers information (generated by the explore_lite node) will be used by Robot Controller to generate the velocity commands.   


### List of messages of the proposed Architecture:

    • All the flags used in this architecture namely, playflag, nearHuman, expLiteFlag should ideally have data type bool. But in our architecture their data type is integer. 
    •  Parameters namely, room and BallToBeFound have data type string.  
    • The Command Manager node odometry and laserScan info which have data types, Odometry and LaserScan, respectively, 
    • Path planner node provides frontiers. These are the locations which robot need to reach in order to explore the complete environment one by one. They have type MarkerArray. 
    • Robot Controller node will generate velocity commands to reach to each frontier and it will have type Twist. 


### State Diagram:

Below is the state diagram of this architecture. This finite state machine lies inside the command manager node of my architecture.

![alt text](https://github.com/laibazahid26/FinalAssignment_expRob/blob/master/finiteStateMachine.png?raw=true)

As can be seen, there are four states, i.e. Normal, Sleep, Play and, Find. The robot comes to life in being in the Normal state. Since we were required to ‘decide’ upon when the robot goes to sleep, we decided that whenever the robot finishes the normal state and the play command was not given by the user, then the robot goes to the Sleep state. We also decided that the robot will come out of the Sleep state when it finishes its sleeping time, i.e. after finishing sleep time, the robot goes back to the Normal state. The sleeping time is randomized between 5-10 seconds. 

On the other hand, if the robot is in the Normal state and the play command is given by the user from the user interface, then the robot enters in the Play state In the play state, we check if the location to go to is already explored or not. If it is not already explored then the robot switches to the Find state. In the Find state the robot tries to explore the environment and after a certain time if the ball is not detected the robot switches to the Play state. And if the ball is found, the robot switches to the play state immediately without waiting for the Find state timer to finish.    

After finishing the Play state, the robot goes back to the Normal state. 

### Packages and File List:

After git cloning the repository, you will have a folder named, FinalAssignment_expRob. 
Inside this folder we have four more folders. Let’s go through them briefly one by one. 

**1. slam_gmapping**

       This package implements one of the strategies for mapping the environment, implemented as a ROS node and provides SLAM capabilities. This package needs two information from our mobile robot namely, laser scan data and the odometry data. By default our robot gives this information as the topics (/scan and /odom) as required by gmapping. Gmapping also wants to know the base frame. Our base_frame name is different than the name understood by gmapping so in our launch file we remapped parameter base_frame to link_chassis.
       
**2. planning**

       move_base is part of the ROS navigation stack. We need to publish targets to move_base server and then taking into account the world environment, move_base plans a global path. Inside the param folder of planning, we have defined various parameters for move_base, and, global and local cost maps.   

**3. m-explore**

       move_base only works if we generate targets, in terms of coordinates, to it. We definitely do not want that. For autonomous navigation it is essential for the robot to move without having the need for the user to publish the coordinates. m-explore does this job for us. M-explore package is an action server and generate frontiers/targets for move base. By default, the global planner used for the move_base is navfn. 
       
**4. exp_assignment3**

       This folder was already provided to us. 
       
       Inside the world folder there lie a world file which describes the environment of our Gazebo world. The gazebo world was already given in the world folder of exp_assignment3.   
       
       Inside the urdf folder, lie the xacro and gazebo files which together describe the description and functionalities of the robot and, the human to be spawned in the Gazebo simulation. To be specific, xacro file describes elements of the robots i.e. the links, joints, their mass, inertia and their complete geometry. Whereas in gazebo file we define the plugins i.e. additional properties needed for simulation purposes in Gazebo.  
       We used the two wheeled robot which was provided to us previously during this course. We just defined additional links and joints for adding a box and a camera. We placed the box on the top of the robot base and on the front of this box we placed the camera.
       
       In the config folder we have saved the rviz configuration in a way that when the rviz will be loaded those parameters which are required for analyzing the complete architecture of the robot are loaded.  Those parameters are RobotModel, LaserScan, Map,  Path and, Camera.  
       
       Inside the scripts folder of exp_assignment3 we have implemented the entire architecture of the robot. The Speak_node.cpp file is the user interface for the user to interact with the robot for sending it in play state and then the location in which the robot should move after coming into the Play state. Whereas, in assignment3.py file the whole architecture of the robot is built as a finite_state machine.

       Inside the launch folder of exp_assignment3 we have two launch files. explore.launch file is repeatedly activated and shut down in our .py file. Whereas the other launch file, which is called simulation.launch, is responsible for launching gazebo, spawning robot and human inside the environment and is also responsible for launching all the nodes which lie inside the scripts folder. 
       
       The msg folder contains the custom data type we built for defining a ball. This custom message is called as Ball. This message has two parts. One part has type Bool, which is used as flag and will be set when a ball is detected and the other part has type Point,  which is used for storing the location of the detected ball. 
       
       The html folder was generated because of running doxygen. 

      
### Installation and Running Procedure:

    • Clone the repository
        ◦ git clone  https://github.com/laibazahid26/FinalAssignment_expRob/tree/master
    • Place the folder named FinalAssignment_expRob inside your workspace. 
    • Go in your workspace using terminal and then do a catkin_make.
    • In the previous step you were in your workspace. Now, using terminal, move to the scripts folder of exp_assignment3 package and give running permissions to it by writing the following command on the terminal:
                                  chmod +x assignment3.py
    • Now, we are in the position to run the assignment completely. We can do it by launching the .launch file by typing the following command on the terminal. 
		              roslaunch exp_assignment3 simulation.launch 

After running the above command you will see the robot waking up in the Normal state and then depending that the play command is given or not, the robot either switches to the Play state or continue to execute the Normal state.  

### Working Hypothesis and Environment: 

This package simulates the Normal, Sleep, Play and, Find states of a real robot simulated in the Gazebo environment. SMACH library was used to implement the finite state machine. These behaviors are implemented as classes in the assignment2.py file. On certain outcomes, the transition from one state/behavior to the other state happens.  
 
Below I’ll explain the working of each class, function, and, call-back, built inside our assignment3.py node. 

**• main()**

      The main function initializes the ros node, subscribes to the topic: robot's odometry topic ("/odom"), laser scan data (/scan), and the robot's camera image topic ("camera1/image_raw/compressed"), and creates a publisher for the robot's velocity commands on the topic ("/cmd_vel"), a publisher for  cancelling the goal provided to the move_base server ("/move_base/cancel"). 
      
      This function also intializes the "detectBallFlag", "expLiteFlag", "playflag", "nearhuman", "room", "ballToBeFound" parameters in the ROS Parameter Server. This function also defines the state machines and adds four states/behaviors to the container.   

**• RobotPos()**

      This function is the callback of the the topic /odom extracts the x and y location of the  robot in global variables.

**• class Normal()**

      In NORMAL state we have launched the explore_Lite package. While the timerdoesn't exceed a certain time, the robot keeps exploring the environment. While this timer doesn't end and the user gives the play command, ther robot enters into the PLAY state, else the robot goes into the SLEEP behavior. While the explore_lite package is running, the NORMAL also executes a function, detectBall(), to detect any ball in the environment. 

**• detectBall()**
      
      It uses Open CV to process the image received, and detects the contours of the different colored balls in the image using different masks for each color. If any contour is found, it passes the radius and center of the detected countour, and the detected ball object to the function track(). track() is implemented for going near the ball after shutting down the explore_lite package.
      
**• Play()**

      For PLAY state we implemented the Timer as a global variable because we wanted this state to stop after a certain time. This is done in this way becuase PLAY state will repeat more than once and we did not want Timer to be initialized again. 
      After giving play command in the GUI, the PLAY state of the robot is activated. First the robot moves near the human and waits for the GoTo location from the GUI. If the ball corresponding to the location has already been detected, the robot goes towards that ball or else the robot switches to the FIND state.

**• Track()**

      This function is used to associate the coordinate to every ball and hence every room, in a way that it records its own location as the ball location, once the robot has reached quite near to the ball. When the explore_lite package gets shut down, the robot approaches the ball while avoiding obstacles and when a certain threshold of the radius of the ball has been reached the robot stops and the explore_lite package is activated again.  

**• Sleep()**

      If no play command is given while the robot is in the NORMAL state, the robot switches to the SLEEP state. This state is implemented by publishing (0,0) coordinates to move_base server where the robot stays for 5-10 seconds and then switches to the NORMAL state.

**• movebase_client()**

      This function serves as a move_base client. It publishes goal to the move_base server and wait until the target has been reached.

**• Find()**

      In the PLAY state when the desired location is not yet explored, the robot switches to the FIND state. In this state, first the explore_lite package is activated for a certain time to explore and detect the required ball.
      While the time is running, detectBall() funtion is called. After returning from the detectBall() function we check if the required ball is found or not. If the ball is found, the state switches to PLAY state. If the ball is not found, it keep's on calling the detectBall() function till the timer expires. After the timer expires, the explore_lite package is shut and the robot switches to the PLAY state. 

### System’s limitations:

    1. This assignment relies on Explore_Lite for exploration purposes in all the states. Explore_Lite packages does get repeatedly shut down and activate where the need comes. Once the Explore_Lite package has explored the entire arena, more frontiers will not be generated and hence the robot will not move in Normal state and other functions where .
    2. Once the robot has switched to the Play state and some ball comes in its vision, the robot will not detect it. The detection happens only in Normal and Play states.  
    3. Error handling is not done in the system.
    4. After launching this node, the node can’t be interrupted (by Ctrl-C) and has to be closed by completely by shutting the terminal. 

### Possible technical Improvements:

    1. We thought about one technical improvement which can be used for removing limitation number 1, mentioned in System’s Limitation. We can check using topics of explore_lite that if the exploration is finished or not, if yes then we can rely on manually generating targets for move_base server. 
    2. There are so many ROS parameters used in this assignment. During the assignment it was difficult to manage them. We realized later on that this difficulties could have been mitigated if we have added more things in our custom built message named Ball().      

Authors and Contacts:

I collaborated with a fellow colleague for doing this assignment. Only the writing of readme file and making the github repository was done individually.  
Below I mention the names and contacts of both of us. 

Author 1: Laiba Zahid (S4853477)					email ID: laibazahid26@gmail.com
Author 2: Syed Muhammad Raza Rizvi (S4853521)		email ID: smrazarizvi96@gmail.com

 
