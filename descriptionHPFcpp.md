## Description for Harmonic Potential Fields Algorithm
Author: Fabio Ugalde Pereira <br>
v0.0 - Initial draft (C++ and QtCreator) - 07/2023 <br>
Main reference -> https://www.inf.ufrgs.br/~prestes/publicacoes/RAS02.pdf <br>
Master's program for AI and Robotics @ INF-UFRGS, Brazil <br>

#### GENERAL ALGORITHM DESCRIPTION
This project was made for the Robotics course at the master's program taught by prof. Dr. Edson Prestes. The methodology that was implemented is an active exploration method based on harmonic functions to model the robot's environment and its behavior around to avoid obstacles and to always aim at unexplored regions of its simulated environment. <br>
The robot used is called 'Pioneer 3DX' and this code was used both in simulated environments using the 'MobileSim' software as well as in the real machine on the hallways of the Informatics Institute at UFRGS. <br>
This code was implemented in Linux Ubuntu 20.04 using QtCreator 4.9.2 with a ROS plugin that allows information to be read and published to ROS server. ROS 1 Noetic with nodes written on C++ were used for robot control and sensing. <br>
Regarding obstacle representation on metric maps, two methods were implemented: Bayes and Histogramic In-Motion Mapping (HIMM). Bayes relies on a statistical approach to calculate the probability of a given cell being an obstacle or not, while HIMM performs a simple linear function that increases the likelihood as sensor data bounces off more times. <br>
Aside from that, the code has the 'Active Exploration' and 'Navigation' modes. The former is always used first so that the robot can sense its environment and create a global map of obstacles. The latter is then used to navigate towards any reachable goal given by the user. <br>
This description does not include attributes specific to QtCreator and the graphical user interface (GUI), only HPF and ROS variables and functions. <br>
There are still many aspects to improve in this algorithm, such as refactoring it into OOP style, creating abstract classes and increasing information traceability, converting it to python and ROS 2, etc. These changes will be performed as the project moves forward.  

#### VARIABLES AND STRUCTS
---
This struct, a custom defined type in C++, was used as a global variable for keeping track of properties of each indivual cell of the map. 
```
struct gridCell {
  float prob_occ;     //bayes occupation probability 
  int himm_occ;       //himm occupation probability
  bool occupied;      //whether it is occupied or not
  bool obst_goal;     //if it is goal or obstacle
  bool path_cell;     //if a cell was in the path of robot
  float angle_pot;    //angle to plot harmonic field
  double harm_pot;    //harmonic potential of a given cell
  int hole_pot;       //property to close holes outside map
};
```
The grid used was 800 cells wide by 800 cells in height, all of which were of type 'gridCell'. 
```
gridCell world[800][800];   //world matrix of gridCells
```

The robot's properties are individual variables as shown below.
```
double x_pos       //x position
double y_pos       //y position
double yaw_rot     //rotation angle in radians
```

The sensors used were 8 sonars placed at the front face of the robot. The sonars provide a 'point-cloud' reading, that is, the data that is generated when an obstacle is sensed is sent back as a (x,y) pair in the sonar's frame of reference (FOR) that is later converted to the robot's FOR. This information was stored in a matrix of 8 lines (one for each sensor) and 2 columns (one for each coordinate of each sensor).
```
double sonar_readings[8][2]{};   //8 (x,y) readings 
```
Variables for setting methods of obstacle representation and harmonic fields estimation.
```
bool btn_BAYES; //BAYES obstacle representation
bool btn_HIMM;  //HIMM obstacles representation
bool btn_GOAL;  //Active Exp. Mode (false) - Navigation Mode (True)
bool btn_GS;    //Gauss-Seidel harmonic fields method
bool btn_SOR;   //SOR harmonic fields method
```

When in navigation mode, the 'goal' variable represents its coordinates.

```
std::pair<int,int> goal = std::make_pair (0,0); //goal for navigation
```
#### FUNCTIONS AND ALGORITHM FLOW

The first step in the program is to initialize the world cells with empty values.
```
for (int i=0;i<img_side;++i) {                          //iterate x axis
  for (int j=0;j<img_side;++j) {                        //iterate y axis
    world[i][j].occupied = 0;
    world[i][j].prob_occ = 0.5;
    world[i][j].himm_occ = 0;
    world[i][j].harm_pot = 0.0;
    world[i][j].obst_goal = 0;
    world[i][j].angle_pot = 0.0;
    world[i][j].path_cell = false;
    world[i][j].unused_var = false;
    world[i][j].hole_pot = 0;
    world[i][j].align = 0.0;
  }
}
```
Next, ROS configuration is performed. In this program, ROS uses a plugin called 'RosAria', which has functionalities and configurations specific to the Pioneer 3DX robot. This plugin facilitates the interface between the robot and its consuming endpoints, broadcasting or reading data in both ways.
During runtime, three ROS topics are created:<br>
1. /RosAria/pose 
2. /RosAria/sonar
3. /RosAria/cmd_vel <br>

The first two topics (/pose and /sonar) are sensor information coming from the robot. /pose provides a (x,y) pair for position and (x,y,z,w) quaternion for angle information based on the odometry calculated from the relative movement of the robot's wheels, and this information is used to estimate the position of the robot in the known environment, as well as its trajectory. /sonar returns an array of 8 (x,y) data pairs regarding the readings of each individual sonar sensor. If the read is beyond the sensor's capacity (more than 2.5 meters), the readings come as null. Otherwise, it returns valid (x,y) pair based on the sensor frame of reference.

```
//ROS config ------------------------------------------
nh_.reset(new ros::NodeHandle("~"));

// setup the timer for ros 
ros_timer = new QTimer(this);
connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
ros_timer->start(100);  // set the rate to 100ms

//setup subscriber for simulation
std::string listen_pose, listen_sonar, pub_topic;
nh_->param<std::string>("listen_topic",listen_pose,"/RosAria/pose");
pose_sub_  = nh_->subscribe<nav_msgs::Odometry>(listen_pose, 1, &RosMappingGUI::posePosition, this);
nh_->param<std::string>("listen_topic2", listen_sonar, "/RosAria/sonar");
sonar_sub_ = nh_->subscribe<sensor_msgs::PointCloud>(listen_sonar, 1, &RosMappingGUI::sonarObstacles, this);

//setup publisher for simulation
std::string pose_topic;
nh_->param<std::string>("pose_topic", pose_topic, "/RosAria/cmd_vel");
pub_topic_ = nh_->advertise<geometry_msgs::Twist>(pose_topic,1);
```
The readings coming from /pose are converted and attributed to the robot in the function 'posePosition'. The (x,y) position variables are attributed to the robot. The quaternion information is converted to roll, pitch and yaw angles. The roll and pitch angles are always zero, and yaw is then attributed to the robot's rotation.
```
void RosMappingGUI::posePosition(const nav_msgs::Odometry::ConstPtr &msg){

  //set global variables
  x_pos = msg->pose.pose.position.x;
  y_pos = msg->pose.pose.position.y;

  //convert quaternion to RPY
  tf::Quaternion q(
         msg->pose.pose.orientation.x,
         msg->pose.pose.orientation.y,
         msg->pose.pose.orientation.z,
         msg->pose.pose.orientation.w);
   tf::Matrix3x3 m(q);
   double roll, pitch, yaw;
   m.getRPY(roll, pitch, yaw);
   yaw_rot = yaw;
}
```
The sonar readings coming in from **/sonar** are stored in the **'sonar_readings'** matrix and the function **markOccupied()** is called to set occupation status the world cells based on the incoming readings.
```
void RosMappingGUI::sonarObstacles(const sensor_msgs::PointCloudConstPtr& son){

  //stores readings in x, y format
  for (unsigned long i=0; i<8; i++){
    sonar_readings[i][0] = double(son->points[i].x);
    sonar_readings[i][1] = double(son->points[i].y);
  }

  //function to mark occupied cells in the world
  markOccupied();
};
```
The function **markOccupied()** that is called performs many coordinate transformations and is summarized below:
1. The sonar data is read from the sonar_readings matrix.
2. If the user chooses  **Bayes** estimation method, the **bayes()** function is called and it marks as occupied cells that have a probability greater than 50% of being occupied. Cells with probabilities lower than 50% are left as free space.
3. If the user chooses **HIMM** estimation method, the **HIMM()** function is called and it marks as occupied cells that have a score greater than 7 (the HIMM scale goes from -1 (free) to 15 (100% probability of occupation)).

Both **Bayes()** and **HIMM()** functions perform a double coordinate conversion using the **convertCoord(sonar_readings)** function: <br>
**->Sonar (1 through 8) Frame of reference -> Robot Frame of reference -> World Frame of reference** 

The **Bayes()** function estimates the probabilities based on the bayes probability function as follows: (add reference)

The **HIMM()** function estimates the occupation score on a scale from -1 (fully unoccupied) to 15 (fully occupied). Every time a grid cell is read from one of the sonars, the cell HIMM score increments by one. If the cell is not read on the following iteration, it is decremented by one. When the score reaches 7 or greater, the cell is considered occupied and an obstacle is marked on that spot. This method is preferred over Bayes due to its simplicity and good results.