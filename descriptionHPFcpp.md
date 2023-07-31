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
This description does not include attributes specific to QtCreator and the graphical user interface (GUI), only HPF variables and functions. <br>
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