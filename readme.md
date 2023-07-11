# Inverse dynamics using RNEA.
The code is based on featherstone book. 

#### There are 4 sripts:

  a) link.cpp : To add new link in the robot and declaring links properties.
  
  b) robot.cpp: To add new robot model.
  
  c) id.cpp: To calculate tau using recursive newton wuler method
  
  d) main.cpp: main file to exute test cases.(we can change robot model using this code)

#### Robot model
For Robot model we are using json files in /Robot_model folder. There are two test cases, one is 2d planar robot of two link with revolute joint and other one is a ur5 robot with 6 links all revolute joint.

#### Header file
There are header files in /incude folder for the above scripts.

#### compilation and execution of code.
To compile the code just use make. To execute the same on terminal, run ./test1

 #### Psedu code for RNEA is as below: 

  ![image](https://github.com/Prakhar-Mishra51/Dynamics/assets/69066786/1a68ab13-d42e-41ae-bc67-53d5c3c875a1)
