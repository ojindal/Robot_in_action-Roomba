# Roomba

<p align="justify">
The aim is to design a Roomba-like system capable of mapping its environment and providing a certain level of area coverage for the supposed cleaning action, similar to a Roomba robot
</p>

## Project Report
[Orish Jindal, Sanchit Gupta, 'Roomba: design motivation and architecture, modules’ functionalities, and results', CSE 276A, Course Project, UCSD](https://github.com/ojindal/Robot_in_action-Roomba/blob/main/CSE276A%20HW5%20Report.pdf)

## Path traced by Algorithm: Two Obstacles 
<p align="center">
  <img width="400" alt="image" src = "https://user-images.githubusercontent.com/89351094/209572628-7fc27b92-47c3-4042-ba8b-b0e744d0097a.png"/>
 </p>

 
 ## Robot in action: Cleaning action (Without obstacles)
<p align="center">
  <img src = "https://user-images.githubusercontent.com/89351094/209573403-fcacd706-24b1-4732-8807-c9af0dad67ef.gif"/>
</p>


## Details to run the code

The architecture of the code includes the following four nodes:

* <b> Node 1: </b> Camera node initialized using rb5_camera_main_ocv.launch executable
* <b> Node 2: </b> April tag detection node initialized using apriltag_detection_array.py executable
* <b> Node 3: </b> MPI control node initialized using hw2_mpi_control_node.py executable
* <b> Node 4: </b> Path planning node initialized using fccp_hw5.py executable
