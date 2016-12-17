# Robotics Project Readme

Team 10/Learn to Ducking Park
Greg Echeverria, Charles Parslow
This explains the matlab codes used to run simulations explained in the report

drive_simulation.m closed loop simulator 

open_loop_simulator.m open loop simulator 

wheel_calibration_simulation.m wheel calibration simulator

makeBlock.m Creates blocks to create the road 

addToBoard.m Creates the board (the road, not sure why i chose the word board) one block at a time, building a matrix called occfn which tells the code the actual distance and angle for any given position on the road. This was useful for validating control models and not having to use noisy image processing to approximate errors at first. 

getDistanceAndAngle.m Uses occfn to get distance and angle errors for any position on the board

getCameraVision.m Generates projective2D transform and field of view for camera 

parameters getPOV.m Creates POV image as seen from simulated camera

transformBoard.m Rotates board and centers camera position

parsePOV.m Parses POV image to obtain guesses for angle and position error
