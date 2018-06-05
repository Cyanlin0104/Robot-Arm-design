%{
Program: RobotArmKinematicsSimulation.m

Describe: This program demostrate a robot arm Simulation.
	  For testing a design of robot arm.
          

	Args:
	    Inputs:
			dof: An int (3 or 6)
			DH_table: A (dof, 4) matrix describing your desired arm. 	
			
			Link_length: A vector indicating yout length of each link.
				ex: [2, 4, 4] for a 3-axis robot arm.
					[2, 4, 3, 2] for a 6-axis 				
%}

% clean up

clc;
close all;

prompt1 = 'Please choose your drgree of freedom(3 or 6)'
prompt2 = 'Please choose your '
dof = input(prompt);
while(~(dof == 3 || dof == 6))
	dof = input(prompt);

forwardOrInverse

