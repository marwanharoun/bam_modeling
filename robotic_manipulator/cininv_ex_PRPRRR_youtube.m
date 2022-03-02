% https://www.youtube.com/watch?v=ZM9GOENJcuo&t=1353s
% PRPRRR
% This is a manipulator with spherical wrist


clear;
clc;

syms a1 a2 a3 a4 a5 a6 a7
syms d1 TETA2 d3 TETA4 TETA5 TETA6

% We start by drawing the kinematic diagram of the first 3 joints, and then 
% doing the inverse kinematics to find the position of W 

% Then we do the forward kinematics from joints 0 to 3, and then
% extract from that the rotation from 0 to 3:
T1 = T_Trans([0 0 1], d1)
T2 = T_Rot([0 0 a1], [0 0 1], TETA2)
T3 = T_Trans([1 0 0], d3)
T03_0 = [[0; 1; 0; 0] [0; 0; 1; 0] [1; 0; 0; 0] [a3+a4; 0; a1+a2; 1]]

T03 = T1*T2*T3*T03_0

% We can get the rotation part of the first 3 joints:
R03 = [T03(1,1) T03(1,2) T03(1,3);T03(2,1) T03(2,2) T03(2,3);T03(3,1) T03(3,2) T03(3,3)]


% R06 is given and is actually the desired rotation matrix of the end effector:
R06 = [[-1; 0; 0] [0; -1; 0] [0; 0; 1]]

% Since R06 = R03*R36, we find R36 with the inverse of R03:
R36 = R03\R06

% Now we do the forward kinematics from joints 3 to 6, so basically we find
% an analytical form of R36. We'll call it R36_cindir:
% ATT!!! When doing that part of the forward kinematics, remember to take
% the frame 3 as the base frame, and not the frame zero! So for example in
% the transformation matrix T4 below, [0 0 1] are the coordinates of the
% rotation axis of joint 4 expressed in frame {3} and not {0}.


T4 = T_Rot([a3+a4 0 a1+a2],[0 0 1], TETA4)
T5 = T_Rot([a3+a4+a5 0 a1+a2],[-1 0 0],TETA5)
T6 = T_Rot([a3+a4+a5+a6 0 a1+a2],[0 0 1],TETA6)
T36_0 = [[0; 1; 0; 0] [-1; 0; 0; 0] [0; 0; 1; 0] [a3+a4+a5+a6+a7; 0; a1+a2; 1]]

T36 = simplify(T4*T5*T6*T36_0, 'steps',10)

R36_cindir = [T36(1,1) T36(1,2) T36(1,3);T36(2,1) T36(2,2) T36(2,3);T36(3,1) T36(3,2) T36(3,3)]


% Now we solve for R36 = R36_cindir. But we must have solved for the
% position of W already, using geometric intuition.