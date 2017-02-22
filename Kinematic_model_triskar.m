%%%%%%%%% KINEMATIC MODEL OF AN OMNIDIRECTIONAL ROBOT %%%%%%%%%
clear all;
close all;
%% INPUT PARAMETERS: VELOCITY ON THE BODY REFERENCE FRAME

xLdot=-0.5;        % [m/s] 
yLdot=0;        % [m/s]
thetaLdot=0;    % [deg/s]

%%%%% ANGULAR PARAMETERS %%%%%

% wheels displacement
alpha1 = 0;   % degree
alpha2 = 120; % degree
alpha3 = 240; % degree
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%% STRUCTURAL PARAMETERS %%%%
L = 7e-2;    % [meters] distance from wheel to center of gravity
r = 2e-2;      % [meters] wheels radius
theta = 0;   % [degree] angle between xGdot and motor 1 shaft
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%   KINEMATICS   %%%%%%%%%%%

% Velocity vector in the local reference frame 
VL = [xLdot; yLdot; thetaLdot];  

% Rotation matrix from Global to Local reference frame
R  = [cosd(theta),-sind(theta),0;  
      sind(theta),cosd(theta),0; 
      0,0,1;]; 

% Geometrical Jacobian   
%B = [sind(theta+alpha1),sind(theta+alpha2),sind(theta+alpha3);       
%     cosd(theta+alpha1),cosd(theta+alpha2),cosd(theta+alpha3);
%     L,L,L;];
 B = [0, sqrt(3)/2, -sqrt(3)/2;
      1, -0.5, -0.5;
      L, L, L;];

% Velocity vector in the global reference frame
VG = R*VL;         

% Angular velocity of the three wheels [deg/sec]
phidot = (1/r)*B.'*VL;   

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%% DYNAMIC MODEL TRISKAR %%%%%%%%%%%%%%


%%%%%%%%%%%%  DYNAMIC PARAMETERS OF THE ROBOT  %%%%%%%%%%%%%%%%

Jm = 2.7e-7;  % Inertia of the motor [Kg/m^2]
JL = 8e-5;    % Inertia of the load  [Kg/m^2]
 J = 4.6e-3;  % Rotational inertia of the robot [Kg/m^2]
 n = 14;      % Reduction Ratio ( or 17.5)
cm = 5e-8;    % Damping coefficient motor
%cl = 1e-6;   % Damping coefficient load
cl = 0;
 m = 2.36;    % Mass of the robot [Kg]
 M = [m,0,0;0,m,0;0,0,J]; % Mass matrix of the robot
 
%%%%%%%%%%%% MOTORS PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%

Ke  = 0.0145;  % Back EMF constant of the motor
Km  = 0.0144;  % Torque constant of the motor
Klr = 1/8.71;  % Motor constant where 8.71 Ohm is the motor resistance (inductance neglected)
