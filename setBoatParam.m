function [param,state0]=setBoatParam(boatName)
% Initializes various boat parameters and initial state of the boat. Make
% changes to the parameters in this file to test various conditions.
%   Inputs:
%       boatName    string name of .mat file created with makeBoat.m
%                   function containing boat parameter information
%
%   Outputs:
%       param       structure containing relavent parameters
%       state0      12-by-1 array containing the initial pose of the boat
%
%   Cornell University
%   Author Name: Jesse Miller 
%   Author NetID: jam643

%% Load boat parameters
load(boatName);

%% Real time boat control vs. precalculating trajectory
param.realTime=1;   %0: precalculate boat trajectory
                    %1: real time control of boat using keyboard
%% initial conditions
%initial pose and (d/dt)pose
x0=0; y0=0; z0=0; % initial x,y,z coordinates of boat COM
phi0=0; theta0=0; psi0=40*pi/180; % initial roll, pitch, yaw of boat
xdot0=0; ydot0=0; zdot0=0; % initial x,y,z velocities of boat COM
p0=0; q0=0; r0=0; %initial roll, pitch, yaw angular velocities in boat frame
%contains the initial state/pose of the boat
state0=[x0,y0,z0,phi0,theta0,psi0,xdot0,ydot0,zdot0,p0,q0,r0]';

%% Initial sail and rudder angles
param.sail.angle_relBody= 50*pi/180; %angle of sail relative to boat [rad]
param.rudder.angle_relBody=10*pi/180; %(for rudder.type=1) angle of rudder relative to boat [rad]
param.rudder.angle_relSail=15*pi/180; %(for rudder.type=2) angle of rudder relative to sail [rad]

%% simulation fps and run time
param.time=5; %total run time [s]
param.fps=100; %frames per second
param.tspan=linspace(0,param.time,param.fps*param.time); %timespan of simulation [s]

%% Parameters Affecting Simulation Accuracy
param.threeD=1;     %0: 2D simulation, 1: 3D simulation
param.nElements=2;  %number of blade elements to use for airfoil calculations
param.numInt=7;     %number of times to perform Euler integration per timestep
                    %(increase if solution explodes, decrease if simulation is
                    %too slow). Only used when param.realTime=1
param.accuracy=1;   %param.accuracy defines the accuracy with which lift and drag
                    %coeff. are interpolated
                    %1: most accurate but slower (using pchip interpolation)
                    %2: less accurate but fastest (approx. Clift Cdrag as sinusoidal)

%% Environmental Parameters
param.v_air_relFixed=[3.5,0,0]'; %(x,y,z) components of the true wind [m/s]
param.v_water_relFixed=[0,0,0]'; %(x,y,z) components of the water velocity [m/s]
param.density_air=1.2; %density of air [kg/m^3]
param.density_water=1000; %density of water [kg/m^3]
param.g=9.8; %acceleration due to  gravity [m/s^2]

%% Sail and Rudder Control (requires param.realTime=1). This will override
%% the manual control option.
param.servoOn=0; % 1: send predefined commands to the sail and tail servos
param.sail.rotRate=0.5;% servo rotation rate[rad/s]
param.sail.angleVect=[45,-45]*pi/180; % vector containing sail angle waypoints [rad]
param.sail.timeVect=10; % time at which actuation starts [sec]
% calculates time vector corresponding to changes in sail angle
for k=2:length(param.sail.angleVect)
    param.sail.timeVect(k)=param.sail.timeVect(k-1)+...
        abs(param.sail.angleVect(k)-param.sail.angleVect(k-1))/param.sail.rotRate;
end

param.rudder.rotRate=0.5/3;% servo rotation rate[rad/s]
param.rudder.angleVect=[15,-15]*pi/180; % vector containing rudder angle waypoints [rad]
param.rudder.timeVect=10; % time at which actuation starts [sec]
% calculates time vector corresponding to changes in rudder angle
for k=2:length(param.rudder.angleVect)
    param.rudder.timeVect(k)=param.rudder.timeVect(k-1)+...
        abs(param.rudder.angleVect(k)-param.rudder.angleVect(k-1))/param.rudder.rotRate;
end

%% DO NOT MAKE CHANGES TO THE CODE BELOW
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% calculate boat properties based on chosen parameters 
param=boatGeometry(param,state0);

%% Tabulated Data for NACA 0015 airfoil to be interpolated later on
%angles [deg] corresponding with the lift and drag coefficient data points
angle = [0,10,15,17,23,33,45,55,70,80,90,100,110,120,130,...
    140,150,160,170,180,190,200,210,220,230,240,250,260,...
    270,280,290,305,315,327,337,343,345,350,360]';
%lift coefficient data points corresponding with the angle array
lift = [0,0.863,1.031,0.58,.575,.83,.962,.8579,.56,.327,...
    .074,-.184,-.427,-.63,-.813,-.898,-.704,-.58,-.813,0,...
    .813,.58,.704,.898,.813,.63,.427,.184,-.074,-.327,...
    -.56,-.8579,-.962,-.83,-.575,-.58,-1.031,-.863,0]';
%drag coefficient data points corresponding with the angle array
drag = [0,.021,.058,.216,.338,.697,1.083,1.421,1.659,1.801,...
    1.838,1.758,1.636,1.504,1.26,.943,.604,.323,.133,0,...
    .133,.323,.604,.943,1.26,1.504,1.636,1.758,1.838,1.801,...
    1.659,1.421,1.083,.697,.338,.216,.058,.021,0]';

param.paraDrag=.15; %parasitic drag used to limit max(Cl/Cd)=5
drag=drag+param.paraDrag; %adjusted drag

param.maxLDratio=max(lift./drag); %max lift to drag ratio

param.C0=0.9;  % nominal lift coefficient for sinusoidal lift/drag approx.

% Fit a pchip piecewise-polynomial curve fit to the data
% This is good because it preserves local maximum and minimum
% Another option would be to replace pchip() with spline(), which would
% produce a more smooth curve, but would add new peaks to the data.
param.lift = pchip(angle,lift);
param.drag = pchip(angle,drag);

%calculate maximum lift to drag ratio
alpha=linspace(0,180,181);
lift_smooth=zeros(1,length(alpha));
drag_smooth=zeros(1,length(alpha));
for i=1:length(alpha)
    lift_smooth(i)=ppval(param.lift,alpha(i));
    drag_smooth(i)=ppval(param.drag,alpha(i));
end
param.maxLDratio=max(lift_smooth./drag_smooth); %max lift to drag ratio

