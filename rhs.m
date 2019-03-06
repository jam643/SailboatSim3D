function stateDot = rhs(t,state,param)
% Calculate d/dt(state)=stateDot of the boat's state by calculating net
% forces and moments on boat and converting to rotational and translational
% accelerations using AMB and LMB
%   Inputs:
%       t           current time [sec]
%       state       12-by-1 array of the current state of the boat
%       param           structure containing various parameters
%
%   Outputs:
%       stateDot    12-by-1 array of d/dt(state) array          
%
%   Cornell University
%   Author Name: Jesse Miller 
%   Author NetID: jam643

if param.realTime == 0
    param=boatGeometry(param,state);
end
if param.threeD == 0
    state(4)=0; state(10)=0;
    state(5)=0; state(11)=0;
end
%% unpack variables
boat_angles=state(4:6); % [roll,pitch,yaw]'
phi=boat_angles(1);
theta=boat_angles(2);
p=state(10); q=state(11); r=state(12);

% moments of inertia
Ixx=param.boat.Ixx; Iyy=param.boat.Iyy; Izz=param.boat.Izz;

%% calculate forces and moments on airfoils (sail, keel, and rudder)
% net force and moment on sail
[netForce_sail_relFixed, netMoment_sail_relBody] = airfoil_forces2(state,param.sail,param);
% net force and moment on keel
[netForce_keel_relFixed, netMoment_keel_relBody] = airfoil_forces2(state,param.keel,param);
% net force and moment on rudder
[netForce_rudder_relFixed, netMoment_rudder_relBody] = airfoil_forces2(state,param.rudder,param);

%% calculate hull forces and moments
% forces and moments due to hull resistance
[resistanceForce_hull_relFixed,resistanceMoment_hull_relBody]=...
hullForce_resistance(state,param);
% forces and moments due to hull resistance
[buoyancyForce_hull_relFixed,buoyancyMoment_hull_relBody]=...
    hullForce_buoyancy(state,param);
% forces and moments due to hull damping forces
[dampingForce_hull_relFixed,dampingMoment_hull_relBody]=...
    hullForce_damping(state,param);

%% Calculate weight
Force_weight_relFixed=[0,0,-param.g*param.boat.mass]';

%% Total moments and forces
netForce_relFixed=(netForce_sail_relFixed+netForce_keel_relFixed+...
    netForce_rudder_relFixed+resistanceForce_hull_relFixed+...
    buoyancyForce_hull_relFixed+dampingForce_hull_relFixed+...
    Force_weight_relFixed);
netMoment_relBody=netMoment_sail_relBody+netMoment_keel_relBody+...
    netMoment_rudder_relBody+resistanceMoment_hull_relBody+...
    buoyancyMoment_hull_relBody+dampingMoment_hull_relBody;

%% Linear Momentum Balance (a=F/m)
vdot=netForce_relFixed/param.boat.mass;

%% Angular Momentum Balance
% roll angular acceleration in body frame
pdot=(Iyy-Izz)*q*r/Ixx+netMoment_relBody(1)/Ixx;
% pitch angular acceleration in body frame
qdot=(Izz-Ixx)*p*r/Iyy+netMoment_relBody(2)/Iyy; 
% yaw angular acceleration in body frame
rdot=(Ixx-Iyy)*p*q/Izz+netMoment_relBody(3)/Izz;

% roll angular acceleration in inertial frame
phidot=p+(q*sin(phi)+r*cos(phi))*tan(theta);
% pitch angular acceleration in inertial frame
thetadot=q*cos(phi)-r*sin(phi);
% yaw angular acceleration in inertial frame
psidot=(q*sin(phi)+r*cos(phi))/cos(theta);

%% update loading bar to represent progress
if ~param.realTime && floor(20*rand) == 0
    waitbar(t/param.time,param.waitBar);
end

% update d/dt(state) array
stateDot=[state(7:9)',phidot,thetadot,psidot,...
    vdot',pdot,qdot,rdot]';
