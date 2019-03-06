function param=boatGeometry(param,state)
% Calculates the boat's total COM, pose of sail keel and rudder relative to
% the boat's COM, the moment of inertia of the boat, and rotational
% matrices.
%   Inputs:
%       param       structure containing relavent parameters
%       state   12-by-1 array containing the state of the boat
%
%   Outputs:
%       param       updated structure containing relavent parameters
%
%   Cornell University
%   Author Name: Jesse Miller 
%   Author NetID: jam643

%% unpack sail, keel, rudder structures
sail=param.sail; keel=param.keel; rudder=param.rudder; ballast=param.ballast; hull=param.hull;
i_hat=[1,0,0]'; %unit vector along x-axis
k_hat=[0,0,1]'; %unit vector along z-axis

% param.keel.origin_relHullCOM=[0.5*sin(param.keel.pitch)*param.keel.length,0,-param.hull.radius]';

%% calculate surface areas of airfoils
sail.SA=sail.width*sail.length; %surface area sail [m^2]
keel.SA=keel.width*keel.length; %surface area keel [m^2]
rudder.SA=rudder.width*rudder.length; %surface area rudder [m^2]

%% calculates mass of airfoils;
sail.mass=sail.SA*sail.density;
keel.mass=keel.SA*keel.density;
rudder.mass=rudder.SA*rudder.density;

%% center of mass calculations relative to top center of hull
% calculate COM of the sail, keel, and rudder relative to the Hull top
% center point
sail_COM=sail.origin_relHullCOM+0.5*sail.length*sail.direction*k_hat;
keel_COM=keel.origin_relHullCOM+0.5*keel.length*keel.direction*k_hat;
if rudder.type == 1 % rudder fixed to boat
    rudder_COM=rudder.origin_relHullCOM+0.5*rudder.length*rudder.direction*k_hat;
elseif rudder.type == 2 %rudder fixed to sail
    % homog. matrix of sail relative to hull
    [sail.H_relHullCOM,~,~,~]=euler2Hom([0,0,sail.angle_relBody],sail.origin_relHullCOM);
    % homog. matrix of tail relative to sail
    [rudder.H_relSail,~,~,~]=...
        euler2Hom([0,0,rudder.angle_relSail],rudder.origin_relSail);
    % homog. matrix of tail relative to hull
    H_rudder_relHullCOM=sail.H_relHullCOM*rudder.H_relSail;
    % tail center of mass relative bottom of tail
    rudderCOM_relRudder=[0.5*rudder.length*rudder.direction*k_hat;1];
    % rudder COM relative to the hull top center point
    rudder_COM=H_rudder_relHullCOM*rudderCOM_relRudder;
    rudder_COM(4)=[];
end
% ballast and hull COM relative to the top center of the keel
ballast_COM=keel.origin_relHullCOM+keel.length*keel.direction*(k_hat*...
    cos(keel.pitch)+i_hat*sin(keel.pitch));
hull_COM=0;%-0.5*hull.radius*k_hat;

%total boat mass
boat.mass=sail.mass+keel.mass+ballast.mass+rudder.mass+hull.mass;
% boat COM relative to the hull top center
boat.COM=(sail_COM*sail.mass+keel_COM*keel.mass+rudder_COM*rudder.mass...
    +ballast_COM*ballast.mass+hull_COM*hull.mass)/boat.mass;

% origin (bottom) of sail, keel, rudder, hull relative to boat COM
sail.origin=sail.origin_relHullCOM-boat.COM;
keel.origin=keel.origin_relHullCOM-boat.COM;
rudder.origin=rudder.origin_relHullCOM-boat.COM;
hull.origin=-boat.COM;

%% Moments of inertia relative to center of mass of boat
% Moments of inertia about x axis
sail.Ixx=momentOfInertia(sail.mass,sail.length,0,sail_COM(3)-boat.COM(3),0);
keel.Ixx=momentOfInertia(keel.mass,keel.length,0,keel_COM(3)-boat.COM(3),0);
rudder.Ixx=momentOfInertia(rudder.mass,rudder.length,0,rudder_COM(3)-boat.COM(3),0);
ballast.Ixx=ballast.mass*(ballast_COM(3)-boat.COM(3))^2;
hull.Ixx=(2/5)*hull.mass*hull.radius^2+hull.mass*(boat.COM(3))^2;
% Total moment of inertia about x axis
boat.Ixx=sail.Ixx+keel.Ixx+rudder.Ixx+ballast.Ixx+hull.Ixx;

% Moments of inertia about z axis
sail.Izz=momentOfInertia(sail.mass,sail.width,0,sail_COM(1)-boat.COM(1),0);
keel.Izz=momentOfInertia(keel.mass,keel.width,0,keel_COM(1)-boat.COM(1),0);
rudder.Izz=momentOfInertia(rudder.mass,rudder.width,0,rudder_COM(1)-boat.COM(1),0);
ballast.Izz=ballast.mass*(ballast_COM(1)-boat.COM(1))^2;
hull.Izz=(1/5)*hull.mass*(hull.radius^2+(hull.length/2)^2)+...
    hull.mass*(boat.COM(1))^2;
% Total moment of inertia about z axis
boat.Izz=sail.Izz+keel.Izz+rudder.Izz+ballast.Izz+hull.Izz;

% Moments of inertia about y axis
sail.Iyy=momentOfInertia(sail.mass,sail.width,sail.length,...
    sail_COM(3)-boat.COM(3),sail_COM(1)-boat.COM(1));
keel.Iyy=momentOfInertia(keel.mass,keel.width,keel.length,...
    keel_COM(3)-boat.COM(3),keel_COM(1)-boat.COM(1));
rudder.Iyy=momentOfInertia(rudder.mass,rudder.width,rudder.length,...
    rudder_COM(3)-boat.COM(3),rudder_COM(1)-boat.COM(1));
ballast.Iyy=ballast.mass*((ballast_COM(3)-boat.COM(3))^2+...
    (ballast_COM(1)-boat.COM(1))^2);
hull.Iyy=(1/5)*hull.mass*(hull.radius^2+(hull.length/2)^2)+...
    hull.mass*((boat.COM(1))^2+(boat.COM(3))^2);
% Total moment of inertia about z axis
boat.Iyy=sail.Iyy+keel.Iyy+rudder.Iyy+ballast.Iyy+hull.Iyy;

%% Moment of area of hull for pitching moment
hull.Iyy_Area=(pi/4)*(hull.length/2)^3*hull.radius;

%% Update homog. transform matrices and rotation matrices for different parts of the boat
% homog. transform, rotation matrix, and inverses of both, representing
% sail, keel, and keel frames relative to fixed frame
[sail.H,sail.Ht,sail.R,sail.Rt]=euler2Hom([0,0,sail.angle_relBody],sail.origin);
[keel.H,keel.Ht,keel.R,keel.Rt]=euler2Hom([0,keel.pitch,keel.angle_relBody],keel.origin);
[hull.H,hull.Ht,hull.R,hull.Rt]=euler2Hom([0,0,0],hull.origin);
% different Rot. and Hom. matrix calculations for rudder vs tail design
if rudder.type==1 %rudder type
    [rudder.H,rudder.Ht,rudder.R,rudder.Rt]=euler2Hom([0,0,rudder.angle_relBody],rudder.origin);
elseif rudder.type==2 %tail type
    [rudder.H_relSail,rudder.Ht_relSail,rudder.R_relSail,rudder.Rt_relSail]=...
        euler2Hom([0,0,rudder.angle_relSail],rudder.origin_relSail);
    rudder.H=sail.H*rudder.H_relSail;
    rudder.R=sail.R*rudder.R_relSail;
    rudder.Rt=rudder.R.';
end
% homog. transform, rotation matrix, and inverses of both, representing
% boat frame relative to fixed frame
[boat.H,boat.Ht,boat.R,boat.Rt]=euler2Hom(state(4:6),state(1:3));

%% Breaks up sail, keel, and rudder into Blade Elements
sail.points_relAirfoil=bladeElement2(sail,param);
keel.points_relAirfoil=bladeElement2(keel,param);
rudder.points_relAirfoil=bladeElement2(rudder,param);

%% Repack variables
param.sail=sail; param.keel=keel; param.rudder=rudder; param.boat=boat;
param.hull=hull;