function makeBoat(boatName)
% Creates a file, 'boatName'.mat, that contains sailboat parameter 
% information. This file can then be used to run the simulation.
%   Inputs:
%       boatName    string name of .mat file that this function will create
%
%   Cornell University
%   Author Name: Jesse Miller 
%   Author NetID: jam643

%% Hull Parameters
param.hull.radius= 0.11; % radius of hull at widest part [m]
param.hull.mass=3.3; % mass of hull [kg]
param.hull.length=0.9;  % length of hull [m]. The hull length must be around 
                        % 1 meter since the hull resistance used is based
                        % on a 1 m long hull.
%calculates volume of hull by estimating it as an ellipsoid
param.hull.vol_ellipsoid=(4/3)*pi*(param.hull.radius)^2*(param.hull.length/2);
                    
%% Sail Parameters
% (x,y,z) position of base of sail relative to the top center of the hull
param.sail.origin_relHullCOM=[0,0,0.05]';
% (x,y,z) Unit vector that points in the direction that the sail extends
param.sail.unit_vector=[0,0,1]';
% 1 or -1 indicating the sail points up or down unit vector
param.sail.direction=1;
param.sail.length=1;  %length of sail [m]
param.sail.width=0.24; %width of sail [m]
param.sail.mass=0.66;  %mass of sail [kg]
param.sail.density=param.sail.mass/(param.sail.width*param.sail.length);

%% Keel Parameters
%(x,y,z) position of base of keel relative to the top center of the hull
param.keel.origin_relHullCOM=[0,0,-param.hull.radius]';
% (x,y,z) Unit vector that points in the direction that the keel extends
param.keel.unit_vector=[0,0,-1]';
% 1 or -1 indicating the keel points up or down unit vector
param.keel.direction=-1;
param.keel.length=0.68; %length of keel [m]
param.keel.width=0.04; %width of keel [m]
param.keel.mass=0.3; %mass of keel [kg]
param.keel.angle_relBody=0; %angle of keel relative to boat [rad]
param.keel.pitch=0; % pitch angle of the keel
param.keel.density=param.keel.mass/(param.keel.width*param.keel.length);

%% Rudder Parameters
param.rudder.type=2;    %1: rudder is attached to boat
                        %2: rudder is attached to sail, AKA 'tail' design
%(x,y,z) position of base of rudder relative to the top center of the hull
param.rudder.origin_relHullCOM=[-0.5,0,-.1]';
% (x,y,z) Unit vector that points in the direction that the rudder extends
param.rudder.origin_relSail=[-0.4,0,0.-.17]';
% 1 or -1 indicating the rudder points up or down unit vector
param.rudder.direction=-1;
param.rudder.length=0.1; %length of rudder [m]
param.rudder.width=0.1; %width of rudder [m]
param.rudder.mass=0.04; %mass of rudder [kg]
param.rudder.density=param.rudder.mass/(param.rudder.width*param.rudder.length);

%% Ballast Parameters
param.ballast.mass=1.1; %mass of ballast [kg]

save(boatName)