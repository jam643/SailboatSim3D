function [buoyancyForce_hull_relFixed,buoyancyMoment_hull_relBody]=hullForce_buoyancy(state,p)
% Calculates forces and moments in the fixed and body frames respectively
% related to buoyance forces
%   Inputs:
%       state                           12-by-1 array of the current state of the boat
%       p                               structure containing various parameters
%   Outputs:
%       buoyancyForce_hull_relFixed     3-by-1 net hull buoyancy force
%                                       relative to fixed frame
%       buoyancyMoment_hull_relBody     3-by-1 net hull buoyancy moment 
%                                       relative to boat frame
%
%   Cornell University
%   Author Name: Jesse Miller
%   Author NetID: jam643

%% unpack variables
z=state(3);
phi=state(4);
theta=state(5); 
R=p.boat.R; % boat rotation matrix
Rt=p.boat.Rt; % transpose of boat rotation matrix

%% pitching moment due to small angles
if z-p.hull.radius < 0 % hull is in the water
    % pitching moment approximation for small angles
    pitchMoment_hull_relPitch=[0;-p.hull.Iyy_Area*p.density_water*p.g*theta;0];
else %boat is above water
    pitchMoment_hull_relPitch=[0;0;0];
end
% convert pitching moment to boat frame
R_banked=euler2Rot(phi,0,0);
pitchMoment_hull_relBody=R_banked'*pitchMoment_hull_relPitch;

%% calculate buoyancy forces
% vector of COM to center of volume of hull in fixed frame
hull_COV_relFixed=R*p.hull.origin;
% vertical height of center of volume of hull above waterline
hull_height=hull_COV_relFixed(3)+z;
% limit sumberged height to radius of hull
if hull_height<-p.hull.radius
    hull_height=-p.hull.radius;
elseif hull_height>p.hull.radius
    hull_height=p.hull.radius;
end
% draft of hull
draft=p.hull.radius-hull_height;
% submerged volume of ellipsoid (hull) approximation
V_sub=-(p.hull.vol_ellipsoid/2)*cos(pi*draft/(2*p.hull.radius))+(p.hull.vol_ellipsoid/2);

%% Net forces and moments due to buoyancy
% net buoyancy force of submerged ellipsoid relative to fixed frame
buoyancyForce_hull_relFixed=[0,0,V_sub*p.density_water*p.g]';

% net buoyancy moment
buoyancyForce_hull_relBody=Rt*buoyancyForce_hull_relFixed;
buoyancyMoment_hull_relBody=cross(p.hull.origin,buoyancyForce_hull_relBody)+...
    pitchMoment_hull_relBody;