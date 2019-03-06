function [dampingForce_hull_relFixed,dampingMoment_hull_relBody]=hullForce_damping(state,p)
% Calculates forces and moments in the fixed and body frames respectively
% related to buoyance forces
%   Inputs:
%       state                           12-by-1 array of the current state of the boat
%       p                               structure containing various parameters
%   Outputs:
%       dampingForce_hull_relFixed      3-by-1 net hull damping force 
%                                       relative to fixed frame
%       dampingMoment_hull_relBody      3-by-1 net hull damping moment 
%                                       relative to boat frame
%
%   Cornell University
%   Author Name: Jesse Miller
%   Author NetID: jam643

%% unpack variables
z=state(3);
zdot=state(9);
q=state(11);
r=state(12);
Rt=p.boat.Rt; % transpose of boat rotation matrix

%% vertical drag force on hull
% cross sectional area of hull approximation
Area=pi*p.hull.radius*p.hull.length/2;

% use drag coefficient of flat plate if hull is submerged
if z-p.hull.radius < 0
    density_fluid=p.density_water;
else
    density_fluid=p.density_air;
end
Cd=1.28;
% vertical damping force in fixed frame
zDragForce_hull_relFixed=[0,0,-0.5*density_fluid*Cd*Area*abs(zdot)*zdot]';
% moment in body frame from vertical damping force
zDragForce_hull_relBody=Rt*zDragForce_hull_relFixed;
zDragMoment_hull_relBody=cross(p.hull.origin,zDragForce_hull_relBody);

%% rotational damping on the hull in pitch and yaw directions
%coefficient for rotational damping
coeff=(1/2)*(p.hull.length/4)^3*density_fluid*Area*Cd;
rotDampMoment_hull_relBody=[0,-coeff*q*abs(q),-coeff*r*abs(r)]';

%% net force and moment due to damping
dampingMoment_hull_relBody=rotDampMoment_hull_relBody+zDragMoment_hull_relBody;
dampingForce_hull_relFixed=zDragForce_hull_relFixed;
