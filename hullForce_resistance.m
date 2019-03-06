function [resistanceForce_hull_relFixed,resistanceMoment_hull_relBody]=hullForce_resistance(state,p)
% Calculates forces and moments in the fixed and body frames respectively
% related to buoyance forces
%   Inputs:
%       state                           12-by-1 array of the current state of the boat
%       p                               structure containing various parameters
%   Outputs:
%       resistForce_hull_relFixed     3-by-1 net hull resistance force
%                                       relative to fixed frame
%       resistMoment_hull_relBody     3-by-1 net hull resistance moment 
%                                       relative to boat frame
%
%   Cornell University
%   Author Name: Jesse Miller
%   Author NetID: jam643

%% unpack variables
z=state(3); % vertical height of COM
psi=state(6); % yaw
v_boat_relFixed=state(7:9); % [xdot,ydot,zdot]'
Rt=p.boat.Rt; % transpose of boat rotation matrix

%% calculate forces differently if hull is above or below water
if z-p.hull.radius < 0 % if hull is in the water
    % apparent velocity of hull
    v_boat_relWater=v_boat_relFixed-p.v_water_relFixed;
    % apparent velocity of hull projected on xy plane
    v_boatXy_relWater=[v_boat_relWater(1:2);0];
    % direction of water current
    angle_boat_apparent=atan2(v_boatXy_relWater(2),v_boatXy_relWater(1));
    % angle of attack of hull in water
    alpha_hull=wrapTo2Pi(psi-angle_boat_apparent);
    
    % sinusoidally interpolate location of center of pressure of hull from quarter
    % chord from bow to quarter chord from stern as angle of attack varies from
    % 0 to 180 degrees
    center_of_pressure=p.hull.length*0.25*cos(abs(wrapToPi(alpha_hull)));
    
    %% net force and moment due to hull resistance
    % hull resistance proportional to v^3 with coefficient based on experiment
    % This experement was performed on a 1 meter long hull
    % This equation may need to be updated for different hull types
    resistanceForce_hull_relFixed=6.5*norm(v_boatXy_relWater)^2*(-v_boatXy_relWater);
%     resistanceForce_hull_relFixed=[1.3872*norm(v_boatXy_relWater(1:2))^3*(-v_boatXy_relWater(1:2));0];
    
    % represent hull resistance in boat frame
    resistanceForce_hull_relBody=Rt*resistanceForce_hull_relFixed;
    % calculate resulting moment in z direction
    resistanceMoment_hull_relBody=cross([center_of_pressure,0,0]',...
        resistanceForce_hull_relBody);
else % if hull is above water surface
    resistanceMoment_hull_relBody=zeros(3,1);
    resistanceForce_hull_relFixed=zeros(3,1);
end