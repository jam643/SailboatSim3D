function [LD_efficiency] = airfoil_efficiency(state,airfoil,p)
% Calculates net force relative to the fixed frame and net moment relative
% to the boat frame. The airfoil can be translating and rotating. Blade
% element theory is used to improve the accuracy of the lift and drag
% calculations.
%   Inputs:
%       state               12-by-1 vector of state and stateDot values
%       airfoil             structure containing information about the airfoil
%       p                   structure containing various parameters
%
%   Outputs:
%       LD_efficiency       ratio of current Lift/Drag ratio of airfoil
%                           over maximum Lift/Drag ratio
%
%   Cornell University
%   Author Name: Jesse Miller 
%   Author NetID: jam643

v_boat_relFixed=state(7:9);
w_boat_relBody=state(10:12);

% rotation matrix of airfoil relative to fixed frame
R=p.boat.R*airfoil.R;
% rotation matrix transpose relative to fixed frame
Rt=R.';

%number of blade elements use to mesh airfoil
nElements=p.nElements;

%points at center of blade elements location relative to boat frame
points_airfoil_relBody=airfoil.H*[airfoil.points_relAirfoil;...
    ones(1,nElements)];

efficiency=zeros(1,nElements);

%calculate forces and moments on each blade element of airfoil
for m=1:nElements
    % current point on airfoil relative to fixed frame
    point_relFixed=p.boat.H*points_airfoil_relBody(:,m);
    
    if point_relFixed(3)>0 %point is above water surface
        v_fluid_relFixed=p.v_air_relFixed;
    else % point is below water surface
        v_fluid_relFixed=p.v_water_relFixed;
    end
    
    % Apparent velocity of COM of boat rel to fixed frame
    v_app_relFixed=v_fluid_relFixed-v_boat_relFixed;
    % apparent velocity of COM of boat rel to airfoil frame
    v_app_relAirfoil=Rt*v_app_relFixed;
    
    % linear velocity component caused by rotation of boat relative to boat
    % frame
    rot2linVel_relBody=cross(w_boat_relBody,points_airfoil_relBody(1:3,m));
    % rot. and trans. velocity of point on airfoil relative to fixed frame
    % that instantaneously corresponds with a frame fixed to airfoil
    v_airfoil_relAirfoil=airfoil.Rt*rot2linVel_relBody-v_app_relAirfoil;
    v_airfoil_relAirfoil=[v_airfoil_relAirfoil(1:2);0];
    % angle of attack of point on airfoil
    alpha_airfoil=wrapTo2Pi(-atan2(v_airfoil_relAirfoil(2),v_airfoil_relAirfoil(1)));
    [c_lift,c_drag]=C_LD(alpha_airfoil,p);
    
    %airfoil efficiency at each point
    efficiency(m)=abs(c_lift/c_drag)/p.maxLDratio;
end
LD_efficiency=mean(efficiency);