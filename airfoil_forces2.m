function [netForce_relFixed, netMoment_relBody] = airfoil_forces2(state,airfoil,p)
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
%       netForce_relFixed   3-by-1 array of total force on airfoil relative
%                           to fixed frame
%       netMoment_relBody   3-by-1 array of total moment about boat COM
%                           caused by airfoil relative to boat frame
%
%   Cornell University
%   Author Name: Jesse Miller 
%   Author NetID: jam643

v_boat_relFixed=state(7:9);
w_boat_relBody=state(10:12);
k_hat=[0,0,1]';

% rotation matrix of airfoil relative to fixed frame
R=p.boat.R*airfoil.R;
% rotation matrix transpose relative to fixed frame
Rt=R.';

%number of blade elements use to mesh airfoil
nElements=p.nElements;
%surface area of each blade element
SA_element=airfoil.SA/nElements;
%aspect ratio
AR=airfoil.length/airfoil.width;
%points at center of blade elements location relative to boat frame
points_airfoil_relBody=airfoil.H*[airfoil.points_relAirfoil;...
    ones(1,nElements)];

%calculate forces and moments on each blade element of airfoil
for m=1:nElements
    % current point on airfoil relative to fixed frame
    point_relFixed=p.boat.H*points_airfoil_relBody(:,m);
    
    if point_relFixed(3)>0 %point is above water surface
        % say airfoil is in the air
        density=p.density_air;
        v_fluid_relFixed=p.v_air_relFixed;
    else
        % say airfoil is in water
        density=p.density_water;
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
    
%     COP_airfoil_relBody=points_airfoil_relBody(1:3,m)-...
%         [abs(airfoil.width*alpha_airfoil/(2*pi));0;0];

    COP_airfoil_relBody=points_airfoil_relBody(1:3,m)+...
        [0.25*airfoil.width*(cos(abs(wrapToPi(alpha_airfoil)))-1);0;0];
    
%     0.25*(cos(abs(wrapToPi(alpha_airfoil)))-1)
    
    %interpolated lift and drag coefficients
    [c_lift,c_drag0]=C_LD(alpha_airfoil,p);
    
    %induced drag
    c_dragi=c_lift^2/(0.7*pi*AR);
    
    %total drag
    c_drag=c_dragi+c_drag0;
    
    % lift and drag calculations relative to frame oriented with airfoil
    lift_relAirfoil=.5*density*SA_element*norm(v_airfoil_relAirfoil)*...
        c_lift*cross(k_hat,v_airfoil_relAirfoil);
    drag_relAirfoil=.5*density*SA_element*norm(v_airfoil_relAirfoil)*...
        c_drag*(-v_airfoil_relAirfoil);
    
    % lift and drag forces relative to boat frame
    lift_relBody=airfoil.R*lift_relAirfoil;
    drag_relBody=airfoil.R*drag_relAirfoil;
    
    % total force on blade element added to array
    forces_relBody(:,m)=lift_relBody + drag_relBody;
    % total moment caused by blade element added to array
    moments_relBody(:,m)=cross(COP_airfoil_relBody,forces_relBody(:,m));
end

%net force and moment calculations
netMoment_relBody=sum(moments_relBody,2);
netForce_relFixed=p.boat.R*sum(forces_relBody,2);