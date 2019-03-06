function main(boatName)
% Main script to run SailSim
%   Inputs:
%       boatName    string name of .mat file created with makeBoat.m
%                   function containing boat parameter information
%
%   Cornell University
%   Author Name: Jesse Miller 
%   Author NetID: jam643

% get boat parameters
[p,state0]=setBoatParam(boatName);

%Runs the simulation using pre-calculated trajectory or real-time boat
%control
if p.realTime == 0 %pre-calculate trajectory
    %set accuracy of ode solver
    options=odeset('abstol',1e-4,'reltol',1e-4);
    %initialize loading bar
    p.waitBar=waitbar(0,'Sailing...');
    %numerically solve for boat trajectory
    [tarray,stateArray]=ode23(@rhs,p.tspan,state0,options,p);
    %close the loading bar
    close(p.waitBar);
    %animate the resulting trajectory
    animate(tarray,stateArray,p);
elseif p.realTime == 1 %allow real time control of boat
    %solve for trajectory at each time step in real-time using Euler
    %approximation
    [tarray,stateArray]=odeEuler(@rhs,state0,p);
end
