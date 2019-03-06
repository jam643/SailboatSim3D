function [R,Rt]=euler2Rot(phi,theta,psi)
% Calculates rotation matrix (R and its transpose (Rt)
%   Inputs:
%       phi     Euler lean angle
%       theta   Euler pitch angle
%       psi     Euler yaw angle
%
%   Outputs:
%       R       3-by-3 homogeneous transform matrix
%       Rt      3-by-3 rotation matrix inverse
%
%   Cornell University
%   Author Name: Jesse Miller 
%   Author NetID: jam643

% rotation matrix
R=[ cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta);...
   cos(theta)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta), cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi);...
       -sin(theta),                  cos(theta)*sin(phi),                              cos(phi)*cos(theta)];

% rotation matrix transpose
Rt=R.';