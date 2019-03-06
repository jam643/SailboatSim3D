function [H,Ht,R,Rt]=euler2Hom(rot,trans)
% Calculates homogeneous matrix (H), its inverse (Ht), rotation matrix (R),
% its transpose (Rt)
%   Inputs:
%       rot     1-by-3 vector of [phi,theta,psi] euler angles
%       trans   1-by-3 vector of [x,y,z] translation
%
%   Outputs:
%       H       4-by-4 homogeneous transform matrix
%       Ht      4-by-4 homogeneous transform matrix inverse
%       R       3-by-3 homogeneous transform matrix
%       Rt      3-by-3 rotation matrix inverse
%
%   Cornell University
%   Author Name: Jesse Miller 
%   Author NetID: jam643

%unpack inputs
phi=rot(1); theta=rot(2); psi=rot(3);
x=trans(1); y=trans(2); z=trans(3);

% rotation matrix based on roll pitch yaw euler angles
R=[ cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta);...
   cos(theta)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta), cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi);...
       -sin(theta),                  cos(theta)*sin(phi),                              cos(phi)*cos(theta)];

% rotation matrix transpose
Rt=R.';   

% homog. transform matrix
Htemp=[R,[x;y;z]];
H=[Htemp;[0,0,0,1]];

% homog. transform matrix inverse
Httemp=[Rt,-Rt*[x;y;z]];
Ht=[Httemp;[0,0,0,1]];