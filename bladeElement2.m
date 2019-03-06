function [points_relAirfoil] = bladeElement2(airfoil,p)
% Uses blade element theory to calculated center of elements along airfoil
% given a desired number of nElements relative to a frame at the base of
% the airfoil
%   Inputs:
%       airfoil             structure containing information about the airfoil
%       p                   structure containing various parameters
%
%   Outputs:
%       points_relAirfoil   3-by-nElements array of locations of center of
%                           blade elements relative to the frame at the    
%                           base of the airfoil
%
%   Cornell University
%   Author Name: Jesse Miller 
%   Author NetID: jam643

% number of blade elements
nElements=p.nElements;
%unpack variables
length=airfoil.length;
direction=airfoil.direction;
k=[0,0,1]'; %unit vector in z direction
points_relAirfoil=zeros(3,nElements); %initialize array

for m=1:nElements % for each element
    % calculate point at center of each blade element
    points_relAirfoil(:,m) = (m/(nElements+1))*length*direction*k;
end