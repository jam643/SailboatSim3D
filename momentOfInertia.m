function I=momentOfInertia(mass,l1,l2,d1,d2)
% Calculates moment of intertia of a rectangular mass of length and width
% (l1 and l2) offset a distance (d1 and d2) from the axis of rotation.
%
%   Cornell University
%   Author Name: Jesse Miller 
%   Author NetID: jam643

I=(1/12)*mass*(l1^2+l2^2)+mass*(d1+d2)^2;
