function angle=wrapToPi(angle)
% convert angle [rad] to be in range [-pi,pi]
%   Inputs:
%       angle   angle in radians
%   Outputs:
%       angle   angle in radians in range [-pi,pi]

angle=atan2(sin(angle),cos(angle));