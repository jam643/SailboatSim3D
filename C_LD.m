function [c_lift,c_drag]=C_LD(alpha,p)
% Calculates lift and drag coefficients based on angle of attack alpha
%   Inputs:
%       alpha       angle of attack [rad]
%       p           structure containing various parameters
%
%   Outputs:
%       c_lift      lift coefficient
%       c_drag      drag coefficient
%
%   Cornell University
%   Author Name: Jesse Miller 
%   Author NetID: jam643

if p.accuracy==1
    %interpolation with pchip
    alpha=alpha*180/pi;
    c_lift=ppval(p.lift,alpha);
    c_drag=ppval(p.drag,alpha);
else
    %approximation as sinusoidal
    c_lift=p.C0*sin(2*alpha);
    c_drag=p.paraDrag+p.C0*(1-cos(2*alpha));
end
