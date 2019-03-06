function angle=wrapTo2Pi(angle)
% convert angle [rad] to be in range [0,2pi]
%   Inputs:
%       angle   angle in radians
%   Outputs:
%       angle   angle in radians in range [0,2pi]

for k=1:length(angle)
    angle(k)=rem(angle(k),2*pi);
    if angle(k)<0
        angle(k)=2*pi+angle(k);
    end
end