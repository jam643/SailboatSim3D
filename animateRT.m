function h=animateRT(t,stateArray,p,h)
% Animates the trajectory of the boat in 3D. Includes the sail, rudder,
% keel, hull, and waterline. Plots a single frame of animation which is
% running in real time.
%   Inputs:
%       t           1-N array of time values [sec]
%       stateArray  N-by-12 matrix of state and stateDot values
%       p           structure containing various parameters
%       h           structure containing plotting handles
%
%   Cornell University
%   Author Name: Jesse Miller 
%   Author NetID: jam643

%% unpack length values
hull_l=p.hull.length; hull_r=p.hull.radius;
sail_w=p.sail.width; sail_l=p.sail.length;
keel_w=p.keel.width; keel_l=p.keel.length;
rudder_w=p.rudder.width; rudder_l=p.rudder.length;
% buffer to add to axis limits
buffer=0.6*hull_l;

%% extract position and orientation values of boat COM at current time
x=stateArray(end,1);
y=stateArray(end,2);
z=stateArray(end,3);
phi=stateArray(end,4); %roll
theta=stateArray(end,5); %pitch
psi=stateArray(end,6); %yaw
v_boat=stateArray(end,7:9); % boat velocity

%% plot boat components
% plot trajectory so far
set(h.trajectory,'xdata',stateArray(1:end,1),'ydata',stateArray(1:end,2),'zdata',stateArray(1:end,3));

% Rotate, translate, plot hull section
H=euler2Hom([phi,theta,psi],[x,y,z]);
H_hull=H*p.hull.H;
x_hull=[-0.5*hull_l,0.05*hull_l,0.5*hull_l,0.05*hull_l,-0.5*hull_l,-0.5*hull_l];
y_hull=[-.5*hull_r,-hull_r,0,hull_r,0.5*hull_r,-.5*hull_r];
z_hull=zeros(size(x_hull));
p0_hull=[x_hull;y_hull;z_hull;zeros(size(x_hull))+1];
p_hull=H_hull*p0_hull;
set(h.hull1,'xdata',p_hull(1,:),'ydata',p_hull(2,:),'zdata',p_hull(3,:));

% Rotate, translate, plot hull section
x_hull=[-0.5*hull_l,0.05*hull_l,0.05*hull_l,-0.5*hull_l,-0.5*hull_l];
y_hull=[.5*hull_r,hull_r,hull_r,.5*hull_r,0.5*hull_r];
z_hull=[0,0,-0.7*hull_r,-0.5*hull_r,0];
p0_hull=[x_hull;y_hull;z_hull;zeros(size(x_hull))+1];
p_hull=H_hull*p0_hull;
set(h.hull2,'xdata',p_hull(1,:),'ydata',p_hull(2,:),'zdata',p_hull(3,:));

% Rotate, translate, plot hull section
x_hull=[-0.5*hull_l,-0.5*hull_l,-0.5*hull_l,-0.5*hull_l,-0.5*hull_l];
y_hull=[-.5*hull_r,.5*hull_r,.5*hull_r,-.5*hull_r,-.5*hull_r];
z_hull=[0,0,-0.5*hull_r,-0.5*hull_r,0];
p0_hull=[x_hull;y_hull;z_hull;zeros(size(x_hull))+1];
p_hull=H_hull*p0_hull;
set(h.hull3,'xdata',p_hull(1,:),'ydata',p_hull(2,:),'zdata',p_hull(3,:));

% Rotate, translate, plot hull section
x_hull=[0.05*hull_l,0.5*hull_l,0.05*hull_l,0.05*hull_l];
y_hull=[hull_r,0,hull_r,hull_r];
z_hull=[0,0,-0.7*hull_r,0];
p0_hull=[x_hull;y_hull;z_hull;zeros(size(x_hull))+1];
p_hull=H_hull*p0_hull;
set(h.hull4,'xdata',p_hull(1,:),'ydata',p_hull(2,:),'zdata',p_hull(3,:));

% Rotate, translate, plot hull section
x_hull=[-0.5*hull_l,0.05*hull_l,0.05*hull_l,-0.5*hull_l,-0.5*hull_l];
y_hull=[-.5*hull_r,-hull_r,-hull_r,-.5*hull_r,-.5*hull_r];
z_hull=[0,0,-0.7*hull_r,-0.5*hull_r,0];
p0_hull=[x_hull;y_hull;z_hull;zeros(size(x_hull))+1];
p_hull=H_hull*p0_hull;
set(h.hull5,'xdata',p_hull(1,:),'ydata',p_hull(2,:),'zdata',p_hull(3,:));

% Rotate, translate, plot hull section
x_hull=[0.05*hull_l,0.5*hull_l,0.05*hull_l,0.05*hull_l];
y_hull=[-hull_r,0,-hull_r,-hull_r];
z_hull=[0,0,-0.7*hull_r,0];
p0_hull=[x_hull;y_hull;z_hull;zeros(size(x_hull))+1];
p_hull=H_hull*p0_hull;
set(h.hull6,'xdata',p_hull(1,:),'ydata',p_hull(2,:),'zdata',p_hull(3,:));

% Rotate, translate, plot sail
H_sail=H*p.sail.H;
x_sail=[-sail_w/2,sail_w/2,sail_w/2,-sail_w/2,-sail_w/2];
y_sail=zeros(size(x_sail));
z_sail=[0,0,sail_l,sail_l,0]*p.sail.direction;
p0_sail=[x_sail;y_sail;z_sail;zeros(size(x_sail))+1];
p_sail=H_sail*p0_sail;
set(h.sail,'xdata',p_sail(1,:),'ydata',p_sail(2,:),'zdata',p_sail(3,:));

% Rotate, translate, plot keel
H_keel=H*p.keel.H;
x_keel=[-keel_w/2,keel_w/2,keel_w/2,-keel_w/2,-keel_w/2];
y_keel=zeros(size(x_keel));
z_keel=[0,0,keel_l,keel_l,0]*p.keel.direction;
p0_keel=[x_keel;y_keel;z_keel;zeros(size(x_keel))+1];
p_keel=H_keel*p0_keel;
set(h.keel,'xdata',p_keel(1,:),'ydata',p_keel(2,:),'zdata',p_keel(3,:));

% Rotate, translate, plot hull section
%homogeneous transform of rudder different if it's attached to sail
if p.rudder.type==1
    H_rudder=H*p.rudder.H;
elseif p.rudder.type==2
    H_rudder=H*p.sail.H*p.rudder.H_relSail;
end
x_rudder=[-rudder_w/2,rudder_w/2,rudder_w/2,-rudder_w/2,-rudder_w/2];
y_rudder=zeros(size(x_rudder));
z_rudder=[0,0,rudder_l,rudder_l,0]*p.rudder.direction;
p0_rudder=[x_rudder;y_rudder;z_rudder;zeros(size(x_rudder))+1];
p_rudder=H_rudder*p0_rudder;
set(h.rudder,'xdata',p_rudder(1,:),'ydata',p_rudder(2,:),'zdata',p_rudder(3,:));

%only change axis limits if boat is leaving the axis
if (x-buffer)-h.limx(1)<0
    Lx=(x-buffer)-h.limx(1);
elseif (x+buffer)-h.limx(2)>0
    Lx=(x+buffer)-h.limx(2);
else
    Lx=0;
end
if (y-buffer)-h.limy(1)<0
    Ly=(y-buffer)-h.limy(1);
elseif (y+buffer)-h.limy(2)>0
    Ly=(y+buffer)-h.limy(2);
else
    Ly=0;
end
if (z-buffer)-h.limz(1)<0
    Lz=(z-buffer)-h.limz(1);
elseif (z+buffer)-h.limz(2)>0
    Lz=(z+buffer)-h.limz(2);
else
    Lz=0;
end
h.limx=h.limx+Lx; h.limy=h.limy+Ly; h.limz=h.limz+Lz;
axis([h.limx,h.limy,h.limz]);

%set water surface location
set(h.water,'xdata',[h.limx,h.limx(2:-1:1),h.limx(1)],'ydata',[h.limy(1),h.limy(1),h.limy(2),h.limy(2)]);

% location of true wind arrow
windArrow(1,1:3)=[h.limx(1),mean(h.limy),mean(h.limz)+0.5*(h.limz(2)-mean(h.limz))];
wind_norm=p.v_air_relFixed'/norm(p.v_air_relFixed);
windArrow(2,:)=windArrow(1,:)+wind_norm*hull_l*0.5;
% set position and text of wind arrow
set(h.trueWind,'position',windArrow(1,:),'string',sprintf('True Wind = %0.0f m/s',norm(p.v_air_relFixed)))
% set position of true wind arrow
set(h.trueWindArrow,'xdata',windArrow(:,1),'ydata',windArrow(:,2),...
    'zdata',windArrow(:,3));
% set position of start of arrow
set(h.trueWindArrowStart,'xdata',windArrow(1,1),'ydata',windArrow(1,2),...
    'zdata',windArrow(1,3))

% velocity made good calculation
v_madeGood=dot(-v_boat,wind_norm);
%angle of boat trajectory relative to true wind calc
angle_relWind=180-acosd(dot(v_boat,wind_norm)/norm(v_boat));
lt=length(t);

% calculate airfoil efficiency for sail, keel, and rudder
[LD_efficiency_sail] = airfoil_efficiency(stateArray(end,:)',p.sail,p);
[LD_efficiency_keel] = airfoil_efficiency(stateArray(end,:)',p.keel,p);
[LD_efficiency_rudder] = airfoil_efficiency(stateArray(end,:)',p.rudder,p);

%calculate fps of animation
if mod(lt,10) == 0
    h.fps=1/mean(t(lt-8:lt)-t(lt-9:lt-1));
end
% calculate position of text box
textPos=[h.limx(2),mean(h.limy),mean(h.limz)];
% set values of text
set(h.boatStats,'position',textPos,'string',sprintf(...
    'time=%0.0f sec\n\nV_{boat}=%0.2f m/s\n\nV_{madeGood}=%0.2f m/s\n\n\\Theta_{/wind}=%0.0f^{\\circ}\n\n\\eta_{sail}=%0.2f\n\n\\eta_{keel}=%0.2f\n\n\\eta_{rudder}=%0.2f\n\nFPS=%0.0f',...
    t(end),norm(v_boat),v_madeGood,angle_relWind,LD_efficiency_sail,...
    LD_efficiency_keel,LD_efficiency_rudder,h.fps))
