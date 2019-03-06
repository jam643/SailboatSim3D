function animate(t,stateArray,p)
% Animates the trajectory of the boat in 3D. Includes the sail, rudder,
% keel, hull, and waterline.
%   Inputs:
%       t           1-N array of time values [sec]
%       stateArray  N-by-12 matrix of state and stateDot values
%       p           structure containing various parameters
%
%   Cornell University
%   Author Name: Jesse Miller 
%   Author NetID: jam643

% update transform matrices
p=boatGeometry(p,stateArray(1,:));

%% set up figure for animation
f=figure(1);
clf
%set fullscreen and figure color
set(f,'units','normalized','outerposition',[0 0 1 1],'color',[.5,.8,1]);

%% unpack length values
hull_l=p.hull.length; hull_r=p.hull.radius;
sail_w=p.sail.width; sail_l=p.sail.length;
keel_w=p.keel.width; keel_l=p.keel.length;
rudder_w=p.rudder.width; rudder_l=p.rudder.length;
% buffer to add to axis limits
buffer=0.6*hull_l;


%% initialize axis limits

limx=[stateArray(1,1)-2*hull_l,stateArray(1,1)+2*hull_l];
limy=[stateArray(1,2)-1.5*hull_l,stateArray(1,2)+1.5*hull_l];
limz=[stateArray(1,3)-1.5*max([sail_l,keel_l,rudder_l,hull_l/2]),stateArray(1,3)+1.5*max([sail_l,keel_l,rudder_l,hull_l/2])];

%% set up plotting options
axis('equal')
%set view angle
az = 5;
el = 30;
view(az, el);
hold on
grid on
% set 3D perspective on
camproj('perspective')
% set axis properties
set(gca,'Unit','normalized','Position',[0 0 1 1],'color',[.5,.8,1],...
'gridlinestyle','--','xcolor',[.3,.6,1],'ycolor',...
    [.3,.6,1],'zcolor',[.3,.6,1])

%% initialize plot objects in simulation
% initialize text box with boat stats
h.boatStats=text(0,0,0,'True Wind','HorizontalAlignment','left',...
    'verticalalignment','bottom','FontSize',10,'color','w','fontweight','bold');
% text indicating true wind
h.trueWind=text(0,0,0,'True Wind','HorizontalAlignment','left',...
    'verticalalignment','bottom','FontSize',12,'fontweight','bold');
% line indicating true wind direction
h.trueWindArrow=plot3(0,0,0,'k','linewidth',2);
% point indicating wind direction
h.trueWindArrowStart=plot3(0,0,0,'ko','linewidth',2,'markersize',10);
% line of trajectory of COM so far
h.trajectory=plot3(0,0,0,'color','k','linewidth',1);
% hull sections
h.hull1=fill3(0,0,0,'k','linewidth',1,'edgecolor','k','FaceAlpha', 0.1);
h.hull2=fill3(0,0,0,[0.3,0.3,0.3],'linewidth',1,'edgecolor','none');
h.hull3=fill3(0,0,0,[0,0,0],'linewidth',1,'edgecolor','none');
h.hull4=fill3(0,0,0,[0.4,0.4,0.4],'linewidth',1,'edgecolor','none');
h.hull5=fill3(0,0,0,[0.1,0.1,0.1],'linewidth',1,'edgecolor','none');
h.hull6=fill3(0,0,0,[0.2,0.2,0.2],'linewidth',1,'edgecolor','none');
% sail 
h.sail=fill3(0,0,0,[1 .5 0],'linewidth',3,'edgecolor',[1 .5 0],'FaceAlpha', 0.7);
% keel
h.keel=fill3(0,0,0,'b','linewidth',3,'edgecolor','b','FaceAlpha', 0.7);
% rudder
h.rudder=fill3(0,0,0,'g','linewidth',3,'edgecolor','g','FaceAlpha', 0.7);
% water surface
h.water=fill3(zeros(1,5),zeros(1,5),zeros(1,5),[.3,.6,1],'FaceAlpha',...
    0.2,'edgecolor','none');

%% plot for each timestep
for m=1:length(t)
    % stop animation if figure is closed
    if ~ishandle(f)
        break;
    end
    
    % extract position and orientation values of boat COM at current time
    x=stateArray(m,1);
    y=stateArray(m,2);
    z=stateArray(m,3);
    phi=stateArray(m,4); %roll
    theta=stateArray(m,5); %pitch
    psi=stateArray(m,6); %yaw
    v_boat=stateArray(m,7:9); % boat velocity
    
    %plot trajectory so far
    set(h.trajectory,'xdata',stateArray(1:m,1),'ydata',stateArray(1:m,2),'zdata',stateArray(1:m,3));
    
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
    if (x-buffer)-limx(1)<0
        Lx=(x-buffer)-limx(1);
    elseif (x+buffer)-limx(2)>0
        Lx=(x+buffer)-limx(2);
    else
        Lx=0;
    end
    if (y-buffer)-limy(1)<0
        Ly=(y-buffer)-limy(1);
    elseif (y+buffer)-limy(2)>0
        Ly=(y+buffer)-limy(2);
    else
        Ly=0;
    end
    if (z-buffer)-limz(1)<0
        Lz=(z-buffer)-limz(1);
    elseif (z+buffer)-limz(2)>0
        Lz=(z+buffer)-limz(2);
    else
        Lz=0;
    end
    limx=limx+Lx; limy=limy+Ly; limz=limz+Lz;
    axis([limx,limy,limz]);
    
    %set water surface location
    set(h.water,'xdata',[limx,limx(2:-1:1),limx(1)],'ydata',[limy(1),limy(1),limy(2),limy(2)]);
    
    % wind arrow direction and position
    windArrow(1,1:3)=[limx(1)+0.5*hull_l,mean(limy),mean(limz)+0.5*(limz(2)-mean(limz))];
    wind_norm=p.v_air_relFixed'/norm(p.v_air_relFixed);
    windArrow(2,:)=windArrow(1,:)+wind_norm*hull_l*0.5;
    %plot true wind text
    set(h.trueWind,'position',windArrow(1,:))
    %plot true wind arrow
    set(h.trueWindArrow,'xdata',windArrow(:,1),'ydata',windArrow(:,2),...
        'zdata',windArrow(:,3))
    %plot true wind direction
    set(h.trueWindArrowStart,'xdata',windArrow(1,1),'ydata',windArrow(1,2),...
        'zdata',windArrow(1,3))
    
    % velocity made good calculation
    v_madeGood=dot(-v_boat,wind_norm);
    %angle of boat trajectory relative to true wind calc
    angle_relWind=180-acosd(dot(v_boat,wind_norm)/norm(v_boat));
    % calculate position of text box
    textPos=[limx(2),mean(limy),mean(limz)];
    % set values of text
    set(h.boatStats,'position',textPos,'string',sprintf(...
        'time=%0.0f sec\n\nV_{boat}=%0.2f m/s\n\nV_{madeGood}=%0.2f m/s\n\n\\Theta_{/wind}=%0.0f^{\\circ}',...
        t(m),norm(v_boat),v_madeGood,angle_relWind))
    
    % pause
    pause(0.035)
end

figure(4)
hold on
axis equal
plot(stateArray(:,1),stateArray(:,2),'-.c','linewidth',2)
xlabel('x-position [m]','fontsize',16);
ylabel('y-position [m]','fontsize',16);
% 
% figure(5)
% hold on
% plot(t,atan2(stateArray(:,2),stateArray(:,1)),'b','linewidth',2)
% xlabel('Time [sec]','fontsize',16);
% ylabel('Direction [rad]','fontsize',16);

figure(6)
hold on
plot(t,stateArray(:,4),'--r','linewidth',2)
xlabel('Time [sec]','fontsize',16);
ylabel('Heel Angle [rad]','fontsize',16);