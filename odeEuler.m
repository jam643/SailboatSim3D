function [t,stateArray]=odeEuler(fhandle,stateArray0,p)
% Uses Euler method to estimate trajectory of sailboat. Allows for user
% input through keyboard keypress callback functions to control the boat.
% Animates the boat motion in real-time.
%   Inputs:
%       fhandle             function handle of rhs function
%       stateArray0         12-by-1 array of the initial state of the boat
%       p                   structure containing various parameters
%
%   Outputs:
%       t                   1-by-N time span array [sec]
%       stateArray          12-by-N state of boat at each time step
%
%   Cornell University
%   Author Name: Jesse Miller 
%   Author NetID: jam643

%% initialize parameters
t=0; %initial time of simulation
%initial state
stateArray0=stateArray0';
stateArray=stateArray0;
keyspeed = 1; % speed effect that keypresses have on changing the values

%% Initialize figure object
f=figure(1);
cla;
% set figure size and color
set(f,'units','normalized','outerposition',[0 0 1 1],'color',[.5,.8,1]);
% set up callback functions when a key is pressed
set(f,'WindowKeyPressFcn',@KeyPress,'WindowKeyReleaseFcn',@KeyRelease);

%% Initialize bar plot to display rudder angle
subplot(2,22,22)
barLim_rudder = [-180 180];   %Bar scale
if p.rudder.type==2
    bar_rudder = bar(round(180*p.rudder.angle_relSail/pi)); %Initialize the bar
elseif p.rudder.type==1
    bar_rudder = bar(round(180*p.rudder.angle_relBody/pi)); %Initialize the bar
end
bar_rudder_title = title(sprintf('Rudder Angle = %d degrees',get(bar_rudder,'ydata')),'fontweight','bold');
ylim(barLim_rudder);       %Set upper limit
xlabel('Up/Down Arrow');
set(gca, 'XTick', []);

%% Initialize bar plot to display sail angle
subplot(2,22,44)
barLim_sail = [-180 180];   %Bar scale
bar_sail = bar(round(180*p.sail.angle_relBody/pi));     %Initialize the bar
ylim(barLim_sail);       %Set upper limit
xlabel('Left/Right Arrow');
set(gca, 'XTick', []);
bar_sail_title=title(sprintf('Sail Angle =  %d degrees',get(bar_sail,'ydata')),'fontweight','bold');

%% Initialize main plot area for animation
%axis size and color
h.main_axis=subplot(2,22,[1:20,23:42],'color',[.5,.8,1]);
axis('equal')
axis vis3d
hold on
grid on
if p.threeD
    camproj('perspective') % 3D perspective
    %initial view perspective
    h.az = 5;
    h.el = 30;
else
    %initial view perspective
    h.az=0;
    h.el=90;
end
view(h.az, h.el);
h.fps=NaN;  % frames per second of animation
% set axis properties
set(gca,...
'gridlinestyle','--','xcolor',[.3,.6,1],'ycolor',...
    [.3,.6,1],'zcolor',[.3,.6,1],'yticklabel',[],'xticklabel',[],'zticklabel',[])
% initialize plot objects in simulation
h.trueWind=text(0,0,0,'True Wind','HorizontalAlignment','left',...
    'verticalalignment','bottom','FontSize',10,'fontweight','bold');
% initialize text box with boat stats
h.boatStats=text(0,0,0,'True Wind','HorizontalAlignment','left',...
    'verticalalignment','middle','FontSize',10,'color','w','fontweight','bold');
% initialize arrow representing true wind
h.trueWindArrow=plot3(0,0,0,'k','linewidth',2);
% initialize start of true wind arrow
h.trueWindArrowStart=plot3(0,0,0,'ko','linewidth',2,'markersize',10);
% initialize boat trajectory line
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

%% initialize axis limits
h.limx=[stateArray0(1,1)-2*p.hull.length,stateArray0(1,1)+2*p.hull.length];
h.limy=[stateArray0(1,1)-1.5*p.hull.length,stateArray0(1,1)+1.5*p.hull.length];
h.limz=[stateArray0(1,3)-1.5*max([p.sail.length,p.keel.length,...
    p.rudder.length,p.hull.length/2]),stateArray0(1,3)+...
    1.5*max([p.sail.length,p.keel.length,p.rudder.length,p.hull.length/2])];
 
%% Initialize the arrow keys as not being pressed
commands.up = false;
commands.down = false;
commands.left = false;
commands.right = false;
commands.w = false;
commands.s = false;
commands.d = false;
commands.a = false;
commands.h = false;
commands.f = false;
set(f,'UserData',commands); %Store these commands in the figure object

%% Run simulation by continuously using Euler method to solve ODEs
tic % start timer
while(ishandle(f))
    
    % update view
    view(h.az, h.el);
    
    %% get updated rudder and sail angles from bar plots
    % get rudder angle
    bar2val_rudder = get(bar_rudder,'ydata');
    p.rudder.angle_relBody = pi*bar2val_rudder/180; %convert to rad
    p.rudder.angle_relSail=p.rudder.angle_relBody;
    % get sail angle
    bar2val_sail=get(bar_sail,'ydata');
    p.sail.angle_relBody=pi*bar2val_sail/180; % convert to rad
    % get sail and rudder angles from servo rates if servoOn is set
    if p.servoOn
        % update sail angle
        if t(end)>p.sail.timeVect(end)
            p.sail.angle_relBody=p.sail.angleVect(end);
        elseif t(end)<p.sail.timeVect(1)
            p.sail.angle_relBody=p.sail.angleVect(1);
        else p.servoOn
            p.sail.angle_relBody=interp1(p.sail.timeVect,p.sail.angleVect,t(end));
        end
        % update tail angle
        if t(end)>p.rudder.timeVect(end)
            p.rudder.angle_relBody=p.rudder.angleVect(end);
        elseif t(end)<p.rudder.timeVect(1)
            p.rudder.angle_relBody=p.rudder.angleVect(1);
        else p.servoOn
            p.rudder.angle_relBody=interp1(p.rudder.timeVect,p.rudder.angleVect,t(end));
        end
        p.rudder.angle_relSail=p.rudder.angle_relBody;
    end
    
    %% update current time and state 
    t(end+1)=toc; % measure current time
    dt=t(end)-t(end-1); % calculate dt since last time
    statePrevious=stateArray(end,:)'; % previous state
    
    %% update COM location and moments of inertia since tail pose may change 
    p=boatGeometry(p,statePrevious);
    
    %% integrate using Euler's Method
    % integrate with Euler's method numInt times since last state
    for k=1:p.numInt
        % calculate rhs dynamics to get d/dt(state)=statedot
        statedot=feval(fhandle,t,statePrevious,p);
        % use Euler method update state
        statePrevious=statePrevious+(1/p.numInt)*dt*statedot;
    end
    % set to be current stae
    state=statePrevious;
    %update state array with current state
    stateArray(end+1,:)=state';
    
    %% Animate current time frame
    % stop simulation if figure has been closed
    if ~ishandle(f)
        break;
    end
    % animate current state and update h structure with plot handles
    h=animateRT(t,stateArray,p,h);
    

   %% Get user input from the keyboard
   if ishandle(f)
       commands = get(f,'UserData') ;
   end
   
   %% Adjust the bar position if the user has given input
   % update rudder angle if up or down arrows pressed
   if (commands.up && bar2val_rudder<=barLim_rudder(2))
       set(bar_rudder,'ydata',bar2val_rudder + keyspeed);
       set(bar_rudder_title,'string',sprintf('Rudder Angle = %d degrees',round(get(bar_rudder,'ydata'))));
   elseif (commands.up && bar2val_rudder>=barLim_rudder(2))
       set(bar_rudder,'ydata',barLim_rudder(1));
       set(bar_rudder_title,'string',sprintf('Rudder Angle = %d degrees',round(get(bar_rudder,'ydata'))));
   elseif (commands.down && bar2val_rudder >=barLim_rudder(1))
       set(bar_rudder,'ydata',bar2val_rudder - keyspeed);
       set(bar_rudder_title,'string',sprintf('Rudder Angle = %d degrees',round(get(bar_rudder,'ydata'))));
   elseif (commands.down && bar2val_rudder <=barLim_rudder(1))
       set(bar_rudder,'ydata',barLim_rudder(2));
       set(bar_rudder_title,'string',sprintf('Rudder Angle = %d degrees',round(get(bar_rudder,'ydata'))));
   % update sail angle if left of right arrows pressed
   elseif (commands.right && bar2val_sail <= barLim_sail(2))
       set(bar_sail,'ydata',bar2val_sail + keyspeed);
       set(bar_sail_title,'string',sprintf('Sail Angle = %d degrees',round(get(bar_sail,'ydata'))));
   elseif (commands.right && bar2val_sail >= barLim_sail(2))
       set(bar_sail,'ydata',barLim_sail(1));
       set(bar_sail_title,'string',sprintf('Sail Angle = %d degrees',round(get(bar_sail,'ydata'))));
   elseif (commands.left && bar2val_sail >= barLim_sail(1))
       set(bar_sail,'ydata',bar2val_sail - keyspeed);
       set(bar_sail_title,'string',sprintf('Sail Angle = %d degrees',round(get(bar_sail,'ydata'))));
   elseif (commands.left && bar2val_sail <= barLim_sail(1))
       set(bar_sail,'ydata',barLim_sail(2));
       set(bar_sail_title,'string',sprintf('Sail Angle = %d degrees',round(get(bar_sail,'ydata'))));
   % update elevation angles if w or s keys are pressed
   elseif commands.w && (h.el + keyspeed) < 90
       h.el=h.el + keyspeed;
   elseif commands.s && (h.el- keyspeed) > -89
       h.el=h.el - keyspeed;
   % update azimuth angle if d or a keys are pressed
   elseif commands.d
       h.az=h.az + keyspeed;
   elseif commands.a
       h.az=h.az - keyspeed;
   % update azimuth angle if d or a keys are pressed
   elseif commands.h
       p.sail.length=p.sail.length+0.005;
       p.sail.width=p.sail.length/4;
   elseif commands.f
       p.sail.length=p.sail.length-0.005;
       p.sail.width=p.sail.length/4;
   end
   
   %Pause the program for a tiny amount of time. This is necessary to make
   %it work.
   pause(0.01);
end

%%%%%%%%%%% KEYBOARD CALLBACKS %%%%%%%%%%% 
function KeyPress(varargin)
    %This function is continuously running and checking the keyboard input.
    %It takes in varargin, which stands for variable argument input. 
     fig = varargin{1};
     key = varargin{2}.Key;
     %Change 'UserData' part of the figure object depending on the input.
     if strcmp(key,'uparrow')
         x=get(fig,'UserData'); %Get the UserData
         x.up = true;           %Update the command
         set(fig,'UserData',x); %Put the new UserData back into the object
     elseif strcmp(key,'downarrow')
         x=get(fig,'UserData');
         x.down = true;
         set(fig,'UserData',x);
     elseif strcmp(key,'rightarrow')
         x=get(fig,'UserData');
         x.right=true;
         set(fig,'UserData',x);
     elseif strcmp(key,'leftarrow')
         x=get(fig,'UserData');
         x.left=true;
         set(fig,'UserData',x);
     elseif strcmp(key,'w')
         x=get(fig,'UserData');
         x.w=true;
         set(fig,'UserData',x);
     elseif strcmp(key,'s')
         x=get(fig,'UserData');
         x.s=true;
         set(fig,'UserData',x);
     elseif strcmp(key,'a')
         x=get(fig,'UserData');
         x.a=true;
         set(fig,'UserData',x);
     elseif strcmp(key,'h')
         x=get(fig,'UserData');
         x.h=true;
         set(fig,'UserData',x);
     elseif strcmp(key,'d')
         x=get(fig,'UserData');
         x.d=true;
         set(fig,'UserData',x);
     elseif strcmp(key,'f')
         x=get(fig,'UserData');
         x.f=true;
         set(fig,'UserData',x);
     end
end

function KeyRelease(varargin)
    %This function is the same as KeyPress, except it resets the command
    %when the user releases the up/down arrow.
     fig = varargin{1};
     key = varargin{2}.Key;
     if strcmp(key,'uparrow')
         x=get(fig,'UserData');
         x.up = false;
         set(fig,'UserData',x);
     elseif strcmp(key,'downarrow')
         x=get(fig,'UserData');
         x.down = false;
         set(fig,'UserData',x);
     elseif strcmp(key,'rightarrow')
         x=get(fig,'UserData');
         x.right = false;
         set(fig,'UserData',x);
     elseif strcmp(key,'leftarrow')
         x=get(fig,'UserData');
         x.left = false;
         set(fig,'UserData',x)
     elseif strcmp(key,'w')
         x=get(fig,'UserData');
         x.w=false;
         set(fig,'UserData',x);
     elseif strcmp(key,'s')
         x=get(fig,'UserData');
         x.s=false;
         set(fig,'UserData',x);
     elseif strcmp(key,'d')
         x=get(fig,'UserData');
         x.d=false;
         set(fig,'UserData',x);
     elseif strcmp(key,'a')
         x=get(fig,'UserData');
         x.a=false;
         set(fig,'UserData',x);
     elseif strcmp(key,'h')
         x=get(fig,'UserData');
         x.h=false;
         set(fig,'UserData',x);
     elseif strcmp(key,'f')
         x=get(fig,'UserData');
         x.f=false;
         set(fig,'UserData',x);
     end
end

end