close all
clear variables
clc
% Don't go reversetrack. 
% Will have to change x-axis of graphs for oval track
%% Simulation Parameters
% Simulation Parameters
waypointspacing = 1; %spacing between waypoints measured along the path in m.
preview         = 0.5; %Time Preview of Vehicle 
vx              = 5; % m/s. Constant longitudinal speed of the car for the simulation.
totaltime       = 50; %Time to run the simulation for.

%Animation Options
axismoves       = true; %Set it to true to have the position vs. time zoom in on the car and move with it. False to display the entire graph zoomed out without motion.
axiswidth       = 5; %dimensions of the zoomed-in graph window, if axismoves is set to true.
simulationspeed = 2; %Simulation speed in multiples of real time. i.e. 2 is twice real time, 1 is real time, etc.
drawrealtime    = false; %if true, the green line moves at the speed of the vehicle. If not, the whole graph draws instantly.
drawwheels      = true;
calculateerror  = false; %Calculates distances from wheels to lane. Computationally expensive, so it can be toggled to save time.
displayoverview = true; %Displays an overview of the entire course above the simulation window. Can be set to false to save time on simulation.
writegif        = false; %drawrealtime must be enabled to record gif.
reversetrack    = false;
%% Definitions

global trajectory
global gains
global ffangle fbangle steerangle
global radius 
global lateralerror

gains = csvread('gains.csv');
ffangle = [];
fbangle = [];
steerangle = [];
radius=[];
lateralerror=[];


car = defineCar();

points = csvread('track.csv');

if reversetrack
    temppoints = zeros(size(points));
    for i=1:length(points)
        temppoints(length(points)-i+1,:) = points(i,:);
    end
    points = temppoints;
end
%% Creating Splines
lane1 = points(:,1:2);
lane2 = points(:,3:4);
center = points(:,5:6);

f1 = fit(lane1(:,1),lane1(:,2),'smoothingspline');
f2 = fit(lane2(:,1),lane2(:,2),'smoothingspline');
f3 = fit(center(:,1),center(:,2),'smoothingspline'); % Baseline

x1 = linspace(lane1(1,1),lane1(length(lane1),1),1000);
y1 = f1(x1);
x2 = linspace(lane2(1,1),lane2(length(lane2),1),1000);
y2 = f2(x2);
x3 = linspace(center(1,1),center(length(center),1),1000);
y3 = f3(x3);

xsample = [center(1,1)];
%% Collects Way Points
j = 1;
arc_length = 0;
arcXinc = .02;

if reversetrack
    while xsample(j)>42.2858
    xinc = 0;
    arc_length = 0;
    while arc_length < waypointspacing
        %continues until arc length is at the goal value.
        slope = (f3(xsample(j)+xinc)-f3(xsample(j)+xinc+arcXinc))/arcXinc;
        arc_length = arc_length + sqrt(arcXinc^2+(slope*arcXinc)^2);
        xinc = xinc + arcXinc;
    end    
    xsample(j+1) = xsample(j)-xinc;
    j = j+1;
    end
else
    while xsample(j)<285
        xinc = 0;
        arc_length = 0;
        while arc_length < waypointspacing
            %continues until arc length is at the goal value.
            slope = (f3(xsample(j)+xinc)-f3(xsample(j)+xinc+arcXinc))/arcXinc;
            arc_length = arc_length + sqrt(arcXinc^2+(slope*arcXinc)^2);
            xinc = xinc + arcXinc;
        end
        xsample(j+1) = xsample(j)+xinc;
        j = j+1;
    end
end

points = [xsample' f3(xsample)];
%% Lookahead
trajectory = points(:,1:2); % Fixed


% setLookahead([1;2;3], trajectory(1:3,:)); % For preview, that three will change.
newUpdatePoints(points(1,1),points(1,2), vx, preview);

%% Solving Differential Equation
step = 0.1;
if reversetrack
    options = odeset('MaxStep',.05);
else
    options = odeset('MaxStep',.05, 'Events', @carevents);
end

fprintf('Solving Differential Equation...\n')
if reversetrack
    [t_car,q_car] = ode45(@(t,q) bicycleModel(t,q,car,vx, preview), 0:step:totaltime, [points(1,1),points(1,2),pi/2+pi/4-.3,0,0], options);
else
    [t_car,q_car] = ode45(@(t,q) bicycleModel(t,q,car,vx, preview), 0:step:totaltime, [points(1,1),points(1,2),-.1,0,0], options);
end
fprintf('Differential Equation Solved!\n')
%% Plotting
%Map the t values of steerangle and fbangle to the t values of t_car (the results of
%the ode solver)
steeringanglenorm = zeros(length(t_car),1);
fbanglenorm = zeros(length(t_car),1);
ffanglenorm = zeros(length(t_car),1);
radiusnorm = zeros(length(t_car),1);
lateralerrornorm = zeros(length(t_car),1);
for i=1:length(t_car)
    [c steerind] = min(abs(steerangle(:,1)-t_car(i))); % Takes the closest time
    steeranglenorm(i) = steerangle(steerind,2);
    fbanglenorm(i) = fbangle(steerind,2);
    ffanglenorm(i) = ffangle(steerind,2);
    radiusnorm(i) = radius(steerind);
    lateralerrornorm(i) = lateralerror(steerind);
end

trackwidth = 1.7145-8*0.0254;
carlength = 102.5*0.0254;
wheelwidth = 12;
wheeld = 25*0.0254;

frwpos = [q_car(:,1) + trackwidth*sin(q_car(:,3))/2 + car.a*cos(q_car(:,3)),...
            q_car(:,2) - trackwidth*cos(q_car(:,3))/2 + car.a*sin(q_car(:,3))];
flwpos = [q_car(:,1) - trackwidth*sin(q_car(:,3))/2 + car.a*cos(q_car(:,3)),...
            q_car(:,2) + trackwidth*cos(q_car(:,3))/2 + car.a*sin(q_car(:,3))];
rrwpos = [q_car(:,1) + trackwidth*sin(q_car(:,3))/2 - car.b*cos(q_car(:,3)),...
        q_car(:,2) - trackwidth*cos(q_car(:,3))/2 - car.b*sin(q_car(:,3))];
rlwpos = [q_car(:,1) - trackwidth*sin(q_car(:,3))/2 - car.b*cos(q_car(:,3)),...
        q_car(:,2) + trackwidth*cos(q_car(:,3))/2 - car.b*sin(q_car(:,3))];
frontcentpos = [(frwpos(:,1)+flwpos(:,1))/2, (frwpos(:,2)+flwpos(:,2))/2];

f1 = figure('Name','Simulation','NumberTitle','off',...
    'units','normalized','outerposition',[0 0 1 1]);

if (displayoverview)&&(drawrealtime)
    subplot(4,6,[7 8 9 10 13 14 15 16 19 20 21 22]);
else
    subplot(4,6,[1 2 3 4 7 8 9 10 13 14 15 16 19 20 21 22]);
end

hold on
plot(x1,y1)
plot(x2,y2)
plot(x3,y3)
axis equal
xlabel('X Position (m)')
ylabel('Y Position (m)')

scatter(xsample',f3(xsample),[10])

% Draw initial locations of wheels and car edges
if (drawwheels)&&(drawrealtime)
    frontedge = plot([frwpos(1,1) flwpos(1,1)], [frwpos(1,2) flwpos(1,2)], 'r--');
    backedge = plot([rrwpos(1,1) rlwpos(1,1)], [rrwpos(1,2) rlwpos(1,2)], 'r--');
    leftedge = plot([flwpos(1,1) rlwpos(1,1)], [flwpos(1,2) rlwpos(1,2)], 'r--');
    rightedge = plot([frwpos(1,1) rrwpos(1,1)], [frwpos(1,2) rrwpos(1,2)], 'r--');
    
    frontrightwheelline = plot([frwpos(1,1)-wheeld*cos(q_car(1,3)+steeranglenorm(1))/2, frwpos(1,1)+wheeld*cos(q_car(1,3)+steeranglenorm(1))/2],...
            [frwpos(1,2)-wheeld*sin(q_car(1,3)+steeranglenorm(1))/2, frwpos(1,2)+wheeld*sin(q_car(1,3)+steeranglenorm(1))/2],...
            'LineWidth', wheelwidth, 'Color', 'k');
    frontleftwheelline = plot([flwpos(1,1)-wheeld*cos(q_car(1,3)+steeranglenorm(1))/2, flwpos(1,1)+wheeld*cos(q_car(1,3)+steeranglenorm(1))/2],...
        [flwpos(1,2)-wheeld*sin(q_car(1,3)+steeranglenorm(1))/2, flwpos(1,2)+wheeld*sin(q_car(1,3)+steeranglenorm(1))/2],...
        'LineWidth', wheelwidth, 'Color', 'k');
    rearrightwheelline = plot([rrwpos(1,1)-wheeld*cos(q_car(1,3))/2, rrwpos(1,1)+wheeld*cos(q_car(1,3))/2],...
            [rrwpos(1,2)-wheeld*sin(q_car(1,3))/2, rrwpos(1,2)+wheeld*sin(q_car(1,3))/2],...
            'LineWidth', wheelwidth, 'Color', 'k');
    rearleftwheelline = plot([rlwpos(1,1)-wheeld*cos(q_car(1,3))/2, rlwpos(1,1)+wheeld*cos(q_car(1,3)/2)],...
        [rlwpos(1,2)-wheeld*sin(q_car(1,3))/2, rlwpos(1,2)+wheeld*sin(q_car(1,3)/2)],...
        'LineWidth', wheelwidth, 'Color', 'k');
end

error = zeros(length(t_car),1);
n = 1;
filename = 'Animation.gif';

if displayoverview
    subplot(4,6,[1 2 3 4])
    title('Course Overview')
    axis equal
    ylim([min(y3)-5 max(y3)+5]);
    xlabel('X Position (m)')
    ylabel('Y Position (m)')
end

subplot(4,6,[5 6]);
ylim([min(lateralerrornorm),max(lateralerrornorm)]);
if reversetrack
    xlim([q_car(length(q_car),1),q_car(1,1)]);
else
    xlim([q_car(1,1),q_car(length(q_car),1)]);
end
title('Lateral Error')
xlabel('X Position (m)')
ylabel('Distance (m)')

subplot(4,6,[11 12]);
ylim([min(radiusnorm),max(radiusnorm)]);
if reversetrack
    xlim([q_car(length(q_car),1),q_car(1,1)]);
else
    xlim([q_car(1,1),q_car(length(q_car),1)]);
end
title('Computed Radius of Curvature')
xlabel('X Position (m)')
ylabel('Radius (m)')

% subplot(4,6,[11 12]);
% ylim([-34*pi/180,34*pi/180]);
% if reversetrack
%     xlim([q_car(length(q_car),1),q_car(1,1)]);
% else
%     xlim([q_car(1,1),q_car(length(q_car),1)]);
% end
% title('Requested Total Steer Angle')
% xlabel('X Position (m)')
% ylabel('Steer Angle (rad)')

subplot(4,6,[17 18]);
ylim([-34*pi/180,34*pi/180]);
if reversetrack
    xlim([q_car(length(q_car),1),q_car(1,1)]);
else
    xlim([q_car(1,1),q_car(length(q_car),1)]);
end
title('Requested Feedforward Angle')
xlabel('X Position (m)')
ylabel('Feedforward Angle (rad)')

subplot(4,6,[23 24]);
ylim([-34*pi/180,34*pi/180]);
if reversetrack
    xlim([q_car(length(q_car),1),q_car(1,1)]);
else
    xlim([q_car(1,1),q_car(length(q_car),1)]);
end
title('Requested Feedback Angle')
xlabel('X Position (m)')
ylabel('Feedback Angle (rad)')
        
if drawrealtime
    for i = 1:length(t_car)
        if displayoverview
            subplot(4,6,[7 8 9 10 13 14 15 16 20 21 22]);
        else
            subplot(4,6,[1 2 3 4 7 8 9 10 13 14 15 16 19 20 21 22]);
        end

        hold on
        if drawwheels      
            delete(frontedge)
            delete(backedge)
            delete(leftedge)
            delete(rightedge)

            delete(frontrightwheelline)
            delete(frontleftwheelline)
            delete(rearrightwheelline)
            delete(rearleftwheelline)
        end

        a = scatter(q_car(i,1),q_car(i,2),[200],'g.');

        if (drawwheels)&&(drawrealtime)
            frontrightwheelline = plot([frwpos(i,1)-wheeld*cos(q_car(i,3)+steeranglenorm(i))/2, frwpos(i,1)+wheeld*cos(q_car(i,3)+steeranglenorm(i))/2],...
                [frwpos(i,2)-wheeld*sin(q_car(i,3)+steeranglenorm(i))/2, frwpos(i,2)+wheeld*sin(q_car(i,3)+steeranglenorm(i))/2],...
                'LineWidth', wheelwidth, 'Color', 'k');
            frontleftwheelline = plot([flwpos(i,1)-wheeld*cos(q_car(i,3)+steeranglenorm(i))/2, flwpos(i,1)+wheeld*cos(q_car(i,3)+steeranglenorm(i))/2],...
                [flwpos(i,2)-wheeld*sin(q_car(i,3)+steeranglenorm(i))/2, flwpos(i,2)+wheeld*sin(q_car(i,3)+steeranglenorm(i))/2],...
                'LineWidth', wheelwidth, 'Color', 'k');
            rearrightwheelline = plot([rrwpos(i,1)-wheeld*cos(q_car(i,3))/2, rrwpos(i,1)+wheeld*cos(q_car(i,3))/2],...
                [rrwpos(i,2)-wheeld*sin(q_car(i,3))/2, rrwpos(i,2)+wheeld*sin(q_car(i,3))/2],...
                'LineWidth', wheelwidth, 'Color', 'k');
            rearleftwheelline = plot([rlwpos(i,1)-wheeld*cos(q_car(i,3))/2, rlwpos(i,1)+wheeld*cos(q_car(i,3))/2],...
                [rlwpos(i,2)-wheeld*sin(q_car(i,3))/2, rlwpos(i,2)+wheeld*sin(q_car(i,3))/2],...
                'LineWidth', wheelwidth, 'Color', 'k');

            frontedge = plot([frwpos(i,1) flwpos(i,1)], [frwpos(i,2) flwpos(i,2)], 'r--');
            backedge = plot([rrwpos(i,1) rlwpos(i,1)], [rrwpos(i,2) rlwpos(i,2)], 'r--');
            leftedge = plot([flwpos(i,1) rlwpos(i,1)], [flwpos(i,2) rlwpos(i,2)], 'r--');
            rightedge = plot([frwpos(i,1) rrwpos(i,1)], [frwpos(i,2) rrwpos(i,2)], 'r--');
        end

        if drawrealtime
            if i>2
                pause((t_car(i)-t_car(i-1))/simulationspeed)
            end
        end

        widthmodifier = 1.8;
        if axismoves
            xlim([q_car(i,1)-widthmodifier*axiswidth q_car(i,1)+widthmodifier*axiswidth])
            ylim([q_car(i,2)-axiswidth q_car(i,2)+axiswidth])
        end

        if displayoverview
            subplot(4,6,[1 2 3 4])
            hold on
            plot(x3,y3,'r')
            a = scatter(q_car(i,1),q_car(i,2),[200],'g.');
        end

        if calculateerror
            Elane1 = min([sqrt((x1-frwpos(i,1)).^2 + (y1'-frwpos(i,2)).^2),...
                sqrt((x1-flwpos(i,1)).^2 + (y1'-flwpos(i,2)).^2),...
                sqrt((x1-rrwpos(i,1)).^2 + (y1'-rrwpos(i,2)).^2),...
                sqrt((x1-rlwpos(i,1)).^2 + (y1'-rlwpos(i,2)).^2)]);
            Elane2 = min([sqrt((x2-frwpos(i,1)).^2 + (y2'-frwpos(i,2)).^2),...
                sqrt((x2-flwpos(i,1)).^2 + (y2'-flwpos(i,2)).^2),...
                sqrt((x2-rrwpos(i,1)).^2 + (y2'-rrwpos(i,2)).^2),...
                sqrt((x2-rlwpos(i,1)).^2 + (y2'-rlwpos(i,2)).^2)]);
            Etotal = min([Elane1 Elane2]);
        else
            Etotal = 0;
        end
        error(i) = Etotal;
        
        
        subplot(4,6,[5 6]);
        hold on
        if i>1
            plot([q_car(i-1,1), q_car(i,1)] ,[lateralerrornorm(i-1), lateralerrornorm(i)],'b');
        end
        
        subplot(4,6,[11 12]);
        hold on
        if i>1
            plot([q_car(i-1,1), q_car(i,1)] ,[radiusnorm(i-1), radiusnorm(i)],'b')
        end
        
        subplot(4,6,[17 18]);
        hold on
        if i>1
            plot([q_car(i-1,1), q_car(i,1)],[ffanglenorm(i-1), ffanglenorm(i)],'b')
        end
        
        subplot(4,6,[23 24]);
        hold on
        if i>1
            plot([q_car(i-1,1), q_car(i,1)],[fbanglenorm(i-1),fbanglenorm(i)],'b')
        end
        
        if writegif
            frame = getframe(f1);
            im = frame2im(frame);
            [imind,cm] = rgb2ind(im,256);
            if n == 1
                imwrite(imind,cm,filename,'gif', 'DelayTime',0.1, 'Loopcount',0);
            else
                imwrite(imind,cm,filename,'gif','DelayTime',0.1, 'WriteMode','append');
            end
            n = n+1;
        end
    end
else
    subplot(4,6,[1 2 3 4 7 8 9 10 13 14 15 16 19 20 21 22]);
    a = scatter(q_car(:,1),q_car(:,2),[200],'g.');
    
    hold on
    plot(x1,y1)
    plot(x2,y2)
    plot(x3,y3)
    axis equal
    xlabel('X Position (m)')
    ylabel('Y Position (m)')
    scatter(xsample',f3(xsample),[10])
    
    for i=1:length(t_car)
        if calculateerror
            Elane1 = min([sqrt((x1-frwpos(i,1)).^2 + (y1'-frwpos(i,2)).^2),...
                sqrt((x1-flwpos(i,1)).^2 + (y1'-flwpos(i,2)).^2),...
                sqrt((x1-rrwpos(i,1)).^2 + (y1'-rrwpos(i,2)).^2),...
                sqrt((x1-rlwpos(i,1)).^2 + (y1'-rlwpos(i,2)).^2)]);
            Elane2 = min([sqrt((x2-frwpos(i,1)).^2 + (y2'-frwpos(i,2)).^2),...
                sqrt((x2-flwpos(i,1)).^2 + (y2'-flwpos(i,2)).^2),...
                sqrt((x2-rrwpos(i,1)).^2 + (y2'-rrwpos(i,2)).^2),...
                sqrt((x2-rlwpos(i,1)).^2 + (y2'-rlwpos(i,2)).^2)]);
            Etotal = min([Elane1 Elane2]);
        else
            Etotal = 0;
        end
        error(i) = Etotal;
    end
    
    subplot(4,6,[5 6]);
    hold on
    plot(q_car(:,1),lateralerrornorm(:),'b');
    ylim([min(lateralerrornorm),max(lateralerrornorm)]);
    if reversetrack
        xlim([q_car(length(q_car),1),q_car(1,1)]);
    else
        xlim([q_car(1,1),q_car(length(q_car),1)]);
    end
    title('Lateral Error')
    xlabel('X Position (m)')
    ylabel('Distance (m)')
    
    subplot(4,6,[11 12]);
    hold on
    plot(q_car(:,1),radiusnorm(:),'b')
    ylim([min(radiusnorm),max(radiusnorm)]);
    if reversetrack
        xlim([q_car(length(q_car),1),q_car(1,1)]);
    else
        xlim([q_car(1,1),q_car(length(q_car),1)]);
    end
    title('Radius of Curvature')
    xlabel('X Position (m)')
    ylabel('Radius (m)')
    
    subplot(4,6,[17 18]);
    hold on
    plot(q_car(:,1),ffanglenorm(:),'b')
    ylim([-34*pi/180,34*pi/180]);
    if reversetrack
        xlim([q_car(length(q_car),1),q_car(1,1)]);
    else
        xlim([q_car(1,1),q_car(length(q_car),1)]);
    end
    title('Requested Feedforward Angle')
    xlabel('X Position (m)')
    ylabel('Feedforward Angle (rad)')
    
    subplot(4,6,[23 24]);
    hold on
    plot(q_car(:,1),fbanglenorm(:),'b')
    ylim([-34*pi/180,34*pi/180]);
    if reversetrack
        xlim([q_car(length(q_car),1),q_car(1,1)]);
    else
        xlim([q_car(1,1),q_car(length(q_car),1)]);
    end
    title('Requested Feedback Angle')
    xlabel('X Position (m)')
    ylabel('Feedback Angle (rad)')
end

average_distance_from_edge = mean(error)
min_distance_from_edge = min(error)