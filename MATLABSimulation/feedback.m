function [dfb] = feedback(xcar,ycar,vx,theta_car,dtheta)
%This function is used to calculate feedback. Lateral Error uses the current position 
%of the car the circle's center and radius to calculate the lateral error.
%For yaw rate, it utilizes longitudinal velocity, radius and current yaw
%rate of car.
%For heading dif, we can use the theta from ode45 of the car and I'm not
%sure for the trajectory yet expcept for cheating right now

global lookahead direc      %Establish needed variables
global gains

%  Heading Error
x1 = lookahead.points(1,1);
x2 = lookahead.points(2,1);
y1 = lookahead.points(1,2);
y2 = lookahead.points(2,2);

dx = x2 - x1;
dy = y2 - y1;

for i=1:length(gains)
    if vx <= gains(i,1)
        klat = gains(i,2);
        kyaw = gains(i,3);
        kyawrate = gains(i,4);
        break
    end
end

%  Lateral Error
xcir = lookahead.circleFit.xc;      %circle's center x position 
ycir = lookahead.circleFit.yc;      %circle's center y position 
r = lookahead.circleFit.R;         %circle's radius

if(r==inf)  %this means the 3 points are collinear
    if dy == 0 && dx == 0
        error('Point 1 and Point 2 are the same. Found in feedback') 
    elseif dx == 0      %if x2 = x1 only therefore path going north or south
        elat = xcar - x1;
        LoRvectors = [xcar-x1,0; cos(theta_car),sin(theta_car)];
        LoR = sign(det(LoRvectors));
        klat = -klat*LoR;
    else    %need to figure out the sign
        m = dy/dx;    a = [1,m];  b = [xcar,ycar-(y1-m*x1)];
        ab = [a;b];
        klat = -klat;
        if dx > 0   %going east, northeast or southeast
            elat = det(ab)/norm(a);
        else        %going west, northwest or southwest
            elat = -det(ab)/norm(a);
        end
    end      
else
    elat = sqrt((xcar-xcir)^2+(ycar-ycir)^2)-r;
    LoRvectors = [xcar-xcir,ycar-ycir; cos(theta_car),sin(theta_car)];
    LoR = sign(det(LoRvectors));
    klat = klat*LoR;
end

%  Car's Yaw Rate - v/r Error
eyawrate = (dtheta-direc*vx/r);

%  Heading Error
% calculate the closest point to the circle
magnitude = sqrt((xcar-xcir)^2 + (ycar-ycir)^2);
Cx = xcir + (r*(xcar-xcir)/magnitude);
Cy = ycir + (r*(ycar-ycir)/magnitude);
q1 = [lookahead.points(1,1),lookahead.points(1,2)]; %point A
q2 = [lookahead.points(2,1),lookahead.points(2,2)]; %point B
q3 = [lookahead.points(3,1),lookahead.points(3,2)]; %point C
AB = q1-q2;
BC = q2-q3;

abbc = [AB;BC];
turning = sign(det(abbc));
bound = 0.05;

if (turning == 0) %when straight
    q1 = [x2-x1,y2-y1];
    bob = atan2d(q1(2),q1(1));
else    %when there is a circle
%     if (ycar-ycir) <=bound && (ycar-ycir) >=-bound && turning <0 %%if the point is at the left or right of the circle
%         bob = -90;%the slope is infinite so 90 degrees 
%         % i need to change this to be 90 or -90 if going up or down
%     elseif(ycar-ycir) <=bound && (ycar-ycir) >=-bound && turning >0 
%         bob = 90;
    if turning<0    %%if not at left or right
        q1 = [xcir-xcar,ycar-ycir];
        bob = atan2d(q1(1),q1(2));
    elseif turning >0
        q1 = [xcir-xcar,ycar-ycir];
        bob = atan2d(q1(1),q1(2))-180;
        bob2 = atan2d(y2-y1,x2-x1);
    end
end

theta_road = bob*pi/180;

eyaw = -(theta_car-theta_road);
theta_wrap = mod(eyaw+pi,2*pi);

if theta_wrap<0
    theta_wrap = theta_wrap+2*pi;
end

theta_wrap = theta_wrap - pi;
eyaw = theta_wrap;
dfb = klat*elat + kyawrate*eyawrate + kyaw*eyaw;
end