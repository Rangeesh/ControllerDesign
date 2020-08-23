function [ dz ] = bicycleModel( t, q, car, vx, preview )
global df_points ffangle fbangle steerangle
Cf = car.Cf; %redefine car inertial parameters for ease of use.
Cr = car.Cr;
a = car.a;
b = car.b;
m = car.m;
I = car.I;
       
x = q(1); %define state variables for ODE.
y = q(2);
theta = q(3);
vy = q(4);
w = q(5);

%get required steer angle from subroutine
% updatePoints(x,y);
newUpdatePoints(x,y,vx,preview);
dff = feedforward(car,vx);
dfb = feedback(x,y,vx,theta,w);
df = dff+dfb;

if df >=34*pi/180
    df = 34*pi/180;
elseif df <=-34*pi/180
    df = -34*pi/180;
else
    df = df;
end

ffangle = [ffangle; [t dff]];
fbangle = [fbangle; [t dfb]];
steerangle = [steerangle; [t df]];

 %Solve equations of motion.
dxdt = vx*cos(theta)-vy*sin(theta);
dydt = vx*sin(theta)+vy*cos(theta);
dthetadt = w;
dvydt = -((Cf+Cr)*vy/(m*vx)) - ((vx + (a*Cf-b*Cr)/(m*vx))*w) + Cf*df/m;
dwdt = (-(a*Cf-b*Cr)*vy/(I*vx)) - (a^2*Cf+b^2*Cr)*w/(I*vx) + a*Cf*df/I;

dz = [dxdt;dydt;dthetadt;dvydt;dwdt];

end