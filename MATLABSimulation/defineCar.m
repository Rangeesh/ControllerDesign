function [ car ] = defineCar( )
%DEFINECAR Defines physical and inertial parameters of the car.

%1949 Buick Given Data
car.m = 1605; %kg Mass
car.I = 2045; %kgm^2 Yaw Inertia
car.a = 1.135; %m CG to front axle
car.b = 1.465; %m CG to rear axle
car.Cf = 77850; %N/rad Front Cornering Stiffness
car.Cr = 76510; %N/rad Rear Cornering Stiffness

car.Wf = car.m*car.b*9.81/(car.a+car.b); %calculated through statics
car.Wr = car.m*car.a*9.81/(car.a+car.b);

car.ksg = car.Wf/car.Cf - car.Wr/car.Cr;
end

