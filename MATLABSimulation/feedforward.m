function [dff] = feedforward(car,vx)
%For feedforward, dff = L/R + Ksg*(vx^2/g*R) is used to calculate
%the steering angle. The direction is calculated by using
%vectors. The first is from the first point to the second.
%The second is from the second point to the third. Then
%the sign of the cross product calculated is the direction.

global lookahead direc

% Direction Calculation
q1 = [lookahead.points(1,1),lookahead.points(1,2)]; %point A
q2 = [lookahead.points(2,1),lookahead.points(2,2)]; %point B
q3 = [lookahead.points(3,1),lookahead.points(3,2)]; %point C
AB = q1-q2;
BC = q2-q3;

abbc = [AB;BC];
turning = det(abbc);
direc = sign(turning);
r = lookahead.circleFit(3);
   
    %heading and output of the code.
dff = direc*((car.a+car.b)/r + car.ksg*vx^2/(9.81*r));
    
end