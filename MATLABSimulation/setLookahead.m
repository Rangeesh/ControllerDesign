function [] = setLookahead( i, points )
% lookahead is a struct that contains the next 3 points, their indeces in
% the trajectory array, and the circle fit data for the 3 points.
global lookahead

lookahead.i = i; %3x1 vector containing indeces of next 3 points in trajectory array.
lookahead.points = points; %3x2 array of next 3 points' x and y values.
[xc, yc, r] = circleFit(points); %circle fit data for points.
lookahead.circleFit = [xc yc r];

end

