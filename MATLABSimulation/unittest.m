% P = [ 1,7; 2,6; 5,8; 7,7; 9,5; 3,7];
P = [ 2,6; 5,8; 7,7; 9,5; 3,7];
New=newCircleFit(P);
Old=circleFit(P);
scatter(P(:,1),P(:,2), 'k','filled')
xlim([0,10])
ylim([0,10])
hold on;
theta=0:0.1:2*3.14;
scatter(New.xc+cos(theta)*New.R, New.yc+sin(theta)*New.R, '*b')
scatter(Old.xc+cos(theta)*Old.R, Old.yc+sin(theta)*Old.R, '*r')