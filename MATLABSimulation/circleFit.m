function [xc,yc,R] = circleFit(preview_data)
%CircleFit takes at least 3 or more given waypoints
%and calculates the x position, y position, and radius
%of the circle that crosses each waypoint.
[m,n] = size(preview_data);
if (n ~=2)
    print('error');
else
X = preview_data(:,1); Y = preview_data(:,2);
Sx = sum(X); Sy = sum(Y);
Sxy= sum(X.*Y);
Sx2 = sum(X.*X); Sy2 = sum(Y.*Y);
 
Sx3 = sum(X.*X.*X); Sxy2= sum(X.*Y.*Y);
Sx2y = sum(X.*X.*Y); Sy3 = sum(Y.*Y.*Y);
 
a11 = 2*(Sx*Sx-m*Sx2); a12 = 2*(Sx*Sy-m*Sxy); a22 = 2*(Sy*Sy-m*Sy2);
A=[a11 a12; a12 a22]; 
 
b1 = Sx*(Sx2+Sy2)-m*(Sx3+Sxy2);
b2 = Sy*(Sx2+Sy2)-m*(Sx2y+Sy3);
b=[b1;b2];
 
        if -1*10^12<det(A) && det(A)<1*10^-12
            R = inf;
            xc = inf;
            yc = inf;
        else
            cntr=A\b;
            xc=cntr(1); yc=cntr(2);
            Xtilde = X-xc; Ytilde = Y-yc;
            R = sqrt((Xtilde'*Xtilde + Ytilde'*Ytilde)/m);

        end
end