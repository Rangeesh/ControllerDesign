function circle = newCircleFit(preview_data)
%CircleFit takes at least 3 or more given waypoints
%and calculates the x position, y position, and radius
%of the circle that crosses each waypoint.
%% Unit Test is there -- unittest.m -- OK I guess
%% Getting a good initial guess for the iterative robust fitting of circle arc
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
% R, xc, yc are the initial guess
disp("Solution");
disp(R);
disp(xc);
disp(yc);
%% Assigning Parameters -- Currently testing with mean of preview data.
accuracy = 0.001;
lambda=1;
xc_old=xc;
% xc_old = mean(preview_data(:,1));
yc_old=yc;
% yc_old = mean(preview_data(:,2));
disp("Initial Guess");
disp(xc_old);
disp(yc_old);
iter = 0;
itermax=100000;
while true
    r_set=[];
    cos_set=[];
    sin_set=[];
    % Finding Radii, Sin and Cos sets.
    for i=1:size(preview_data,1)
        rad_i = sqrt((preview_data(i,1)-xc_old)^2+(preview_data(i,2)-yc_old)^2);
        r_set = [r_set; rad_i];
        cos_i = (preview_data(i,1)-xc_old)/rad_i;
        cos_set= [cos_set;cos_i];
        sin_i = (preview_data(i,2)-yc_old)/rad_i;
        sin_set=[sin_set;sin_i];
    end
    % Calculating Radius
    median_r = median(r_set);
    % Absolute Geometric Error
    error_old = errorFit(preview_data,xc_old,yc_old,median_r);
    I=[];
    O=[];
    C=[];
    Eaplus=0;
    Eaminus=0;
    Ebplus=0;
    Ebminus=0;
    % identifying the I,O,C sets and the Ea,Eb +- values.
    for i=1:size(preview_data,1)
        if r_set(i)<median_r
            I=[I;i];
            Eaplus = Eaplus + cos_set(i);
            Eaminus = Eaminus + cos_set(i);
            Ebplus = Ebplus + sin_set(i);
            Ebminus = Ebminus + sin_set(i);
        elseif r_set(i)>median_r
            O=[O;i];
            Eaplus = Eaplus - cos_set(i);
            Eaminus = Eaminus - cos_set(i);
            Ebplus = Ebplus - sin_set(i);
            Ebminus = Ebminus - sin_set(i);
        else
            C=[C;i];
            Eaplus = Eaplus + abs(cos_set(i));
            Eaminus = Eaminus - abs(cos_set(i));
            Ebplus = Ebplus + abs(sin_set(i));
            Ebminus = Ebminus - abs(sin_set(i));
        end
    end
    
    % Identifying alpha, beta
    % Identifying d1, d2
    
    alpha = 0.5;
    beta = 0.5;
    if Eaminus >=0
        alpha = 1;
    elseif Eaplus<=0
        alpha = 0;
    end
    
    if Ebminus>=0
        beta=1;
    elseif Ebplus<=0
        beta=0;
    end
    d1 = -(alpha*Eaminus+(1-alpha)*Eaplus);
    d2 = -(beta*Ebminus+(1-beta)*Ebplus);
    
    % Identifying xc_new and yc_new
    % identifying the new error
    xc_new = xc_old + lambda*d1;
    yc_new = yc_old + lambda*d2;
    error_new = errorFit(preview_data,xc_new,yc_new,median_r); % TODO
    
    % Update Step
    if error_new < error_old 
        lambda = 1.1* lambda;
        xc_old = xc_new;
        yc_old = yc_new;
    else
        lambda = 0.9*lambda;
    end
    
    % Ending Loop Requirements.
    if lambda < accuracy
        disp("Lambda valued reached");
        disp(iter);
        break;
    end
    iter=iter+1;
    if iter>itermax
        disp("Iteration Max Reached");
        disp(lambda);
        break;
    end
end

circle.xc = xc_old;
circle.yc = yc_old;
circle.R = median_r;

end