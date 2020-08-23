function [lookahead_local] = newUpdatePoints(xcar,ycar,vx, preview)
% This function updates the current lookahead data.

global lookahead trajectory radius
% trajectory - all waypoints available


%% Identify the closest point in the trajectory to the car position
% Output is row index in trajectory.

% Initialization
row_index = 1;
min_dist = distance_2points(xcar,ycar,trajectory(1,1),trajectory(1,2));
dist=0;

% For Loop
for i=1:size(trajectory,1)
    dist=distance_2points(xcar,ycar,trajectory(i,1),trajectory(i,2));
    if (min_dist>dist)
        min_dist=dist;
        row_index=i;
    end
end
% Output: starting row_index of your preview.

lookahead_local=newSetLookahead(row_index,vx, preview);
if lookahead_local.circleFit.R<1000
    radius = [radius, lookahead_local.circleFit.R];
else
    radius = [radius, 1000];
end

% Pass this as parameter to newSetLookahead
% It should compute the required points. And it's xc,yc,R.


end