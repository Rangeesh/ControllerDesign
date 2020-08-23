function [lookahead] = newSetLookahead(row_index,vx, preview)
% Unit Test Passed
% row_index := Starting preview index
global trajectory lookahead
% TODO: if row_index is in the end (Not in last 3, exit?), exit.

row_i = row_index;
preview_dist = vx*preview;
accumulated_dist=0;

while row_i<size(trajectory,1) % Since you have +1
    accumulated_dist = accumulated_dist + distance_2points(trajectory(row_i,1),trajectory(row_i,2),trajectory(row_i+1,1),trajectory(row_i+1,2));
    if (accumulated_dist >=preview_dist)
        break;
    end
    row_i=row_i+1;
end
% Output: row_index to row_i is the new preview section.

if accumulated_dist<preview_dist
    % TODO: Exit with error string
%     exit;
end



lookahead.i= row_index:row_i;
lookahead.points = trajectory(row_index:row_i,:);
circle=circleFit(lookahead.points); % [xc, yc, r] inf if straight line.
lookahead.circleFit=circle;


end