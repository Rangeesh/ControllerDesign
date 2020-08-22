function [ ] = updatePoints(xcar, ycar)
% This function compares the car's current x and y to the current
% lookahead data, and updates the lookahead data if the car has past the
% second waypoint in lookahead.points.

global lookahead trajectory

% Define x1,x2, etc. as coordinates of first 2 points in lookahead data.
x1 = lookahead.points(1,1);
x2 = lookahead.points(2,1);
y1 = lookahead.points(1,2);
y2 = lookahead.points(2,2);

% Get delta y and x
dy = y2-y1;
dx = x2-x1;

    function [] = update()
        % Updates lookahead data to the next 3 points.
        if lookahead.i(3)<length(trajectory)
            setLookahead(lookahead.i+1, trajectory(lookahead.i+1,:));
        end
    end

%this draws a perpendicular line on the second point relative to the line
%from point 1 to point 2. STILL NEED TO CONSIDER IF THE FIRST AND SECOND
%POINT ARE THE SAME FOR SAFETY CONCERNS

if dy == 0 && dx == 0   %this means point 1 and point 2 are the same
        error('Point 1 and Point 2 are the same. Found in updatePoints') 
elseif dx==0
    if dy>0 %car is moving straight up
        if ycar>y2
            update()
        end
    elseif dy<0 %car is moving straight down
        if ycar<y1
            update()
        end
    else %x2==x1, y2==y1: same waypoint passed twice
        update()
    end
elseif dy==0
    if dx>0 %car is moving straight right
        if xcar>x2
            update()
        end
    elseif dx<0 %car is moving straight left
        if xcar<x1
            update()
        end
    end
else
    m = dy/dx;
    m = -1/m;
    if dx>0 %car is travelling to the right
        if dy>0 %car is travelling up and to the right
            if ycar>m*(xcar-x2)+y2
                update()
            end
        elseif dy<0 %car is travelling down and to the right
                if ycar<m*(xcar-x2)+y2
                    update()
                end
        end
    elseif dx<0 %car is travelling to the left
        if dy>0 %car is travelling to the left and up
            if ycar>m*(xcar-x2)+y2
                update()
            end
        elseif dy<0 %car is travelling down and to the left
            if ycar<m*(xcar-x2)+y2
                update()
            end
        end
    end
end

end

