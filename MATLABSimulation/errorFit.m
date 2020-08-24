function error = errorFit(preview,xc,yc,r)
% using AGE metric - Absolute Geometric Error
    error=0;
    for i=1:size(preview,1)
        error = error + abs(sqrt((preview(i,1)-xc)^2+(preview(i,2)-yc)^2) - r);
    end
end


