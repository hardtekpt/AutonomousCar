function [index,min] = findnpoint(xx,yy,x,y,ahead,prevIndex)
    %Finding the closest point in the path to the present point in the
    %trajectory.
    min = inf;
    maxIndex=prevIndex+ahead;
    if prevIndex+ahead > size(xx,2)
        maxIndex = size(xx,2);
    end
    
    for i = prevIndex:maxIndex
        dist = sqrt((xx(i)-x(end))^2+(yy(i)-y(end))^2);
        if dist < min
            min = dist; 
            index = i;
        end
    end
    if ahead ~= size(xx,2)
        if (index+ahead > size(xx,2))
            index = size(xx,2)-ahead;
        end
    end
end