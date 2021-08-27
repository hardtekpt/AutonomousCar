function map = eventMapping(eventList,xx,yy,scale,map,ahead)
    for i=1:size(eventList, 2)
        if eventList(3,i) == -7
            [index,~] = findnpoint(xx,yy,round(eventList(1,i)/scale),round(eventList(2,i)/scale),size(xx,2),1);
            pos_x = round(yy(index)*0.4);
            pos_y = round(xx(index)*0.4);
        else
            pos_x = eventList(2,i);
            pos_y = eventList(1,i);
        end
        map(pos_x, pos_y) = -i-1;
    end
end
