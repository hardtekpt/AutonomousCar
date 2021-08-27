function map =obsInsert(t,map,eventList)
for i = 1:size(eventList,2)
    if eventList(3,i) == -5
        if t >= eventList(4,i) && t < eventList(4,i)+eventList(5,i)
                map(eventList(2,i),eventList(1,i)) = -i-1;
        else 
                map(eventList(2,i),eventList(1,i)) = 0;
        end
    end
end
end