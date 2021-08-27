function [next_obs,theta_car] = directedSensor(crosswalk,theta_car,map,x,y)
    N = 25;
    senses = lidar(map,x(end),y(end),N);
    aux = find(senses < -1); event = zeros(3,1);
    for i = 1:length(aux)
        [a,b]=ind2sub([N,N],[aux(i)]);
        event(1:2,i) = [b;a];
        event(1,i) = x(end)-(floor(N/2)-event(1,i))-1;
        event(2,i) = y(end)-(floor(N/2)-event(2,i))-1;
        event(3,i) = senses(a,b);
    end
    if crosswalk == 0
    theta_car = [y(end)-y(1), x(end)-x(1)];
    end
    next_obs = [];
    prevdist = N+1;
    for i = 1:size(event, 2)
        dir = [event(2,i)-y(end), event(1,i)-x(end)];
        dist = sqrt((dir(1))^2+(dir(2))^2);
        if(sign(dot(dir,theta_car))==1 && dist < prevdist)
            next_obs = event(:,i);
            prevdist = dist;
        end
    end 
end