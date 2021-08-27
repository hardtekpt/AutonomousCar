function [count,flagCol] = collisionDetector(count,map,x,y,scale,flagCol,theta_car,event)
%%%%%%%%%%%%%%%%%%%%%%%%
% dimensions of the car
%
car_length = 3.332;
car_width = 1.508;
car_wheelbase = 2.2;
car_length_out_wheelbase = car_length - car_wheelbase;
% assume that the length out-of-wheelbase is identical at front and back of
% the car
a1 = car_length_out_wheelbase / 2;
%  car polygon constructed ccw
car_polygon = [ -a1, -car_width/2;
    car_wheelbase + a1, -car_width/2;
    car_wheelbase + a1, car_width/2;
    -1, car_width/2 ];

scale1 = 0.2;
temp = count;   
collisionInThisTimeStep = 0;
for p=1:4
    rot_car_polygon(p,:) = ([cos(theta_car), -sin(theta_car); sin(theta_car), cos(theta_car)]*car_polygon(p,:)')';
    carVertices(p,1) = rot_car_polygon(p,1)/scale1 + x;
    carVertices(p,2) = rot_car_polygon(p,2)/scale1 + y;
    if map(round(carVertices(p,2)*scale),round(carVertices(p,1)*scale)) == -1 || (map(round(carVertices(p,2)*scale),round(carVertices(p,1)*scale)) < 0 && event < -3)  %&& flagCol == 0
        collisionInThisTimeStep = 1;
%         count = temp + 1;
%         flagCol = 1;
    end
end

if flagCol == 0 && collisionInThisTimeStep == 1
    count = count + 1;
    flagCol = 1;
end

if ~collisionInThisTimeStep
    flagCol = 0;
end

end