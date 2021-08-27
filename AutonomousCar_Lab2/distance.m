function distsum = distance(x,y)
distsum = 0;
for i = 2:size(x,2)
    distsum = distsum + sqrt((x(i)-x(i-1))^2+(y(i)-y(i-1))^2);
end
end