function senses = lidar(map,x,y,range)
senses = [];
senses = submatrix(floor(range/2),map,senses,[y,x]);
end