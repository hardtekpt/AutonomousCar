function [xx,yy,map,pts,G,theta_car,imp] = pathplanner(h,scale,via,app)
close all;
 P = []; xx=[]; yy=[]; G=[];theta_car=[];imp=0;
img = imread('ist_gmaps.png');
img = imresize(img,scale);
% [map,pts,adj]=GenADJ(); 
file = load('pts.mat'); pts = file.pts;
file = load('map.mat'); map = file.map;
%Getting Via Points
%via = getFromTo(scale);
% via = evalin('base','via'); 
for i = 1 : size(via,2)-1
%Relating each via point to its node number
to = round(via(:,i+1));
from = round(via(:,i));
fromNode = map(from(2),from(1));
toNode = map(to(2),to(1));
if fromNode == -1 || toNode == -1
    disp('Invalid Points! Choose the free space!');
    return;
else
% G = digraph(adj);
file = load('graph.mat'); G = file.G;
%Search in the graph for the shortest path
P = horzcat(P,shortestpath(G,fromNode,toNode));  
if size(P,1) == 0
    app.warning();
    imp = 1;
    return;
end
end
end

%Interpolation of all the path
[xx,yy,theta_car]=teste_traj(h,P,round((1/scale)*pts));
end
            