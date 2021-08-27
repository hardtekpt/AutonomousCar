function [Kv,limPhi] = gainVec(xx,theta_car,ahead)
%Curve Detection
saveIndex = curveDetection(theta_car,ahead,size(xx,2));
%Populate Kv and limPhi
minKv=0.5; medKv = 1; maxKv = 2.85; minPhi=pi/100; maxPhi = pi/16; percenth = 0.2;percentl = 0.3;
[Kv,limPhi] = vecGen(saveIndex,minKv,medKv,maxKv,minPhi,maxPhi,percenth,percentl,xx);
end