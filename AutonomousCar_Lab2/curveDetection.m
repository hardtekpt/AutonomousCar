function saveIndex = curveDetection(theta_car,ahead,sizeX)
KvTop = 3; KvBot=0.1; PhiTop = pi/64; PhiBot=pi/6; thetaBot = 0.298; thetaTop = 5; straightReduceAhead = 30; straightThresh = 0.01;
Kv = KvTop*ones(1,sizeX); limPhi = Kv; count = 0; saveIndex = [];
%Detection of possible curves and straight lines
for i = 2:sizeX-ahead
    if abs(theta_car(i)-theta_car(i+ahead)) > thetaBot && abs(theta_car(i)-theta_car(i+ahead)) < thetaTop
        if Kv(i-1) == KvTop
            count = count + 1;
            saveIndex(count) = i;
        end
        Kv(i:end) = KvBot; limPhi(i) = PhiBot;
        
    elseif abs(theta_car(i)-theta_car(i+straightReduceAhead)) < straightThresh
        if Kv(i-1) == KvBot
            count = count + 1;
            saveIndex(count) = i;
        end
        Kv(i:end) = KvTop; limPhi(i) = PhiTop;
    end
end
% scatter(xx(saveIndex),yy(saveIndex)); pause(0.01);
%Clean SaveIndex to better represent starting or ending a curve
saveIndex = horzcat(saveIndex,size(theta_car,2));
saveIndex = cleanCurves(saveIndex);
% scatter(xx(saveIndex),yy(saveIndex)); pause(0.01);
end