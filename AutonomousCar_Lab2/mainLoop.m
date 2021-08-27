function [collision,flagCol,i,index,xx,yy,x,y,prevV,ahead,scale,xref,yref,theta,thetaref,Kv,limPhi,M,Po,availableEnergy,availableEnergyPercent,battery,distsum,map,next_obs,phi,u,E,h,maxSpeed] = mainLoop(i,index,xx,yy,x,y,prevV,ahead,scale,xref,yref,theta,thetaref,Kv,limPhi,M,Po,availableEnergy,availableEnergyPercent,battery,distsum,map,next_obs,phi,u,E,h,maxSpeed,app,collision,flagCol,goal,dir_car,dist,event)
%Loop to handle events.
KvObs = 1.2;
prevIndex =  index;
if  ( i==1 || abs( prevV) > 0.2)
while ( i==1 || abs( prevV) > 0.2)
    if  i > 1 &&  i <= size( xx,2)
        [ index,~] = findnpoint( xx, yy, x( i-1: i), y( i-1: i),ahead,prevIndex);
        prevIndex = index;
    end
        
    i = i + 1;
    %Controller
    [x(i),y(i),theta(i),phi(i),u(:,i-1),E(i-1)] = stepControl(x(i-1),xref,y(i-1),yref,theta(i-1),thetaref,h,phi(i-1),KvObs,limPhi(index+ahead),M,Po,prevV,availableEnergy(i-1),maxSpeed);
    %Energy monitor
    [availableEnergy,availableEnergyPercent]=energyCounter(i,battery,availableEnergy,availableEnergyPercent,E);
    prevV = u(1,i-1);
    distsum = distsum + sqrt((x(i)-x(i-1))^2+(y(i)-y(i-1))^2);
    app.draw(x,y,goal,goal);
    %Collision detection
    [collision,flagCol] = collisionDetector(collision,map,x(i),y(i),scale,flagCol,theta(i),event);
    app.SpeedometerKmhGauge.Value = round(u(1,i-1)*0.2*3.6);
    app.CollisionCounterGauge.Text = num2str(round(collision));
    app.BatteryStatusGauge.Value = round(availableEnergyPercent(i));
    app.sLabel.Text = num2str(round(i*h*10)/10);
    app.plotLiveSpeed(i,h,u(1,:));
    app.plotLiveAng(i,h,u(2,:));
    app.plotLiveEnergy(i,h,E);
    app.Label_td.Text = num2str(round(distsum*0.2*100)/100);
    app.Label_as.Text = num2str(round((distsum*0.2*3.6)/(i*h)*100)/100);
    eta = (dist-distsum) / (distsum/(i*h));
    if eta < 0
        eta = 0;
    end
    app.Label_eta.Text = num2str(round(eta*10)/10);
    app.Label_pc.Text = num2str(round(sum(E(1:i-1))/(1000*i*h )*100)/100);
    app.Label_ec.Text = num2str(round(sum(E(1:i-1))/3600000*1000)/1000);
end
else
    i = i + 1;
    x(i) = x(i-1); y(i) = y(i-1); theta(i) = theta(i-1); phi(i) = phi(i-1); u(:,i-1)=u(:,i-2); E(i-1) = E(i-2);
    [availableEnergy,availableEnergyPercent]=energyCounter(i,battery,availableEnergy,availableEnergyPercent,E);
    prevV = u(1,i-1);
    distsum = distsum + sqrt((x(i)-x(i-1))^2+(y(i)-y(i-1))^2);
    [a,b,~] = app.draw(x,y,1,dir_car);
    [collision,flagCol] = collisionDetector(collision,map,x(i),y(i),scale,flagCol,theta(i),event);
    app.clearPreviousSimulation(a,b);
    app.SpeedometerKmhGauge.Value = round(u(1,i-1)*0.2*3.6);
    app.CollisionCounterGauge.Text = num2str(round(collision));
    app.BatteryStatusGauge.Value = round(availableEnergyPercent(i));
    app.sLabel.Text = num2str(round(i*h*10)/10);
    app.plotLiveSpeed(i,h,u(1,:));
    app.plotLiveAng(i,h,u(2,:));
    app.plotLiveEnergy(i,h,E);
    app.Label_td.Text = num2str(round(distsum*0.2*100)/100);
    app.Label_as.Text = num2str(round((distsum*0.2*3.6)/(i*h)*100)/100);
    eta = (dist-distsum) / (distsum/(i*h));
    if eta < 0
        eta = 0;
    end
    app.Label_eta.Text = num2str(round(eta*10)/10);
    app.Label_pc.Text = num2str(round(sum(E(1:i-1))/(1000*i*h )*100)/100);
    app.Label_ec.Text = num2str(round(sum(E(1:i-1))/3600000*1000)/1000);
end

end