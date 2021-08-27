function [UserData] = timeStep(obj,event) 
UserData = get(obj, 'UserData');

if size(UserData.eventList,1) ~= 0
    aux = find(UserData.eventList(3,:) == -4);
    if size(aux,1) ~= 0
        for j=1:length(aux)
            r_dur = UserData.eventList(4,aux(j));
            g_dur = UserData.eventList(5,aux(j));
            if rem(UserData.i*UserData.h,r_dur + g_dur) == 0 || UserData.i == 1
                pos_x = UserData.eventList(2,aux(j));
                pos_y = UserData.eventList(1,aux(j));
                UserData.map(pos_x, pos_y) = -aux(j)-1;
                scatter(UserData.app1.UIAxes, round(pos_y /UserData.scale),round(pos_x /UserData.scale), 'red');
            elseif rem(UserData.i*UserData.h-r_dur,r_dur + g_dur) == 0 || UserData.i*UserData.h == r_dur
                pos_x = UserData.eventList(2,aux(j));
                pos_y = UserData.eventList(1,aux(j));
                UserData.map(pos_x, pos_y) = 0;
                scatter(UserData.app1.UIAxes, round(pos_y /UserData.scale),round(pos_x /UserData.scale), 'green');
            end
        end
    end
end

while UserData.app1.pause == 1
    [a,b,~] = UserData.app1.draw(UserData.x,UserData.y,1,UserData.dir_car);
    UserData.prevPause = 1;
    if UserData.app1.stop == 1
        stop(obj);
        UserData.app1.start = 0;
        UserData.app1.STARTSimulationButton.Text = "Simulation finished";
        UserData.app1.STARTSimulationButton.FontColor = [0.85,0.33,0.10];
        UserData.app1.Button_pause.Enable = 0;
        UserData.app1.Button_stop.Enable = 0;
        pause(3);
        UserData.app1.STARTSimulationButton.Text = "START Simulation";
        UserData.app1.STARTSimulationButton.FontColor = [0.39,0.83,0.07];
        UserData.app1.STARTSimulationButton.Enable = 1;
        UserData.app1.SetviaconfigurationsButton.Enable = 1;
        UserData.app1.StoptrafficsignButton.Enable = 1;
        UserData.app1.RadarButton.Enable = 1;
        UserData.app1.SpeedlimitedzoneButton.Enable = 1;
        UserData.app1.PedestriancrossingtrafficsignButton.Enable = 1;
        UserData.app1.PedestriancrossingcustomButton.Enable = 1;
        UserData.app1.SetinitialandfinalconfigurationsButton.Enable = 1;
        UserData.app1.ClearpreviousconfigurationsButton.Enable = 1;
        UserData.app1.ClearprevioussimulationButton.Enable = 1;
        UserData.app1.SaveCurrentConfigurationButton.Enable = 1;
        UserData.app1.LoadConfigurationFileButton.Enable = 1;
        return;
    end
    UserData.app1.clearPreviousSimulation(a,b);
end

if UserData.prevPause == 1 && UserData.app1.pause == 0
    UserData.app1.clearPreviousSimulation(a,b);
    UserData.prevPause = 0;
end

goal = 0;
%Dynamical event insertion
UserData.map = obsInsert(UserData.i*UserData.h,UserData.map,UserData.eventList);

if UserData.i > 1 %Find new reference
    prevUserData.index = UserData.index;
    [UserData.index,~] = findnpoint(UserData.xx,UserData.yy,UserData.x(UserData.i-1:UserData.i),UserData.y(UserData.i-1:UserData.i),UserData.ahead,prevUserData.index);
end

if UserData.i > 10 %Sensor/Detecting events/Save direction
    [UserData.next_obs,temp] = directedSensor(UserData.crosswalk,UserData.dir_car,UserData.map,round(UserData.scale*UserData.x(UserData.i-10:UserData.i)),round(UserData.scale*UserData.y(UserData.i-10:UserData.i)));
    if UserData.crosswalk == 0 && abs(UserData.prevV) > 10
        UserData.dir_car = temp;
    end
end 
if size(UserData.next_obs,1) ~= 0 %Resume after a stop
    if UserData.eventList(3,(1+UserData.next_obs(3))*-1) ~= -2 
        UserData.stopSign = 0;
    end
end

UserData.crosswalk = 0;
if (size(UserData.next_obs, 1) == 0) || UserData.stopSign == 1 %No events
    if UserData.limitedZone == 0
        UserData.maxSpeed = UserData.roadSpeed;
    else
        UserData.maxSpeed = UserData.limitedZone;
    end
    UserData.flagColCross = 0;
    UserData.xref = UserData.xx(UserData.index+UserData.ahead); UserData.yref = UserData.yy(UserData.index+UserData.ahead); UserData.thetaref = atan2(UserData.yy(UserData.index+UserData.ahead)-UserData.y(UserData.i), UserData.xx(UserData.index+UserData.ahead)-UserData.x(UserData.i));
elseif UserData.eventList(3,(1+UserData.next_obs(3))*-1) == -2 && UserData.stopSign == 0 %Stop sign
    UserData.maxSpeed = UserData.roadSpeed;
    UserData.next_obs(1:2)=round(UserData.next_obs(1:2)./UserData.scale);
    [UserData.indexObs,~] = findnpoint(UserData.xx,UserData.yy,UserData.next_obs(1),UserData.next_obs(2),UserData.ahead,prevUserData.index);
    UserData.xref = UserData.xx(UserData.indexObs); UserData.yref =  UserData.yy(UserData.indexObs); UserData.thetaref = atan2(UserData.yref-UserData.y(UserData.i), UserData.xref-UserData.x(UserData.i));
    [UserData.collision,UserData.flagCol,UserData.i,UserData.index,~,~,UserData.x,UserData.y,UserData.prevV,UserData.ahead,UserData.scale,UserData.xref,UserData.yref,UserData.theta,UserData.thetaref,UserData.Kv,UserData.limPhi,UserData.M,UserData.Po,UserData.availableEnergy,UserData.availableEnergyPercent,UserData.battery,UserData.distsum,UserData.map,UserData.next_obs,UserData.phi,UserData.u,UserData.E,UserData.h,UserData.maxSpeed] = mainLoop(UserData.i,UserData.index,UserData.xx(1:(UserData.indexObs)),UserData.yy(1:(UserData.indexObs)),UserData.x,UserData.y,UserData.prevV,UserData.ahead,UserData.scale,UserData.xref,UserData.yref,UserData.theta,UserData.thetaref,UserData.Kv,UserData.limPhi,UserData.M,UserData.Po,UserData.availableEnergy,UserData.availableEnergyPercent,UserData.battery,UserData.distsum,UserData.map,UserData.next_obs,UserData.phi,UserData.u,UserData.E,UserData.h,UserData.maxSpeed,UserData.app1,UserData.collision,UserData.flagCol,goal,UserData.dir_car,UserData.dist,UserData.eventList(3,(1+UserData.next_obs(3))*-1));
    UserData.stopSign = 1;
elseif UserData.eventList(3,(1+UserData.next_obs(3))*-1) == -3 %Radar
    UserData.maxSpeed = UserData.eventList(4,(1+UserData.next_obs(3))*-1);
    UserData.xref = UserData.xx(UserData.index+UserData.ahead); UserData.yref = UserData.yy(UserData.index+UserData.ahead); UserData.thetaref = atan2(UserData.yy(UserData.index+UserData.ahead)-UserData.y(UserData.i), UserData.xx(UserData.index+UserData.ahead)-UserData.x(UserData.i));
elseif UserData.eventList(3,(1+UserData.next_obs(3))*-1) == -4 %UserData.crosswalk sign
    UserData.maxSpeed = UserData.pedestrianMaxSpeed;
    safetyDistance = 120;
    UserData.next_obs(1:2)=round(UserData.next_obs(1:2)./UserData.scale);
    [UserData.indexObs,~] = findnpoint(UserData.xx,UserData.yy,UserData.next_obs(1),UserData.next_obs(2),safetyDistance,prevUserData.index);
    UserData.xref = UserData.xx(UserData.indexObs-safetyDistance); UserData.yref =  UserData.yy(UserData.indexObs-safetyDistance); UserData.thetaref = atan2(UserData.yref-UserData.y(UserData.i), UserData.xref-UserData.x(UserData.i));
    [UserData.collision,UserData.flagCol,UserData.i,UserData.index,~,~,UserData.x,UserData.y,UserData.prevV,UserData.ahead,UserData.scale,UserData.xref,UserData.yref,UserData.theta,UserData.thetaref,UserData.Kv,UserData.limPhi,UserData.M,UserData.Po,UserData.availableEnergy,UserData.availableEnergyPercent,UserData.battery,UserData.distsum,UserData.map,UserData.next_obs,UserData.phi,UserData.u,UserData.E,UserData.h,UserData.maxSpeed] = mainLoop(UserData.i,UserData.index,UserData.xx(1:(UserData.indexObs)),UserData.yy(1:(UserData.indexObs)),UserData.x,UserData.y,UserData.prevV,UserData.ahead,UserData.scale,UserData.xref,UserData.yref,UserData.theta,UserData.thetaref,UserData.Kv,UserData.limPhi,UserData.M,UserData.Po,UserData.availableEnergy,UserData.availableEnergyPercent,UserData.battery,UserData.distsum,UserData.map,UserData.next_obs,UserData.phi,UserData.u,UserData.E,UserData.h,UserData.maxSpeed,UserData.app1,UserData.collision,UserData.flagCol,goal,UserData.dir_car,UserData.dist,UserData.eventList(3,(1+UserData.next_obs(3))*-1));
    [UserData.indexCrossMom,~] = findnpoint(UserData.xx,UserData.yy,UserData.x(UserData.i),UserData.y(UserData.i),safetyDistance,prevUserData.index);
    if UserData.indexCrossMom >= UserData.indexObs
        if UserData.flagColCross == 0
            UserData.collision = UserData.collision + 1;
            UserData.flagColCross = 1;
        end
    end
    UserData.crosswalk = 1;
elseif UserData.eventList(3,(1+UserData.next_obs(3))*-1) == -5 %Custom UserData.crosswalk
    UserData.maxSpeed = UserData.pedestrianMaxSpeed;
    safetyDistance = 120;
    UserData.next_obs(1:2)=round(UserData.next_obs(1:2)./UserData.scale);
    [UserData.indexObs,~] = findnpoint(UserData.xx,UserData.yy,UserData.next_obs(1),UserData.next_obs(2),safetyDistance,prevUserData.index);
    UserData.xref = UserData.xx(UserData.indexObs-safetyDistance); UserData.yref =  UserData.yy(UserData.indexObs-safetyDistance); UserData.thetaref = atan2(UserData.yref-UserData.y(UserData.i), UserData.xref-UserData.x(UserData.i));
    [UserData.collision,UserData.flagCol,UserData.i,UserData.index,~,~,UserData.x,UserData.y,UserData.prevV,UserData.ahead,UserData.scale,UserData.xref,UserData.yref,UserData.theta,UserData.thetaref,UserData.Kv,UserData.limPhi,UserData.M,UserData.Po,UserData.availableEnergy,UserData.availableEnergyPercent,UserData.battery,UserData.distsum,UserData.map,UserData.next_obs,UserData.phi,UserData.u,UserData.E,UserData.h,UserData.maxSpeed] = mainLoop(UserData.i,UserData.index,UserData.xx(1:(UserData.indexObs)),UserData.yy(1:(UserData.indexObs)),UserData.x,UserData.y,UserData.prevV,UserData.ahead,UserData.scale,UserData.xref,UserData.yref,UserData.theta,UserData.thetaref,UserData.Kv,UserData.limPhi,UserData.M,UserData.Po,UserData.availableEnergy,UserData.availableEnergyPercent,UserData.battery,UserData.distsum,UserData.map,UserData.next_obs,UserData.phi,UserData.u,UserData.E,UserData.h,UserData.maxSpeed,UserData.app1,UserData.collision,UserData.flagCol,goal,UserData.dir_car,UserData.dist,UserData.eventList(3,(1+UserData.next_obs(3))*-1));
   [UserData.indexCrossMom,~] = findnpoint(UserData.xx,UserData.yy,UserData.x(UserData.i),UserData.y(UserData.i),safetyDistance,prevUserData.index);
   if UserData.indexCrossMom >= UserData.indexObs
       if UserData.flagColCross == 0
       UserData.collision = UserData.collision + 1;
       UserData.flagColCross = 1;
       end
   end
    UserData.crosswalk = 1;
elseif UserData.eventList(3,(1+UserData.next_obs(3))*-1) == -6 %Start limited zone
    UserData.maxSpeed = UserData.eventList(4,(1+UserData.next_obs(3))*-1);
    UserData.limitedZone = UserData.maxSpeed;
    UserData.xref = UserData.xx(UserData.index+UserData.ahead); UserData.yref = UserData.yy(UserData.index+UserData.ahead); UserData.thetaref = atan2(UserData.yy(UserData.index+UserData.ahead)-UserData.y(UserData.i), UserData.xx(UserData.index+UserData.ahead)-UserData.x(UserData.i));
elseif UserData.eventList(3,(1+UserData.next_obs(3))*-1) == -7 %End limited zone
    UserData.maxSpeed = UserData.roadSpeed;
    UserData.limitedZone = 0;
    UserData.xref = UserData.xx(UserData.index+UserData.ahead); UserData.yref = UserData.yy(UserData.index+UserData.ahead); UserData.thetaref = atan2(UserData.yy(UserData.index+UserData.ahead)-UserData.y(UserData.i), UserData.xx(UserData.index+UserData.ahead)-UserData.x(UserData.i));
end


if UserData.crosswalk == 0 
    UserData.i = UserData.i + 1;
    %Controller
    [UserData.x(UserData.i),UserData.y(UserData.i),UserData.theta(UserData.i),UserData.phi(UserData.i),UserData.u(:,UserData.i-1),UserData.E(UserData.i-1)] = stepControl(UserData.x(UserData.i-1),UserData.xref,UserData.y(UserData.i-1),UserData.yref,UserData.theta(UserData.i-1),UserData.thetaref,UserData.h,UserData.phi(UserData.i-1),UserData.Kv(UserData.index+UserData.ahead),UserData.limPhi(UserData.index+UserData.ahead),UserData.M,UserData.Po,UserData.prevV,UserData.availableEnergy(UserData.i-1),UserData.maxSpeed);
    %Energy monitor
    [UserData.availableEnergy,UserData.availableEnergyPercent]=energyCounter(UserData.i,UserData.battery,UserData.availableEnergy,UserData.availableEnergyPercent,UserData.E);
    UserData.prevV = UserData.u(1,UserData.i-1);
    UserData.distsum = UserData.distsum + sqrt((UserData.x(UserData.i)-UserData.x(UserData.i-1))^2+(UserData.y(UserData.i)-UserData.y(UserData.i-1))^2);
    UserData.app1.draw(UserData.x,UserData.y,goal,goal);
   %Collision Detection
    [UserData.collision,UserData.flagCol] = collisionDetector(UserData.collision,UserData.map,UserData.x(UserData.i),UserData.y(UserData.i),UserData.scale,UserData.flagCol,UserData.theta(UserData.i),0);
end

UserData.app1.SpeedometerKmhGauge.Value = round(UserData.u(1,UserData.i-1)*0.2*3.6);
UserData.app1.CollisionCounterGauge.Text = num2str(round(UserData.collision));
UserData.app1.BatteryStatusGauge.Value = round(UserData.availableEnergyPercent(UserData.i));
UserData.app1.sLabel.Text = num2str(round(UserData.i*UserData.h*10)/10);
UserData.app1.plotLiveSpeed(UserData.i,UserData.h,UserData.u(1,:));
UserData.app1.plotLiveAng(UserData.i,UserData.h,UserData.u(2,:));
UserData.app1.plotLiveEnergy(UserData.i,UserData.h,UserData.E);
UserData.gainVect(UserData.i) = UserData.Kv(UserData.index);
UserData.app1.plotLiveGain(UserData.gainVect,UserData.i,UserData.h);
UserData.app1.Label_td.Text = num2str(round(UserData.distsum*0.2*100)/100);
UserData.app1.Label_as.Text = num2str(round((UserData.distsum*0.2*3.6)/(UserData.i*UserData.h)*100)/100);
UserData.app1.Label_pc.Text = num2str(round(sum(UserData.E(1:UserData.i-1))/(1000*UserData.i*UserData.h )*100)/100);
UserData.app1.Label_ec.Text = num2str(round(sum(UserData.E(1:UserData.i-1))/3600000*1000)/1000);
range = UserData.availableEnergyPercent(UserData.i) * (UserData.distsum*0.2 / (100-UserData.availableEnergyPercent(UserData.i)));
UserData.app1.Label_er.Text = num2str(round(range*100)/100);
eta = (UserData.dist-UserData.distsum) / (UserData.distsum/(UserData.i*UserData.h));
if eta < 0
    eta = 0;
end
UserData.app1.Label_eta.Text = num2str(round(eta*10)/10);


if ~(size(UserData.next_obs, 1) ~= 0 || abs(UserData.prevV) > 0.2 )
    stop(obj);
    goal = 1;
    UserData.app1.draw(UserData.x,UserData.y,goal,UserData.dir_car);
    UserData.x=cleanVect(UserData.x); UserData.y=cleanVect(UserData.y);UserData.theta=cleanVect(UserData.theta);UserData.u=cleanVect(UserData.u);UserData.E=cleanVect(UserData.E);UserData.availableEnergy=cleanVect(UserData.availableEnergy);UserData.availableEnergyPercent=cleanVect(UserData.availableEnergyPercent);
    UserData.app1.Label_td.Text = num2str(round(UserData.distsum*0.2*100)/100);
    UserData.app1.Label_as.Text = num2str(round((UserData.distsum*0.2*3.6)/(UserData.i*UserData.h)*100)/100);
    UserData.app1.Label_eta.Text = num2str(0);
    UserData.app1.Label_pc.Text = num2str(round(sum(UserData.E(1:UserData.i-1))/(1000*UserData.i*UserData.h )*100)/100);
    UserData.app1.Label_ec.Text = num2str(round(sum(UserData.E(1:UserData.i-1))/3600000*100)/100);
  UserData.app1.start = 0;
    UserData.app1.STARTSimulationButton.Text = "Simulation finished";
    UserData.app1.STARTSimulationButton.FontColor = [0.85,0.33,0.10];
    UserData.app1.Button_pause.Enable = 0;
    UserData.app1.Button_stop.Enable = 0;
    pause(3);
    UserData.app1.STARTSimulationButton.Text = "START Simulation";
    UserData.app1.STARTSimulationButton.FontColor = [0.39,0.83,0.07];
    UserData.app1.STARTSimulationButton.Enable = 1;
    UserData.app1.SetviaconfigurationsButton.Enable = 1;
    UserData.app1.StoptrafficsignButton.Enable = 1;
    UserData.app1.RadarButton.Enable = 1;
    UserData.app1.SpeedlimitedzoneButton.Enable = 1;
    UserData.app1.PedestriancrossingtrafficsignButton.Enable = 1;
    UserData.app1.PedestriancrossingcustomButton.Enable = 1;
    UserData.app1.SetinitialandfinalconfigurationsButton.Enable = 1;
    UserData.app1.ClearpreviousconfigurationsButton.Enable = 1;
    UserData.app1.ClearprevioussimulationButton.Enable = 1;
    UserData.app1.SaveCurrentConfigurationButton.Enable = 1;
    UserData.app1.LoadConfigurationFileButton.Enable = 1;
else

    if UserData.app1.stop == 1
        stop(obj);
        UserData.app1.start = 0;
        UserData.app1.STARTSimulationButton.Text = "Simulation finished";
        UserData.app1.STARTSimulationButton.FontColor = [0.85,0.33,0.10];
        UserData.app1.Button_pause.Enable = 0;
        UserData.app1.Button_stop.Enable = 0;
        pause(3);
        UserData.app1.STARTSimulationButton.Text = "START Simulation";
        UserData.app1.STARTSimulationButton.FontColor = [0.39,0.83,0.07];
        UserData.app1.STARTSimulationButton.Enable = 1;
        UserData.app1.SetviaconfigurationsButton.Enable = 1;
        UserData.app1.StoptrafficsignButton.Enable = 1;
        UserData.app1.RadarButton.Enable = 1;
        UserData.app1.SpeedlimitedzoneButton.Enable = 1;
        UserData.app1.PedestriancrossingtrafficsignButton.Enable = 1;
        UserData.app1.PedestriancrossingcustomButton.Enable = 1;
        UserData.app1.SetinitialandfinalconfigurationsButton.Enable = 1;
        UserData.app1.ClearpreviousconfigurationsButton.Enable = 1;
        UserData.app1.ClearprevioussimulationButton.Enable = 1;
        UserData.app1.SaveCurrentConfigurationButton.Enable = 1;
        UserData.app1.LoadConfigurationFileButton.Enable = 1;
        return;
    end
    set(obj, 'UserData', UserData);
end
end

