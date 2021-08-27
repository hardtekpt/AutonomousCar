delete(timerfindall);
exit=0;
myapp = app1;
pause(1);
my_timer = timer('Name', 'my_timer', 'ExecutionMode', 'fixedRate', 'Period', 0.1, ...
    'StartFcn', @(~,~)disp('started...'), ...
    'StopFcn', @(~,~)disp('stopped ...'), ...
    'TimerFcn', @timeStep);
while exit==0
    %     clear all
    clearvars -except exit myapp my_timer
    start_sim = 0;
    
    if strcmpi(get(my_timer,'Running'),'off')
        while start_sim == 0
            start_sim = myapp.start;
            exit = myapp.exit;
            pause(0.01);
            if exit == 1
                delete(myapp)
                return
            end
        end
    end
    
    if start_sim == 1
        battery = str2double(myapp.EnergyBudgetJEditField.Value)*3600;
        eventList = myapp.eventList;
        h = 0.1; M = 810; Po = 3000;  ahead=50; scale = 0.4; imp = 0;
        [xx,yy,map,pts,G,theta_car,imp] = pathplanner(h,scale,myapp.via,myapp);
%         img = imread('ist_gmaps.png');figure(2);imshow(img);hold on;plot(xx,yy);
        if imp
            myapp.start = 0;
            myapp.STARTSimulationButton.Text = "Simulation finished";
            myapp.STARTSimulationButton.FontColor = [0.85,0.33,0.10];
            myapp.Button_pause.Enable = 0;
            myapp.Button_stop.Enable = 0;
            pause(3);
            myapp.STARTSimulationButton.Text = "START Simulation";
            myapp.STARTSimulationButton.FontColor = [0.39,0.83,0.07];
            myapp.STARTSimulationButton.Enable = 1;
            myapp.SetviaconfigurationsButton.Enable = 1;
            myapp.StoptrafficsignButton.Enable = 1;
            myapp.RadarButton.Enable = 1;
            myapp.SpeedlimitedzoneButton.Enable = 1;
            myapp.PedestriancrossingtrafficsignButton.Enable = 1;
            myapp.PedestriancrossingcustomButton.Enable = 1;
            myapp.SetinitialandfinalconfigurationsButton.Enable = 1;
            myapp.ClearpreviousconfigurationsButton.Enable = 1;
            myapp.ClearprevioussimulationButton.Enable = 1;
            myapp.SaveCurrentConfigurationButton.Enable = 1;
            myapp.LoadConfigurationFileButton.Enable = 1;
        else
            if size(xx,1)== 0
                return;
            end
            
            if myapp.printPath
                myapp.plotPath(xx,yy);
            end
            
            dist = distance(xx,yy);
            map = eventMapping(eventList,xx,yy,scale,map,ahead);
            
            [Kv,limPhi] = gainVec(xx,theta_car,ahead);
            x = -1*ones(1,size(xx,2)*2); y = x; theta = x; phi = x;  u =-1*ones(2,size(xx,2)*2);  E =x;  availableEnergyPercent= x; availableEnergy= x;  next_obs = []; dir_car = [0,0];
            x(1) = xx(1); y(1) = yy(1); theta(1) = theta_car(1); phi(1) = 0; u(:,1) = [0;0]; availableEnergy(1) = battery; availableEnergyPercent(1) = 100;
            i = 1; index = 1; distsum = 0; prevV = 0; flag = 1; xref = xx(1+ahead); yref = yy(1+ahead); thetaref = theta_car(1+ahead); roadSpeed = 25; maxSpeed = roadSpeed; pedestrianMaxSpeed = 7; collision = 0; flagCol = 0; crosswalk = 0;
            goal = 1; limitedZone = 0; flagColCross = 0;
            
            UserData.roadSpeed = roadSpeed;
            UserData.crosswalk = crosswalk;
            UserData.dir_car = dir_car;
            UserData.eventList = eventList;
            UserData.pedestrianMaxSpeed = pedestrianMaxSpeed;
            UserData.i = i;
            UserData.index = index;
            UserData.xx = xx;
            UserData.yy = yy;
            UserData.x = x;
            UserData.y = y;
            UserData.prevV = prevV;
            UserData.ahead = ahead;
            UserData.scale = scale;
            UserData.xref = xref;
            UserData.yref = yref;
            UserData.theta = theta;
            UserData.thetaref = thetaref;
            UserData.Kv = Kv;
            UserData.limPhi = limPhi;
            UserData.M = M;
            UserData.Po = Po;
            UserData.availableEnergy = availableEnergy;
            UserData.availableEnergyPercent = availableEnergyPercent;
            UserData.battery = battery;
            UserData.distsum = distsum;
            UserData.map = map;
            UserData.next_obs = next_obs;
            UserData.phi = phi;
            UserData.u = u;
            UserData.E = E;
            UserData.h = h;
            UserData.maxSpeed = maxSpeed;
            UserData.collision = collision;
            UserData.flagCol = flagCol;
            UserData.teste = 0;
            UserData.app1 = myapp;
            UserData.prevPause = 0;
            UserData.limitedZone = limitedZone;
            UserData.flagColCross = flagColCross;
            UserData.stopSign = 0;
            UserData.dist = dist;
            set(my_timer, 'UserData', UserData);
            start(my_timer);
        end
    end
    
    exit = myapp.exit;
end
delete(my_timer);
delete(myapp)
