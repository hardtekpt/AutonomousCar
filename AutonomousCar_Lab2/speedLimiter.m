function u = speedLimiter(availableEnergy,amax,h,prevV,u,maxSpeed,maxW)
    maxSpeedPx = maxSpeed/(0.2*3.6); 
    %Applying speed contrains
    if  availableEnergy ~= 0
        if abs(u(1)) > maxSpeedPx
            u(1) = sign(u(1))*maxSpeedPx;
        end
        if abs(u(1)-prevV) > amax*h
            u(1) = prevV+sign(u(1)-prevV)*amax*h;
        end
        if (sign(u(1)) ~= sign(prevV) && prevV ~= 0)
            u(1) = 0;
        end
        if abs(prevV) > 1.5 && abs(u(1)) < 1.5
            u(1) = 0;
        end
        if u(1) < 0
            u(1) = 0;
        end
    else
    %     if prevV > 0
            u(1) = prevV-0.5*amax*h;
            if u(1) < 0
                u(1) = 0;
            end
    %     else
    %         u(1) = prevV+0.1*amax*h;
    %         if u(1) > 0
    %             u(1) = 0;
    %         end
    %     end
    end
    if abs(u(2)) > maxW
        u(2) = sign(u(2))*maxW;
    end
    if u(1) == 0
        u(2) = 0;
    end
end
