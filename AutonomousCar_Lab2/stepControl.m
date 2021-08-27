function [x,y,theta,phi,u,E]=stepControl(x,xref,y,yref,theta,thetaref,h,phi,Kv,limPhi,M,Po,prevV,availableEnergy,maxSpeed)
amax = 3/(0.2*3.6*h);  maxW=pi/4; 
Ks = 1; Ki = 100; L = 2.2;
K = [Kv,0,0;0,Ki,Ks];
We=[xref-x;yref-y;thetaref-theta];
b = [cos(theta),sin(theta),0;-sin(theta),cos(theta),0;0,0,1]*We;

%Model computation
u = K*b;
u = speedLimiter(availableEnergy,amax,h,prevV,u,maxSpeed,maxW);
dt = [cos(theta) 0;sin(theta),0;tan(phi)/L,0;0,1]*u;

%Integral plus noise (5% of the variation in this time step)
x = (dt(1)*h + x)+((dt(1)*h*0.1)*(2*(rand()-0.5)));
y = (dt(2)*h + y)+((dt(2)*h*0.1)*(2*(rand()-0.5)));
theta = (dt(3)*h + theta)+((dt(3)*h*0.1)*(2*(rand()-0.5)));
phi = (dt(4)*h + phi)+((dt(4)*h*0.1)*(2*(rand()-0.5)));

%Steering Wheel Limitation
if abs(phi) > limPhi
    phi = sign(phi)*limPhi;
end

%Energy computation
if availableEnergy > 0
    v = u(1)*0.2;
    prevV = prevV*0.2;
    a = (v-prevV)/h;
    if a <= 0
        a = 0;
    end
    E = h*((M*a*v)+Po);
else
    E = 0;
end

%Can't spend more than what it is available
if E > availableEnergy
    availableEnergy = 0;
    [x,y,theta,phi,u,E]=stepControl(x,xref,y,yref,theta,thetaref,h,phi,Kv,limPhi,M,Po,prevV/0.2,availableEnergy,maxSpeed);
end
end