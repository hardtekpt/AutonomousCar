function [xx,yy,theta_car]=teste_traj(h,P,pts)
x = pts(1,P);
y = pts(2,P);
npt = length(x);        % number of via points, including initial and final
nvia = [0:1:npt-1];
step = 10;
csinterp_x = csapi(nvia(1:step:end),x(1:step:end));
csinterp_y = csapi(nvia(1:step:end),y(1:step:end));
time = [0:h:npt-1];
xx = fnval(csinterp_x, time);
yy = fnval(csinterp_y, time);
%plot(xx(1:end),yy(1:end));
tp = length(xx);
sp = round(tp / 10);
theta_car = zeros(1,tp);
for k=1:1:tp,
    if k+1<=tp
        theta_car(k) = atan2(yy(k+1)-yy(k), xx(k+1)-xx(k));
    else
        theta_car(k) = atan2(yy(k)-yy(k-1), xx(k)-xx(k-1));
    end
end

end

