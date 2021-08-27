function [availableEnergy,availableEnergyPercent]=energyCounter(i,battery,availableEnergy,availableEnergyPercent,E)
%Computing the available energy at each time step
    if E(i-1) ~= 0
        availableEnergy(i) = battery-sum(E(1:i-1));
    else
        availableEnergy(i)=0;
    end
    availableEnergyPercent(i) =100*availableEnergy(i)/battery;
end