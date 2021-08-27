function x=cleanVect(x)
%Cleaning vector's useless information
    aux = find(x(1,:) == -1); 
    if size(aux,1) ~= 0 
        x = x(:,1:aux(1)-1);
    end
end