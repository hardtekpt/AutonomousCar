function [Kv,limPhi] = vecGen(saveIndex,minKv,medKv,maxKv,minPhi,maxPhi,percenth,percentl,xx)
 Kv = minKv*ones(1,size(xx,2)); limPhi =maxPhi*ones(1,size(xx,2));
for i = 1:size(saveIndex,2)
    to = saveIndex(i);
    if i == 1 %Way to the first curve
        from = 1;
        preview = round((to-from)*percenth);
        Kv(from:from + preview) = linspace(medKv,maxKv,preview+1);
        Kv(from+preview:to-preview) = maxKv;
        Kv(to-preview:to) = linspace(maxKv,medKv,preview+1);
        limPhi(from:to) = minPhi;
    elseif rem(i,2) == 1 %Inside a straight line
        from = saveIndex(i-1);
        preview = round((to-from)*percenth);
        Kv(from:from + preview) = linspace(medKv,maxKv,preview+1);
        Kv(from+preview:to-preview) = maxKv;
        Kv(to-preview:to) = linspace(maxKv,medKv,preview+1);
        limPhi(from:to) = minPhi;
    elseif rem(i,2) == 0 %Inside a curve
        from = saveIndex(i-1);
        preview = round((to-from)*percentl);
        Kv(from:from + preview) = linspace(medKv,minKv,preview+1);
        Kv(from+preview:to-preview) = minKv;
        Kv(to-preview:to) = linspace(minKv,medKv,preview+1);
    end
end