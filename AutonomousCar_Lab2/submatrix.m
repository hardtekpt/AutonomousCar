function sub = submatrix(k,map,sub,aux_pos)
        fromx = aux_pos(1)-k;
        if fromx <= 0
            fromx = 1; end
        if fromx > size(map,1)
            fromx = size(map,1); end
        tox = aux_pos(1)+k;
        if tox <= 0
            tox = 1; end
        if tox > size(map,1)
            tox = size(map,1); end
        fromy = aux_pos(2)-k;
        if fromy <= 0
            fromy = 1; end
        if fromy > size(map,2)
            fromy = size(map,2); end
        toy = aux_pos(2)+k;
        if toy <= 0
            toy = 1; end
        if toy > size(map,2)
            toy = size(map,2); end
        
    sub = map(fromx:tox,fromy:toy);
end