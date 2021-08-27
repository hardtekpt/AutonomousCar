function weight = costdist(all_map,aux_pos,flag,aux_from)
weight = 0;
map = all_map;
pos = aux_pos;
i = 0;
sub = [0];
k = 0;
while size(find(sub == -1),1) == 0
    k = k + 1;
    sub = submatrix(k,map,sub,aux_pos);
end
weight = abs(1/(k^3));
end