function [map,pts,adj]=GenADJ()
cor = imread('0_4_better_color.png');

map = zeros(size(cor,1),size(cor,2));
sz = 1;
pos = [];
pts = [];
%% Tratamento de imagem
for i = 1:size(cor,1)
    for j = 1:size(cor,2)
        r = cor(i,j,1); g = cor(i,j,2);b = cor(i,j,3) ;
        if (r ~= 0 && r~=255) || (g ~= 0 && g~=255) || (b ~= 0 && b~=255 )
                    cor(i,j,1) = 0; cor(i,j,2)=0; cor(i,j,3) =0;
        end
    end
end

%% Criação do mapa nós
for i = 1:size(cor,1)
    for j = 1:size(cor,2)
            r = cor(i,j,1); g = cor(i,j,2);b = cor(i,j,3) ;
            if r == 0 && g == 0 && b == 0
                    map(i,j) = -1;
            else
                   map(i,j) = sz;
                   pos = horzcat(pos,[i;j]);
                   pts = horzcat(pts,[j;i]);
                    sz = sz + 1;
            end
    end
end
%% Matriz de Adjacências
adj = zeros(sz-1);
count = 0;
flag = 0;

for i = 1:size(cor,1)
    for j = 1:size(cor,2)
            r = cor(i,j,1); g = cor(i,j,2);b = cor(i,j,3) ;
            if (r <= 10 && g <= 10 && b <= 10) %Black
                flag = -1;
            elseif g == 255 && r == 0 %Green
                    flag = 1;
            elseif b == 255 && r == 0 %Blue
                    flag = 2;
            elseif r == 255 && g == 0 %Red
                  flag = 3;        
            elseif  r == 255 && g == 255 && b == 0 %Yellow
                    flag = 4;
            elseif (r == 255 && g ==255 && b == 255)%White
                    flag = 0;
            end

            if flag ~= -1
                kini = -1; kend = 1; vini = -1; vend = 1;
                if flag == 1
                    kini = 0;
                elseif flag == 2
                    vend = 0;
                elseif flag == 3
                    kend = 0;
                elseif flag == 4
                    vini = 0;   
                end

                from = map(i,j);
                for k = kini:kend
                    for v = vini:vend
                        if i+k > 0 && j + v > 0 && (k ~= 0 || v ~= 0) && i + k <= size(map,1) && j + v <= size(map,2)
                            if map(i+k,j+v) ~= -1 
                                to = map(i+k,j+v);
                                aux_from = [i,j];
                                aux_to = [i+k,j+v];
                                weight = costdist(map,aux_to,flag,aux_from);
                                adj(from,to) = round(10000*(weight));     
                            end
                        end
                    end
                end
            end
            flag = 0;
    end
end
end