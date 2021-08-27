function saveIndex = cleanCurves(saveIndex)
%Cleaning and joing close curves into bigger ones for the sake of the
%system stability.
    flag = 0;
    for i = 1:length(saveIndex)
        if i + 3 <= length(saveIndex) && rem(i,2)==1
            if saveIndex(i+2)-saveIndex(i+1) <= saveIndex(i+1)-saveIndex(i) || saveIndex(i+2)-saveIndex(i+1) <= saveIndex(i+3)-saveIndex(i+2)
                saveIndex = horzcat(saveIndex(1:i),saveIndex(i+3:end));
                flag = flag + 1;
            end
        end
    end
    if flag ~= 0
        saveIndex = cleanCurves(saveIndex);
    end
end