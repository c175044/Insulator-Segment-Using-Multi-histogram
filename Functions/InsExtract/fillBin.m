function Bin = fillBin(Bin)
% barh(Bin)
binNum = size(Bin,1);
for i = 1:binNum
    if Bin(i) == 0
        % Previous non-zero bin
        lastBin = 0; %Not found, may be at the first position, default is 0
        for j = i:-1:1
            if Bin(j) ~= 0
                lastBin = Bin(j);
                break
            end
        end

        % next non-zero bin
        nextBin = 0; %Not found, may be at the end, default is 0
        for j = i:binNum
            if Bin(j) ~= 0
                nextBin = Bin(j);
                break
            end
        end
        Bin(i) = floor((lastBin + nextBin) / 2);
    end
end
% barh(Bin)
end

