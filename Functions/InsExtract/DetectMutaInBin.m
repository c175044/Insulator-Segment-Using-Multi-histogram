function MutaBin = DetectMutaInBin(Bin)
%DETECTMUTAINBIN 
%   
MutaBin = Bin;
i = 1;
while i <= length(MutaBin)
    if Bin(i) >= 0
        incSum = 0;
        startI = i;
        while i <= length(MutaBin) && Bin(i) >= 0
            incSum = incSum + Bin(i);
            i = i + 1;
        end
        MutaBin(startI) = incSum;
    elseif Bin(i) < 0
        decSum = 0;
        while i <= length(MutaBin) && Bin(i) < 0
            decSum = decSum + Bin(i);
            i = i + 1;
        end
        MutaBin(i-1) = decSum;
    else
        i = i + 1;
    end
end
% subplot(1,2,1);barh(Bin);subplot(1,2,2);barh(MutaBin)
end

