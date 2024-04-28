function MaxLabelInd = MaxLabel(Labels)
% Returns the label number corresponding to the largest cluster
    UniLabels = unique(Labels);
    cnum = length(UniLabels);
    CptsNum = zeros(cnum,1);
    for i = 1:cnum
        CptsNum(i) = sum(Labels==UniLabels(i));
    end
    [~,tepInd] = max(CptsNum);
    MaxLabelInd = UniLabels(tepInd);
end
