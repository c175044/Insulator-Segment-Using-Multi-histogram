function [MPts,Len] = mergeCell3(cellPts)
%MERGECELL 
MPts = zeros(0,4);
Len = [];
k = 0;
for i = 1:size(cellPts,1)
    for j = 1:size(cellPts,2)
        Ptsk = cellPts{i,j};% drowPts(Ptsk,'.r')
        if size(Ptsk,1) > 5
            LjR3 = RotawithAxle(Ptsk,3);
            LjR32 = RotawithAxle(LjR3,2);
            len = max(LjR32(:,1)) - min(LjR32(:,1));
        else
            len = 0;
        end
        Len = [Len;len];
        k = k + 1;
        if ~isempty(Ptsk) && ~(sum(Ptsk(1,:)) == 0)
            MPts = [MPts;[Ptsk,repmat(k,size(Ptsk,1),1)]];
        end
    end
end
% drowPts(MPts,'.y',k)

