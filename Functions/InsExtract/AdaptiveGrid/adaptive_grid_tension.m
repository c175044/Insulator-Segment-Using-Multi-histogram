function [fine_Ins,fine_Len,fine_index,fine_Ve] = adaptive_grid_tension(AllIns, AllLen)

InsNum = size(AllIns,1);
GridNum = size(AllIns,2);
% Calculate the verticality of each insulator
VeI = zeros(InsNum,GridNum);
for i = 1:InsNum
    for j = 1:GridNum
        try
            VeI(i,j) = calcuV(AllIns{i,j});
        catch
            % No insulators were extracted under this width
        end
    end
end
% Determine transverse insulators and longitudinal insulators
XYInsInd = sum(VeI < 0.8 & VeI > 0 & AllLen > 1,2) - sum(VeI > 0,2) / 2 >= 0;
% Calculate final grid width
[~,XYInd] = min(abs(AllLen - mean(AllLen(VeI(:) <  0.8 & AllLen(:) > 2.5))),[],2);
[~,ZInd]  = min(abs(AllLen - mean(AllLen(VeI(:) >= 0.8 & AllLen(:) < 2.5))),[],2);
ResInd = XYInd; ResInd(~XYInsInd) = ZInd(~XYInsInd);
% -------------Extract final results
% length
fine_Len = AllLen(sub2ind(size(AllLen),(1:size(AllLen,1))',ResInd));
% Grid width
fine_index = ResInd;
% Verticality
fine_Ve = VeI(sub2ind(size(VeI),(1:size(VeI,1))',ResInd));
% insulator point
fine_Ins = zeros(0,4);
label = 1;
for i = 1:InsNum
    InsCur = AllIns{i,fine_index(i)};
    if size(InsCur,1) > 1
        fine_Ins = [fine_Ins;InsCur,repmat(label,size(InsCur,1),1)];
        label = label + 1;
    end
end
end

