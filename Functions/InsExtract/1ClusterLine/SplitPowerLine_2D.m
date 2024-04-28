function CrossLine = SplitPowerLine_2D(LinePts,GridWidth,GetNum,Eps)
%SPLITPOWERLINE_2D 
% idx = dbscan(LinePts(:,[1,3]), 1, 1);
% drowPts([LinePts,idx],length(unique(idx)))

% Clustering point clouds on a 2D projection surface
[BinXZ,Grid] = BinProjection(LinePts,GridWidth,2,3);% drowZbarh(BinXZ,1)
% Clustering using the dbscan function
[posY,posX] = find(BinXZ);% plot(posX,posY,'.r');axis equal
idx = dbscan([posX,posY], Eps, 1);
% drowPts([posX,posY,ones(length(posY),1),idx],length(unique(idx)))

% Calculate the lowest point coordinates of each cluster
labels = unique(idx);
labelsNum = length(labels);
MinCor = zeros(labelsNum,1);
for i = 1:labelsNum
    MinCor(i) = min(posY(idx==labels(i)));
end
[~,MinInd] = sort(MinCor,'ascend');

% Extract the lowest GetNum clusters
CrossLine = cell(1,GetNum);
for i = 1:GetNum %The top one is the first cross arm
    GridPY = posY(idx==labels(MinInd(i)));% grid row coordinates
    GridPX = posX(idx==labels(MinInd(i)));% grid column coordinates
    CurLineInd = [];
    for j = 1:length(GridPY)
        CurLineInd = [CurLineInd,Grid{GridPY(j),GridPX(j)}];
    end
    CrossLine{GetNum - i + 1} = LinePts(CurLineInd,:);% drowPts(LinePts(CurLineInd,:),'.r')
end
% drowPts(LinePts,'.y',[CrossLine{1}],'.r',[CrossLine{2}],'.g',[CrossLine{3}],'.b')

end


