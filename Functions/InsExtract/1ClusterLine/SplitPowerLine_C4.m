function CrossLine = SplitPowerLine_C4(Line,GetNum,dist)
%SPLITPOWERLINE_C Separating power line point clouds using clustering methods
% -------------clustering drowPts(Line,'.r')
labels = dbscan(Line,dist,2);
Line(labels == -1,:) = [];
labels(labels == -1) = [];
labelsUni = unique(labels);
cNum = length(labelsUni);  % Number of clusters
% drowPts([Line,labels],cNum)
% -------------Traverse the clusters, separate the left and right clusters and calculate the characteristic parameters of each cluster
%% Merge clusters sequentially based on cluster height differences
% -------------Count the elevation midpoint of each cluster
ZMid = zeros(cNum,1);
for i = 1:cNum
    ClusterPts = Line(labels == labelsUni(i),:);
    ZMid(i) = min(ClusterPts(:,3)) + (max(ClusterPts(:,3)) - min(ClusterPts(:,3))) / 2;
end
%% Number clusters in order
newlabels = zeros(size(labels));
for i = 1:cNum
    newlabels(labels == labelsUni(i)) = i;
end
%% merge clusters
[ZMid_sort,sortInd1] = sort(ZMid);
[~,cutPos] = maxk(abs(diff(ZMid_sort)),GetNum);
cSection = [0;sort(cutPos);cNum];
CrossLine = cell(GetNum,1);
for i = 1:GetNum
    Indi = sortInd1(cSection(i)+1:cSection(i+1));
    CrossLinei = zeros(0,3);
    for j = 1:size(Indi,1)
        CrossLinei = [CrossLinei;Line(newlabels==Indi(j),:)];
    end
    % drowPts(Line,'.b',CrossLinei,'.r')
    CrossLine{GetNum - (i - 1)} = CrossLinei;
end