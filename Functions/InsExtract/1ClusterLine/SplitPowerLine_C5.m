function CrossLine = SplitPowerLine_C5(Line,GetNum,dist)
%SPLITPOWERLINE_C Separate lightning protection wire
% -------------clustering drowPts(Line,'.r')
labels = dbscan(Line,dist,2);
labelsUni = unique(labels);
cNum = length(labelsUni);  % Number of clusters
% drowPts([Line,labels],cNum)
% -------------Traverse the clusters, separate the left and right clusters and calculate the characteristic parameters of each cluster
%% Merge clusters sequentially based on cluster height differences
% -------------Count the highest elevation point of each cluster
ZMid = zeros(cNum,1);
for i = 1:cNum
    ClusterPts = Line(labels == labelsUni(i),:);
    ZMid(i) = min(ClusterPts(:,3)) + (max(ClusterPts(:,3)) - min(ClusterPts(:,3))) / 2;
end
%% Number clusters in order
newlabels = zeros(size(labels));
for i = 1:cNum
    newlabels(labels == labelsUni(i)) = i;
end% drowPts([Line,newlabels],cNum)
%% Delete the highest GetNum clusters
deLabels = [];
for i = 1:GetNum
    [~,maxLabel] = max(ZMid);
    % drowPts(Line,'.b',Line(newlabels == maxLabel,:),'.r');
    deLabels = [deLabels;find(newlabels == maxLabel)];
    ZMid(maxLabel) = min(ZMid);
end
CrossLine = Line;
CrossLine(deLabels,:) = [];

