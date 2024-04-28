function [TowerPtsR,Theta] = RTower(TowerPts)
%RTOWER 
% Denoising (noise points may exist above the power tower point cloud)
[TopZ,TopInd] = maxk(TowerPts(:,3),10); % There will be no more than 10 noise points
cutInd = TopInd(find(diff(TopZ) < -1,1,'last')+1); % drowPts(TowerPts(TopInd,:),'.b',TowerPts(cutInd,:),'.r')
if ~isempty(cutInd)
    TowerPts = TowerPts(TowerPts(:,3) <= TowerPts(cutInd,3),:);% drowPts(cutPts,'.r')
end
% Intercept the 3m point cloud on the power tower to calculate the main direction
[~,Theta] = RotawithAxle(TowerPts(TowerPts(:,3)>max(TowerPts(:,3))-3,:),3);
TowerPtsR = TowerPts*rotz(Theta*180/pi);% drowPts(TowerPtsR,'.r')
end
