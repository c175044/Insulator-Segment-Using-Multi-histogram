function [fleft,fright] = fitPlane2(points)
%FITGROWTHPLANE 
% drowPts(points,'.r')
% Extract cross arm boundary points
PlanePts = points(:,[2,3]);
pts = PlanePts(boundary(PlanePts),:);
% plot(PlanePts(:,1),PlanePts(:,2),'.b',pts(:,1),pts(:,2),'.r');axis equal;
% --------------------------------Delete upper and lower boundary points---------------------------
samGap = 0.2;
ptsD = sinPro(pts,2,samGap); %Project to Y axis barh(ptsD)
[~,cutInd1] = max(ptsD(1:ceil(length(ptsD)/2)));
[~,cutInd2] = max(ptsD(ceil(length(ptsD)/2)+1:end));
cutPos1 = min(pts(:,2)) + cutInd1 * samGap;
cutPos2 = min(pts(:,2)) + (ceil(length(ptsD)/2) + cutInd2 - 1) * samGap;
ptsC = pts;
ptsC(pts(:,2) < cutPos1 | pts(:,2) > cutPos2,:) = [];
% plot(PlanePts(:,1),PlanePts(:,2),'.y',pts(:,1),pts(:,2),'.b',ptsC(:,1),ptsC(:,2),'.r');axis equal;
% -----------------------------Fit the left and right straight lines separately------------------------
midX = min(ptsC(:,1)) + (max(ptsC(:,1)) - min(ptsC(:,1))) / 2;
% left fitting point
fleft = RanSaC_Fitline(ptsC(ptsC(:,1) < midX,:),10000,samGap);
% Right fitting point
fright = RanSaC_Fitline(ptsC(ptsC(:,1) > midX,:),10000,samGap);
% Drawing
% plot(PlanePts(:,1),PlanePts(:,2),'.y',pts(:,1),pts(:,2),'.b',ptsC(:,1),ptsC(:,2),'.r');hold on
% y = min(ptsC(:,2)):0.1:max(ptsC(:,2));
% xleft = (y - fleft(2)) ./ fleft(1);
% xright = (y - fright(2)) ./ fright(1);
% plot(xleft,y,'-g',xright,y,'-g','LineWidth',2);
% axis equal
end