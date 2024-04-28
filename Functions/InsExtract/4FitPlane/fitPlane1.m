function [fleft,fright] = fitPlane1(points,Type)
%FITGROWTHPLANE 
switch Type
    case {3,5}
        isCutMid = true;
    case {1,4}
        isCutMid = false;
    otherwise
        error("Undefined tower type")
end
% plot(points(:,1),points(:,2),'.r');hold on;axis equal
[pointsRZ,theta3] = RotawithAxle(points,3);% drowPts(points,'.r')
[pointsRZY,theta2] = RotawithAxle(pointsRZ,2);% drowPts(pointsRZY,'.r')
pointsRZY = pointsRZY(:,[2,1,3]);
% Compute boundaries
K = boundary(pointsRZY(:,[1,2])); %Extract boundary points
pointsB = pointsRZY(K,:);
%  subplot(1,2,1)
%  plot(pointsRZY(:,1),pointsRZY(:,2),'.b',pointsB(:,1),pointsB(:,2),'.r','Markersize',1);axis equal
%  hold on;
%  for dx = min(pointsB(:,1)):0.1:max(pointsB(:,1))
%      plot([dx,dx],[min(pointsB(:,2)),max(pointsB(:,2))],'-k','Markersize',0.5)
%  end
% %  f = gcf;
% %  f.Position(3) = f.Position(3) * 0.1;
% 
%  subplot(1,2,2)
%  wid = ceil(max(pointsB(:,1)) - min(pointsB(:,1))) ./ 0.1;
%  % width histogram
%  ptsInWInd = floor((pointsB(:,1)-min(pointsB(:,1))) ./ 0.1) + 1; %index of each point
%  widHisto = zeros(wid,1);
%  for i = 1:length(pointsB)
%      widHisto(ptsInWInd(i)) = widHisto(ptsInWInd(i))+1;
%  end
%  b = bar(widHisto);
%  b.FaceColor = 'flat';
%  b.CData(2,:) = [1 0 0];
%  b.CData(3,:) = [1 0 0];
%  b.CData(45,:) = [1 0 0];
% Cut off the middle part of the Y axis
% dY = max(pointsB(:,2)) - min(pointsB(:,2));
% midY = min(pointsB(:,2)) + dY / 2;
% pointsB(pointsB(:,2) < midY + dY/4 & pointsB(:,2) > midY - dY/4,:) = [];
% Translate to coordinate origin
pts(:,1) = pointsB(:,1) - min(pointsB(:,1));
pts(:,2) = pointsB(:,2) - min(pointsB(:,2));
pts(:,3) = pointsB(:,3) - min(pointsB(:,3));
% drowPts(pts,'.r') 
% -----------------------------Fit the left and right straight lines separately------------------------
samGap = 0.1;
midX = min(pts(:,1)) + (max(pts(:,1)) - min(pts(:,1))) / 2;
% left fitting point
ptsLInd = find(pts(:,1) < midX);
ptsLeftR = ExtraFitPts(ptsLInd,pts,pointsB,samGap,isCutMid); %Need to return the original point according to the index (can be improved to reduce input parameters)
% plot(pointsB(:,1),pointsB(:,2),'.b',ptsLeftR(:,1),ptsLeftR(:,2),'.r');axis equal
ptsLeft = ptsLeftR(:,[2,1,3]) * roty(-theta2*180/pi) * rotz(-theta3*180/pi); 
% plot(points(:,1),points(:,2),'.b',ptsLeft(:,1),ptsLeft(:,2),'.r');axis equal
fleft = RanSaC_Fitline(ptsLeft(:,[1,2]),10000,samGap);
% Right fitting point
ptsRInd = find(pts(:,1) >= midX);% drowPts(pts,'.b',pts(ptsRInd,:),'.r') 
ptsRightR = ExtraFitPts(ptsRInd,pts,pointsB,samGap,isCutMid);
ptsRight = ptsRightR(:,[2,1,3]) * roty(-theta2*180/pi) * rotz(-theta3*180/pi); 
% plot(ptsRight(:,1),ptsRight(:,2),'.r');axis equal
fright = RanSaC_Fitline(ptsRight(:,[1,2]),10000,samGap);
%% Drawing
% plot(points(:,1),points(:,2),'.b');hold on;axis equal
% % Draw separated left and right points
% plot(ptsLeft(:,1),ptsLeft(:,2),'.r');
% plot(ptsRight(:,1),ptsRight(:,2),'.r');
% % Draw the left straight line fitting results
% xleft = min(points(:,1)):0.1:max(points(:,1));
% yleft = xleft * fleft(1) + fleft(2);
% plot(xleft,yleft,'-g','LineWidth',1);
% % Draw the right straight line fitting results
% xright = min(points(:,1)):0.1:max(points(:,1));
% yright = xright * fright(1) + fright(2);
% plot(xright,yright,'-g','LineWidth',1);


% Set the straight line with the smaller x coordinate as the left fitting straight line
if mean(ptsLeft(:,2)) > mean(ptsRight(:,2))
    Tepf = fleft;
    fleft = fright;
    fright = Tepf;
end
end
%% ---------------------------Extract the point function to be fitted---------------------------
function HalfPts = ExtraFitPts(HalfInd,pts,points,samGap,isCutMid)
ptsHalf = pts(HalfInd,:);
% Redirect the point cloud once (perform histogram filtering of fitting points based on the redirected point cloud)
ptsR = RotaPts(ptsHalf); % drowPts(ptsHalf,'.b',ptsR,'.r')
ptsRMove(:,1) = ptsR(:,1) - min(ptsR(:,1));
ptsRMove(:,2) = ptsR(:,2) - min(ptsR(:,2));
ptsRMove(:,3) = ptsR(:,3) - min(ptsR(:,3));
% Cut off the middle part
if isCutMid
    Ylen = max(ptsRMove(:,2)) - min(ptsRMove(:,2));
    ptsRMoveOri = ptsRMove;
    OutPtsInd = find(ptsRMove(:,2) <= min(ptsRMove(:,2) + Ylen / 4) | ...
        ptsRMove(:,2) >= max(ptsRMove(:,2) - Ylen / 4));
    ptsRMove = ptsRMove(OutPtsInd,:);
end
% drowPts(ptsRMoveOri,'.b',ptsRMove,'.r')
% -------Project to the X-axis

% width histogram
[widHisto,ptsInWInd] = sinPro(ptsRMove,1,samGap);
% ptsInWInd = floor(ptsRMove(:,1) - min(ptsRMove(:,1)) ./ samGap) + 1; %index of each point
% wid = ceil(max(ptsRMove(:,1)) - min(ptsRMove(:,1))) ./ samGap;
% widHisto = zeros(wid,1);
% for i = 1:size(ptsRMove,1)
%     widHisto(ptsInWInd(i)) = widHisto(ptsInWInd(i))+1;
% end
% widHistonew = histcounts(ptsInWInd,1:wid);
% bar(widHisto)
% -------Filter the points of the fitted straight line based on the projection density
% Extract fitting points
[~,maxInd] = max(widHisto);% barh(widHisto(1:floor(wid / 2)))
SRange = 1; %Take all histograms of a certain height as points to be fitted
tepInd = maxInd-SRange:maxInd+SRange;
lenDgdeInd = [];
for i = 1:length(tepInd)
    lenDgdeInd = [lenDgdeInd;find(ptsInWInd==tepInd(i))];
end
if isCutMid
    DgdeInd = HalfInd(OutPtsInd(lenDgdeInd)); %Calculate the index to the original point
else
    DgdeInd = HalfInd(lenDgdeInd);
end
HalfPts = points(DgdeInd,:); %Extract points based on index
% drowPts(points,'.b',HalfPts,'.r')
% ----------------Check whether it contains both upper and lower points, if not, expand the search range-------------
YMid = min(points(:,2)) + (max(points(:,2)) - min(points(:,2)))/2;
upPtsLen = sum(HalfPts(:,2) > YMid);
dwPtsLen = sum(HalfPts(:,2) < YMid);
isOK = true;
if upPtsLen == 0 || dwPtsLen == 0
    isOK = false;
end
% Expand the scope of
while ~isOK && SRange < length(widHisto) / 3
    SRange = SRange + 1;
    % Expand to the left
    tepIndL = maxInd-SRange:maxInd;
    lenDgdeIndL = [];
    for i = 1:length(tepIndL)
        lenDgdeIndL = [lenDgdeIndL;find(ptsInWInd == tepIndL(i))];
    end
    DgdeIndL = HalfInd(lenDgdeIndL); %Calculate the index to the original point
    HalfPtsL = points(DgdeIndL,:); %Extract points based on index drowPts(points,'.b',HalfPtsL,'.r')
    % Check upper and lower points
    upPtsLenL = sum(HalfPtsL(:,2) > YMid);
    dwPtsLenL = sum(HalfPtsL(:,2) < YMid);
    if upPtsLenL > 1 && dwPtsLenL > 1 %To meet the conditions
        isOK = true;
    end
    % Expand to the right
    tepIndR = maxInd:maxInd+SRange;
    lenDgdeIndR = [];
    for i = 1:length(tepIndR)
        lenDgdeIndR = [lenDgdeIndR;find(ptsInWInd == tepIndR(i))];
    end
    DgdeIndR = HalfInd(lenDgdeIndR); %Calculate the index to the original point
    HalfPtsR = points(DgdeIndR,:); %Extract points based on index  drowPts(points,'.b',HalfPtsR,'.r')
    % Check upper and lower points
    upPtsLenR = sum(HalfPtsR(:,2) > YMid);
    dwPtsLenR = sum(HalfPtsR(:,2) < YMid);
    if upPtsLenR > 1 && dwPtsLenR > 1 %also meet the conditions
        if isOK %You can expand both the left and right sides, choose the side with more points.
            if size(HalfPtsR,1) >= size(HalfPtsL,1)
                HalfPts = HalfPtsR;
            else
                HalfPts = HalfPtsL;
            end
        else %Can only expand to the right
            isOK = true;
            HalfPts = HalfPtsR;
        end
    elseif isOK %Can only expand to the left
        HalfPts = HalfPtsL;
    end
end
end

%% -----------------------------Redirect point cloud function-----------------------------
function [PtsR,Angle] = RotaPts(Pts)
% RotawithAxle An improved version of the function, rotating around the z-axis so that the point cloud is parallel to the y-axis
% Calculate direction vector using 2D points
Pts2D = Pts(:,[1,2]);
Pts2DDownPC = pcdownsample(pointCloud([Pts2D,zeros(length(Pts),1)]),'gridAverage',0.2);
Pts2DDown = Pts2DDownPC.Location(:,1:2);
center = mean(Pts2DDown); %Plane center point
M = Pts2DDown - repmat(center,[size(Pts2DDown,1),1]); %coordinate difference
MM = (M' * M)/size(Pts2DDown,1); %covariance matrix
[V,~] = eig(MM); 
Dire_vector = V(:,1); %direction vector of point cloud
% Rotate points
% plot(Pts2D(:,1),Pts2D(:,2),'.r');hold on
% plot([0,Dire_vector(1)],[0,Dire_vector(2)],'-r');
% axis equal
Angle = acos(abs(Dire_vector(1))/norm(Dire_vector)); %The angle between the direction vector and the projected x-axis
if Dire_vector(1) * Dire_vector(2) < 0 %clockwise from y to x
    Angle = -Angle;
end
PtsR = Pts*rotz(Angle*180/pi);


end

%% -----------------------------straight line fitting function-----------------------------
function bestline = RanSaC_Fitline(pts,Iter,t)
% Fitting 4 vertices of a two-dimensional planar rectangle
% Input parameters: points, Iter (number of sampling), 
% t (distance threshold from point to straight line)
% Output parameters: bestline (plane straight line parameter)

% figure;plot3(points(1,:),points(2,:),points(3,:),'.');hold on; % 
points(:,1) = pts(:,1) - min(pts(:,1));
points(:,2) = pts(:,2) - min(pts(:,2));
maxNum = 0; %The number of optimal fitting interior points
for j = 1:Iter %Cycles
    midY = min(points(:,2)) + (max(points(:,2)) - min(points(:,2))) / 2; %Y midpoint coordinate
    % Randomly sample fitting points (lower part)
    DPts = points(points(:,2) < midY,:); %Points in the lower half
    UPts = points(points(:,2) > midY,:); %The number of points in the upper half (according to the input rules, it is impossible to have points equal to minY)
    sampleNum = min(ceil(size(DPts,1) / 3),ceil(size(UPts,1) / 3)); %The number of sampling points in the upper and lower halves
    sampleidxD = randperm(size(DPts,1),sampleNum); % Index of random sampling points in the lower half    
    sampleidxU = randperm(size(UPts,1),sampleNum); % Index of random sampling points in the upper half
    samPts = [DPts(sampleidxD,:);UPts(sampleidxU,:)];
%     plot(points(:,1),points(:,2),'.r',samPts(:,1),samPts(:,2),'.b');axis equal;
    % preliminary line
    f = polyfit(samPts(:,1),samPts(:,2),1);
    distance = abs(f(1)*points(:,1) - points(:,2) + f(2)) ./ sqrt(f(1)^2 + 1); %The distance from each point to the straight line
    % Number of points near the straight line
    nearSum = sum(distance < t);
    % Combine nearby points to fit a straight line
    if nearSum > sampleNum
        nearPts = points(distance < t,:);
        f = polyfit(nearPts(:,1),nearPts(:,2),1);
        distance = abs(f(1)*points(:,1) - points(:,2) + f(2)) ./ sqrt(f(1)^2 + 1);
        nearSum = sum(distance < t); %Update point count
    end
    % Update optimal planes and proportions
    if nearSum > maxNum 
        finDist = distance;
        maxNum = nearSum;
    end
end
bestpts = pts(finDist < t,:); 
bestline =  polyfit(bestpts(:,1),bestpts(:,2),1);
if abs(bestline(1)) < 20
    midY = min(bestpts(:,2)) + (max(bestpts(:,2)) - min(bestpts(:,2))) / 2;
    DPts = bestpts(bestpts(:,2) < midY,:); %Points in the lower half
    UPts = bestpts(bestpts(:,2) > midY,:); %The number of points in the upper half (according to the input rules, it is impossible to have points equal to minY)
    sampleNum = min(ceil(size(DPts,1) / 3),ceil(size(UPts,1) / 3)); %The number of sampling points in the upper and lower halves
    sampleidxD = randperm(size(DPts,1),sampleNum); % Index of random sampling points in the lower half    
    sampleidxU = randperm(size(UPts,1),sampleNum); % Index of random sampling points in the upper half
    bestpts = [DPts(sampleidxD,:);UPts(sampleidxU,:)];
    bestline = polyfit(bestpts(:,1),bestpts(:,2),1);
end
% Plot the fitting results
% plot(pts(:,1),pts(:,2),'.b',bestpts(:,1),bestpts(:,2),'.r');hold on
% yright = min(pts(:,2)):0.1:max(pts(:,2));
% xright = (yright - bestline(2)) ./ bestline(1);
% plot(xright,yright,'-g','LineWidth',1);axis equal;
end