function TypeID = TypeDetect(TowerPts,LinePts)
% TowerPts = importdata(Tfilename);% drowPts(TowerPts,'.r')
%% Redirect the power tower point cloud (noise points may exist above the power tower point cloud)
[TopZ,TopInd] = maxk(TowerPts(:,3),10); % There will be no more than 10 noise points
cutInd = TopInd(find(diff(TopZ) < -1,1,'last')+1); % drowPts(TowerPts(TopInd,:),'.b',TowerPts(cutInd,:),'.r')
if ~isempty(cutInd)
    TowerPts = TowerPts(TowerPts(:,3) <= TowerPts(cutInd,3),:);% drowPts(cutPts,'.r')
end
% Intercept the 3m point cloud on the power tower to calculate the main direction
[~,Theta] = RotawithAxle(TowerPts(TowerPts(:,3)>max(TowerPts(:,3))-3,:),3);
TowerPtsR = TowerPts*rotz(Theta*180/pi);% drowPts(TowerPtsR,'.r')
%% Read power line
LinePtsRCut = LinePts * rotz(Theta*180/pi); % drowPts(TowerPtsR,'.b',LinePtsR,'.r')
% drowPts(TowerPtsR,'.b',LinePtsRCut,'.r')
%% Calculate the maximum projected cavity area and cross arm position
GridWidth1 = 0.2;
GridWidth2 = 0.1;
BinYZ_O = BinProjection(TowerPtsR,GridWidth1,1,3); %Void detection imshow(flipud(BinYZ_O))
BinYZ_C = BinProjection(TowerPtsR,GridWidth2,1,3); %Cross arm inspection imshow(flipud(BinYZ_C))
Loc = CrossLocation(BinYZ_C,4); %Detect cross arm position, density histogram, width histogram
%% Electric tower type identification and suspension point detection
GanNum = size(Loc,1); %Number of cross arms
if OTowerDetect(BinYZ_O,1/2) %Detect cavities in the upper half
    if GanNum == 1 %O-shaped tower, 1 cross arm -> wine glass tower
        TypeID = 1;
    else %O-shaped tower, multiple cross arms -> cat head tower
        TypeID = 2;
    end
elseif GanNum == 1 && OTowerDetect(BinYZ_O,1/3) %Expanded range detection of wine glass towers
    TypeID = 1;
elseif GanNum == 2% double cross arm
    if OTowerDetect(BinYZ_O,1/2,500) % The cat head tower with missing data may not be stable on other lines.
        TypeID = 2;
    else % dry font
        TypeID = 4;
    end
elseif  GanNum == 1% double cross arm
    TypeID = 3;
else% Multiple cross arms, drum-shaped tower. Continue to determine whether it is a tension type or a DC type
        % drowPts(TowerPtsR,'.b',LinePtsRCut,'.r')
    labels = dbscan(LinePtsRCut,1,1);% drowPts([LinePtsRCut,labels],0)
    CrossLine = LinePtsRCut(labels == MaxLabel(labels),:);% drowPts(CrossLine,'.r')
    % Use the void in the vertical axis direction of the binary image to determine the type of electric tower
    BinXZ = binPro(CrossLine,GridWidth2,2,3);
    % drowZbarh(BinXZ,1)
    ZF = drowZbarh(BinXZ,-2,'epy');% mean(ZF)
    if mean(ZF) > 4 % Counted by pixel, 10 pixels is 1m
        TypeID = 5; % Tension type
    else
        TypeID = 6;
    end
end
% Portal tower cannot be recognized and needs to be changed manually