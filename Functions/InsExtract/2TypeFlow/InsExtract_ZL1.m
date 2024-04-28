function [InsPts,isCable,InsLen] = InsExtract_ZL1(T,L,Loc,GridWidth)
% Special type of cable-stayed cat head tower insulator extraction and processing
InsPts = cell(6,1);% k = 1;
InsLen = zeros(6,1);  isCable = 0;
if size(Loc,1) == 1
    BinXZ1 = BinProjection(T,GridWidth,1,3);
    upimg = BinXZ1(Loc(2):end,:);% drowZbarh(upimg,1);
    [~,LocInd] = maxk(drowZbarh(upimg,1,'Dsum'),2);
    Loc = [min(LocInd)+Loc(2)-1,max(LocInd)+Loc(2)-1;Loc];% drowZbarh(BinXZ,1);
else
    Loc = Loc(end-1:end,:);
end
cross1 = L{1};% drowPts(cross1,'.r')
[cross1R1,theta1] = RotawithAxle(cross1,1);% drowPts(cross1R1,'.r')
BinXZ2 = BinProjection(cross1R1,GridWidth,1,3);% drowZbarh(BinXZ2,1)
ZW = drowZbarh(BinXZ2,1,'wid');% barh(ZW)
FZW = fillBin(ZW);% barh(FZW)
maxW = max(FZW);
midPos = ceil(size(FZW,1) / 2);
if maxW > 12 %The first cross arm is a cable-stayed type
    isCable = 1;
    % -------------------------Split position in z direction
    % drowPts(cross1,'.r')
    % remove noise
    maxV = max(FZW(1:midPos));
    if sum(FZW(1:midPos) == maxV) <= 3
        FZW(FZW(1:midPos) == maxV) = maxV - 1;
    end
    % Calculate threshold
    Thre = max(FZW(1:midPos)) + 1;
    cutPos1 = midPos + find(FZW(midPos+1:end) >= Thre,1,"first") - 1;
    % b = barh(ZW);b.FaceColor = 'flat';b.CData(cutPos1+1,:) = [1 0 0];
    if ~isempty(cutPos1)
        InsPts1R1 = cross1R1(cross1R1(:,3) > min(cross1R1(:,3)) + cutPos1 * GridWidth,:);
    else
        % error("Failed to calculate cut position")
    end
    InsPts1 = InsPts1R1 * rotx(-theta1*180/pi);
    % drowPts(cross1,'.b',InsPts1,'.r')
    % -------------------------Split insulators on power towers
    % end of cross arm
    CT1 = T(T(:,3) > min(T(:,3)) + GridWidth * Loc(2,2) & T(:,3) < min(T(:,3)) + GridWidth * (Loc(1,1)-2) ,:);
    % drowPts(cross1,'.b',CT1,'.r')
    % empty border on both sides
    BinXZ = BinProjection(CT1,GridWidth,1,3);% drowZbarh(BinXZ,-2)
    DWXZ = drowZbarh(BinXZ,-2,'Dsum');% barh(DWXZ)
    midX = floor(size(DWXZ,1)/2);
    cutThre = 10;
    cutPosT1 = find(DWXZ(1:midX) > cutThre,1,"last");
    cutPosT2 = midX + find(DWXZ(midX+1:end) > cutThre,1,"first");
    CT2 = CT1(CT1(:,1) > min(CT1(:,1)) + GridWidth * cutPosT1 & CT1(:,1) < min(CT1(:,1)) + GridWidth * cutPosT2,:);
    % drowPts(CT1,'.g',cross1,'.b',CT2,'.r')
    % Insulator split position
    CT2R1 = CT2 * rotx(theta1*180/pi);
    CT3R1 = CT2R1(CT2R1(:,3) > min(cross1R1(:,3)) + cutPos1 * GridWidth,:);
    CT3 = CT3R1 * rotx(-theta1*180/pi);
    % drowPts(CT1,'.g',cross1,'.b',CT3,'.r')
    % -------------------------Merge insulators on power towers and power lines
    InsPts2 = [InsPts1;CT3];
    labels = dbscan(InsPts2,1,5);
    Ins = InsPts2(labels == MaxLabel(labels),:);% drowPts(InsPts2,'.b',InsPts,'.r')
    Len = max(Ins(:,3)) - min(Ins(:,3));
    InsPts{1} = Ins;
    InsLen(1) = Len;
    k = 2;
else
    k = 1:2;
end


% DC type insulator extraction is the same
for i = k
    CrossLine = [L{i}]; % The power line point at the i-th cross arm % drowPts(CrossLine,'.r')
    labels = dbscan(CrossLine,0.5,1);% drowPts(CrossLine(labels==MaxLabel(labels),:),'.r')
    [~,theta1] = RotawithAxle(CrossLine(labels==MaxLabel(labels),:),3);
    CrossBeg = min(T(:,3)) + GridWidth * (Loc(i,1)-1); %Cross arm starting point Z coordinate
    CrossEnd = min(T(:,3)) + GridWidth * Loc(i,2); %Cross arm end point Z coordinate
    CrossEndPts = T(T(:,3) < CrossEnd & T(:,3) > CrossBeg - 0.2,:);
    CrossEndPtsR = CrossEndPts * rotz(theta1*180/pi);% drowPts(TowerPts*rotz(theta1*180/pi),'.b',CrossEndPtsR,'.r')
    % CrossEndZ = fitPlane3(CrossEndPtsR);
    CrossLineR3 = CrossLine*rotz(theta1*180/pi);
    % TowerPtsR = TowerPts * rotz(theta1*180/pi);
    thirdLen = (max(CrossEndPtsR(:,2)) - min(CrossEndPtsR(:,2))) / 3;
    % drowPts(TowerPtsR,'.b',CrossLineR3,'.r')
    %     % Draw Z plane
    %     scatter3(TowerPts(:, 1), TowerPts(:, 2), TowerPts(:, 3), '.r');hold on;
    %     [X,Y] = meshgrid(min(TowerPts(:,1)):0.5:max(TowerPts(:,1)),min(TowerPts(:,2)):0.5:max(TowerPts(:,2)));
    %     Z = repmat(CrossEnd,[size(X),1]);
    %     mesh(X, Y, Z);axis equal
    InsInTower = T(T(:,3) < CrossBeg,:);
    % drowPts(CrossLine,'.g',InsInTower,'.r')
    % -----------------Extract the insulators from two power lines respectively
    % YMid = min(TowerPts(:,2)) + (max(TowerPts(:,2)) - min(TowerPts(:,2)))/2;
    for j = 1:3
        thirdLenPts = CrossLineR3(CrossLineR3(:,2) >= min(CrossEndPtsR(:,2)) + (j-1)*thirdLen &...
            CrossLineR3(:,2) <= min(CrossEndPtsR(:,2)) + j * thirdLen ...
            ,:) * rotz(-theta1*180/pi);
        % drowPts(CrossLine,'.b',thirdLenPts,'.r')
        [Ins,Len] = InsExtrat_Partone(thirdLenPts,InsInTower,GridWidth);
        % drowPts(TowerPts,'.b',thirdLenPts,'.g',Ins,'.r')
        % InsPts = [InsPts;[Ins,repmat(k,[size(Ins,1),1])]]; k = k+1;
        InsPts{(i-1)*3+j} = Ins;
        InsLen((i-1)*3+j) = Len;
    end
end
%     drowPts(TowerPts,'.y',LinePts{1},'.b',InsPts(:,1:3),'.r');
%     drowPts(TowerPts,'.y',[LinePts{1};LinePts{2};LinePts{3}],'.b',InsPts(:,1:3),'.r');
end

function [Ins,Len] = InsExtrat_Partone(Line,Tower,GridWidth)
% Extracting insulators from individual power lines drowPts(Tower,'.b',Line,'.r')
    Ins = zeros(0,3);
    Len = 0;
    if size(Line,1) < 5
        return
    end
    [LineR3,Theta1] = RotawithAxle(Line,3);% drowPts(LineR3,'.r')
    [LineR32,Theta2] = RotawithAxle(LineR3,2);% drowPts(LineR32,'.r')
    BinXY = BinProjection(LineR32,GridWidth,1,3);% drowZbarh(BinXY,1)
    cutPos = getCutpos1(BinXY);
    if ~isempty(cutPos)
        InsLR = LineR32(LineR32(:,3) > min(LineR32(:,3))+cutPos*GridWidth,:);
        % drowPts(LineR32,'.b',InsR,'.r')
        % Extracting point clouds of insulators on power towers
        TowerR = Tower*rotz(Theta1*180/pi)*roty(Theta2*180/pi);
        InsTR = TowerR(TowerR(:,1) < max(LineR32(:,1)) & TowerR(:,1) > min(LineR32(:,1)) ...
            & TowerR(:,2) < max(LineR32(:,2)) & TowerR(:,2) > min(LineR32(:,2)) ...
            & TowerR(:,3) > min(InsLR(:,3)),:);
        % drowPts(TowerR,'.b',InsT,'.r')
        InsR = [InsLR;InsTR];
        Len = max(InsR(:,3)) - min(InsR(:,3));
        Ins = InsR * roty(-Theta2*180/pi) * rotz(-Theta1*180/pi);
    else
        error("Failed to get cropping position")
    end

end


function cutPos = getCutpos1(BinImg)
% Using hole detection to crop locations

% drowZbarh(BinImg,1)
Epy = drowZbarh(BinImg,1,'epy');
% Mark rows with hole 0
Epy0 = Epy;
Epy0(Epy0 ~= 0) = -1;
Epy0(Epy0 == 0) = 1;
Epy0(Epy0 == -1) = 0;
% Find the starting and ending positions of the interval
DEpy0 = diff(Epy0);
Beg = find(DEpy0 == 1); % Starting hole interval index
% The nearest interval above the position with the largest density change
[~,Dwid_max_ind] = max(drowZbarh(BinImg,1,'Dsum')); 
CutPosInd = find(Beg - Dwid_max_ind >= 0,1,"first");
cutPos = Beg(CutPosInd);

% DWid = drowZbarh(BinXY,1,'Dwid');
% cutThre = mean(DWid(ceil(length(DWid)/5):end)) * 2;
% cutPos = find(DWid(1:ceil(length(DWid)/5*4)) > cutThre,1,"last") + 1;
end


function Loc = mergeLoc(Loc,crossNum)
    while size(Loc,1) > crossNum
        gap = Loc(1:end-1,2) - Loc(2:end,1);
        [~,mingapInd] = min(gap);
        Loc(mingapInd,1) = Loc(mingapInd + 1,1);
        Loc(mingapInd + 1,:) = [];
    end
end