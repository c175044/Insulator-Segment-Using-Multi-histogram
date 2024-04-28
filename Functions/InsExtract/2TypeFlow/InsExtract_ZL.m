function [InsPts,InsLen] = InsExtract_ZL(TowerPts,LinePts,Loc,GridWidth,Type)
% drowPts(TowerPts,'.b',[LinePts{1}],'.r')
switch Type
    case {1,8}
        crossNum = 1;
        Loc = Loc(end,:);
    case 2
        crossNum = 2;
        if size(Loc,1) ~= crossNum
            Loc = mergeLoc(Loc,crossNum);
        end
    case 6
        Loc = Loc(end-2:end,:);
        crossNum = 3;
    otherwise
        error("undefined projection type")
end
InsPts = cell(3 * crossNum,1);
InsPts = cellfun(@(x) [0,0,0], InsPts, 'UniformOutput', false);
InsLen = zeros(3 * crossNum,1);
for i = 1:crossNum
    CrossLine = [LinePts{i}]; % The power line point at the i-th cross arm % drowPts(CrossLine,'.r')
    % Calculate the main transmission direction
    labels = dbscan(CrossLine,0.5,1);% drowPts(CrossLine(labels==MaxLabel(labels),:),'.r')
    [~,theta1] = RotawithAxle(CrossLine(labels==MaxLabel(labels),:),3);
    % Extract cross arm points and rotate
    CrossBeg = min(TowerPts(:,3)) + GridWidth * Loc(i,1); %Cross arm starting point Z coordinate
    CrossEnd = min(TowerPts(:,3)) + GridWidth * Loc(i,2); %Cross arm end point Z coordinate
    CrossEndPts = TowerPts(TowerPts(:,3) < CrossEnd & TowerPts(:,3) > CrossBeg - 3 * GridWidth,:);
    CrossEndPtsR = CrossEndPts * rotz(theta1*180/pi);% drowPts(TowerPts*rotz(theta1*180/pi),'.b',CrossEndPtsR,'.r')
    % Use cross arm ends to cut poles and power lines
    InsInTower = TowerPts(TowerPts(:,3) < CrossBeg,:);
    CrossLineC = CrossLine(CrossLine(:,3) < CrossBeg,:);
    % Draw Z plane
%     scatter3(TowerPts(:, 1), TowerPts(:, 2), TowerPts(:, 3), '.g');hold on;axis equal
%     scatter3(CrossLine(:, 1), CrossLine(:, 2), CrossLine(:, 3), '.c');
%     scatter3(InsInTower(:, 1), InsInTower(:, 2), InsInTower(:, 3), '.b');
%     scatter3(CrossLineC(:, 1), CrossLineC(:, 2), CrossLineC(:, 3), '.r');
%     [X,Y] = meshgrid(min(TowerPts(:,1)):0.5:max(TowerPts(:,1)),min(TowerPts(:,2)):0.5:max(TowerPts(:,2)));
%     Z = repmat(CrossBeg,[size(X),1]); 
%     mesh(X, Y, Z);
    % -----------------Extract insulators from power lines separately
    % Distinguish between left, middle and right three-phase power lines
    CrossLineCR3 = CrossLineC * rotz(theta1*180/pi);
    thiedcell = cell(3,1);
    thirdLen = (max(CrossEndPtsR(:,2)) - min(CrossEndPtsR(:,2))) / 3;
    thiedcell{1} = CrossLineCR3(CrossLineCR3(:,2) < min(CrossEndPtsR(:,2)) + thirdLen,:);
    thiedcell{2} = CrossLineCR3(CrossLineCR3(:,2) >= min(CrossEndPtsR(:,2)) + thirdLen &...
            CrossLineCR3(:,2) < min(CrossEndPtsR(:,2)) + 2 * thirdLen,:);
    thiedcell{3} = CrossLineCR3(CrossLineCR3(:,2) > min(CrossEndPtsR(:,2)) + 2 * thirdLen,:);
    for j = 1:3
        thirdLenPts = thiedcell{j} * rotz(-theta1*180/pi);
        % drowPts(CrossLine,'.g',InsInTower,'.b',thirdLenPts,'.r')
        if size(thirdLenPts,1) < 5
            continue
        end
        [Ins,Len] = InsExtrat_Partone(thirdLenPts,InsInTower,GridWidth);
        % drowPts(TowerPts,'.b',thirdLenPts,'.g',Ins,'.r')
        if ~isempty(Ins)
            InsPts{(i-1)*3+j} = Ins;
            InsLen((i-1)*3+j) = Len;
        else
        end
    end
end
end

function [Ins,Len] = InsExtrat_Partone(Line,Tower,GridWidth)
% Extracting insulators from individual power lines drowPts(Tower,'.b',Line,'.r')
    % PCSHOW2(Tower * rotz(Theta1*180/pi) * roty(Theta2*180/pi),LineR32)
    Ins = zeros(0,3);Len = 0;
    [LineR3,Theta1] = RotawithAxle(Line,3);% drowPts(LineR3,'.r')
    [LineR32,Theta2] = RotawithAxle(LineR3,2);% drowPts(LineR32,'.r')
    BinXY = BinProjection(LineR32,GridWidth,1,3);% drowZbarh(BinXY,1)
    cutPos = getCutpos1(BinXY);
    if ~isempty(cutPos)
        InsLR = LineR32(LineR32(:,3) > min(LineR32(:,3))+cutPos*GridWidth,:);
        % drowPts(LineR32,'.b',InsLR,'.r')
        % Extracting point clouds of insulators on power towers
        TowerR = Tower*rotz(Theta1*180/pi)*roty(Theta2*180/pi);
        % drowPts(LineR32,'.g',TowerR,'.b',InsLR,'.r')
        InsLRC = InsLR(InsLR(:,3) >= min(InsLR(:,3)) & InsLR(:,3) <= min(InsLR(:,3)) + (max(InsLR(:,3)) - min(InsLR(:,3))) / 3 * 2,:);
        % drowPts(InsLR,'.b',InsLRC,'.r')
        InsTR = TowerR(TowerR(:,1) < max(InsLRC(:,1)) & TowerR(:,1) > min(InsLRC(:,1)) ...
            & TowerR(:,2) < max(InsLRC(:,2)) & TowerR(:,2) > min(InsLRC(:,2)) ...
            & TowerR(:,3) > min(InsLR(:,3)),:);
        % drowPts(TowerR,'.g',LineR32,'.g',InsLR,'.g',InsLRC,'.b',InsTR,'.r')
        InsR = [InsLR;InsTR];
        Len = max(InsR(:,3)) - min(InsR(:,3));
        Ins = [Ins;InsR * roty(-Theta2*180/pi) * rotz(-Theta1*180/pi)];
    else
    end
end


function cutPos = getCutpos1(BinImg)
% Using hole detection to crop locations

% drowZbarh(BinImg,1)
Epy = drowZbarh(BinImg,1,'epy');% imshow(flipud(BinImg))
% b = barh(Epy);
% Mark rows with hole 0
Epy0 = Epy;
Epy0(Epy0 ~= 0) = -1;
Epy0(Epy0 == 0) = 1;
Epy0(Epy0 == -1) = 0;
% Find the starting and ending positions of the interval
DEpy0 = diff(Epy0);
Beg = find(DEpy0 == 1); % Starting hole interval index
% The nearest interval above the position with the largest density change
DS = drowZbarh(BinImg,1,'Dsum');
[~,Dwid_max_ind] = max(DS);
% b = barh(DS); b.FaceColor = 'flat'; b.CData(Dwid_max_ind,:) = [1,0,0];
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