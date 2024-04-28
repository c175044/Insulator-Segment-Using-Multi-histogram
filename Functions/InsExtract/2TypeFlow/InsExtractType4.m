function [InsPts,InsLen] = InsExtractType4(T,L,Loc,GridWidth,Type)
%% ---------------------------First cross arm insulator point cloud extraction (horizontal)-----------------------
InsPts = cell(12,1);
InsPts = cellfun(@(x) [0,0,0], InsPts, 'UniformOutput', false);
InsLen = zeros(12,1);
CrossLine1 = [L{1}];% drowPts(T,'.b',CrossLine1,'.r')
CrossEnd = min(T(:,3)) + GridWidth * Loc(1,1); %First cross arm end point
CrossTowerPts1 = T(T(:,3) < CrossEnd-1 & T(:,3) > min(CrossLine1(:,3)),:);
% drowPts(T,'.b',CrossTowerPts1,'.r')
% Boundary line fitting
[fleft,fright] = fitPlane2(CrossTowerPts1); %Extract two boundary lines
fitLine = [fleft;fright];
% Cut off the crossover line
CrossLinePts1 = SplitOverline4Mid1(CrossLine1,GridWidth);
% drowPts(CrossLine1,'.b',CrossLinePts1,'.r')
% Extract insulators in two directions
midTPos = min(CrossLinePts1(:,2)) + (max(CrossLinePts1(:,2)) - min(CrossLinePts1(:,2))) ./ 2;
crossCell = cell(2,1);
crossCell{1} = CrossLinePts1(CrossLinePts1(:,2) < midTPos,:);
crossCell{2} = CrossLinePts1(CrossLinePts1(:,2) > midTPos,:);
for i = 1:2
    cross = crossCell{i};% drowPts(CrossLinePts1,'.b',cross,'.r')
    try 
        [Ins,Len] = ExtraInsWithLineH1(cross,CrossTowerPts1,fitLine(i,:),1,GridWidth);% drowPts(cross,'.b',Ins,'.r')
        InsPts{i} = Ins;
%         InsLen(i) = Len;
    catch
        % InsPts{i} = [0,0,0];
    end
end
% drowPts(T,'.y',[L{1}],'.b',InsPts{1},'.r',InsPts{2},'.r')
%% ---------------------------First cross arm insulator point cloud extraction (vertical)-----------------------
OverL = removeDuplicatePoints(CrossLine1,CrossLinePts1,0.001);% drowPts(CrossLine1,'.b',OverL,'.r')
if Type == 3
    OverLC = OverL(OverL(:,3) < min(T(:,3)) + GridWidth * (Loc(1,1)+1),:); % drowPts(OverL,'.b',OverLC,'.r')
else
    OverLC = OverL;
end
labels = dbscan(OverLC,2,5);% drowPts([OverLC,labels],0)
OverLC = OverLC(labels == MaxLabel(labels),:);
crossHalflen = (max(OverLC(:,2)) - min(OverLC(:,2))) ./ 2;
for i = 1:2
    try
        over1 = OverLC(OverLC(:,2) > min(OverLC(:,2)) + crossHalflen * (i - 1) ...
            & OverLC(:,2) < min(OverLC(:,2)) + crossHalflen * i,:);% drowPts(OverLC,'.b',over1,'.r')
        BinYZ = BinProjection(over1,GridWidth,2,3);% drowZbarh(BinYZ,1)
        ZW = drowZbarh(BinYZ,1,'wid');
        cutPos = find(ZW > 10,1,"last");
        QLC1 = over1(over1(:,3) > min(over1(:,3)) + GridWidth * cutPos,:);
%         Len = max(QLC1(:,3)) - min(QLC1(:,3));
        % drowPts(over1,'.b',QLC1,'.r')
        InsPts{2+i} = QLC1;
%         InsLen(2+i) = Len;
    catch
        % InsPts{2+i} = [0,0,0];
    end
end
% drowPts(T,'.y',[L{1}],'.b',InsPts2,'.r')
% drowPts(T,'.y',[L{1}],'.b',[InsPts1;InsPts2],'.r')
%% ---------------------------Second cross arm insulator point cloud extraction (horizontal)-----------------------
CL2 = [L{2}]; % drowPts(T,'.b',CL2,'.r')
CrossBeg = min(T(:,3)) + GridWidth * Loc(2,2); %The starting point of the second cross arm
CrossEnd = min(T(:,3)) + GridWidth * Loc(2,1); %The starting point of the second cross arm
CP2 = T(T(:,3) < CrossBeg & T(:,3) > CrossEnd,:);
[fleft,fright] = fitPlane1(CP2,Type); %Extract two boundary lines
fitLine = [fleft;fright];
% drowPts(T,'.b',CP2,'.r')
% -------Insulator extraction
midYT = min(CP2(:,2)) + (max(CP2(:,2)) - min(CP2(:,2))) / 2;
halfLenY = abs(min(CL2(:,2)) - midYT);
OverInOneCross = zeros(0,3);
% InsPts3 = zeros(0,3);
for i = 1:2
    % a transmission direction
    HL = CL2( ...
            CL2(:,2) >= min(CL2(:,2)) + halfLenY * (i - 1) & ...
            CL2(:,2) <= min(CL2(:,2)) + halfLenY * i ...
            ,:);% drowPts(CL2,'.b',HL,'.r')
    % Take the largest cluster to rotate the power lines
    labels = dbscan(HL,1,5);
    HLOne = HL(labels==MaxLabel(labels),:);
    [~,theta] = RotawithAxle(HLOne,3);
    HLR3 = HL * rotz(rad2deg(theta));
    CP2R3 = CP2 * rotz(rad2deg(theta));
    % drowPts(HLR3,'.b',CP2R3,'.r')
    % Intercept the general point cloud after rotation
    midYTR = min(CP2R3(:,2)) + (max(CP2R3(:,2)) - min(CP2R3(:,2))) / 2;
    halfLenYR = abs(min(HLR3(:,2)) - midYTR);
    for j = 1:2
        QL = HLR3( ...
            HLR3(:,2) >= min(HLR3(:,2)) + halfLenYR * (j - 1) &...
            HLR3(:,2) <= min(HLR3(:,2)) + halfLenYR * j ...
            ,:) * rotz(rad2deg(-theta));% drowPts(CP2,'.g',CL2,'.b',QL,'.r')
        % ---------split jumper
        [QLC1,QLC2] = Cut_OverLine(QL,GridWidth,fitLine(i,:),midYT);
        % over = Cut_Recursion(QLR32,GridWidth) * roty(-rad2deg(theta2)) * rotz(-rad2deg(theta1));
        % drowPts(QL,'.b',QLC1,'.r')
        OverInOneCross = [OverInOneCross;QLC2];
        % ---------Insulator extraction
        try
            [Ins,Len] = ExtraInsWithLineH1(QLC1,CP2,fitLine(i,:),0,GridWidth);
            % drowPts(QL,'.b',Ins,'.r')
            InsPts{4 + 2 * (i - 1) + j} = Ins;
            InsLen(4 + 2 * (i - 1) + j) = Len;
        catch
            % InsPts{4 + 2 * (i - 1) + j} = [0,0,0];
        end
    end
end
%% ---------------------------Second cross arm insulator point cloud extraction (longitudinal)-----------------------
% drowPts(T,'.y',[L{2}],'.b',InsPts3,'.r')
% drowPts(T,'.y',[L{1}],'.b',[L{2}],'.b',InsPts,'.r')
midXCL = (max(CL2(:,1)) - min(CL2(:,1))) / 2;
for i = 1:2
    QLC3 = OverInOneCross(...
        OverInOneCross(:,1) >= min(CL2(:,1)) + midXCL*(i-1) & ...
        OverInOneCross(:,1) < min(CL2(:,1)) + midXCL*i,1:3);
    % drowPts(OverInOneCross(:,1:3),'.b',QLC3,'.r');
    labels = dbscan(QLC3,1,5);
    QLC4 = QLC3(labels == MaxLabel(labels),:);
    % drowPts(QLC3,'.b',QLC4,'.r');
    BinYZ = BinProjection(QLC4,GridWidth,2,3);% drowZbarh(BinYZ,-2);
    wid1 = drowZbarh(BinYZ,-2,'wid');
    cutThre = 10;
    try
        [begPos,endPos] = findMuta(wid1,cutThre);
    catch
        continue
    end
    if ~isempty(begPos)
        wid2 = endPos(end) - begPos(1);
        try
            HBinYZ = BinYZ(:,begPos(1) - wid2:endPos(end) + wid2);% drowZbarh(HBinYZ,1)
        catch
            continue
        end
        cutPosH2 = find(drowZbarh(HBinYZ,1,'wid') > wid2,1,"last") + 1;% drowZbarh(BinYZ,1)
        % b = barh(drowZbarh(HBinYZ,1,'wid'));b.FaceColor = "flat";b.CData(cutPosH2-1,:) = [1,0,0];
        if isempty(cutPosH2)
            cutPosH2 = find(sum(HBinYZ,2) > 0,1);
        end
        InsInO = QLC4(QLC4(:,3) > min(QLC4(:,3)) + cutPosH2 * GridWidth & ...
            QLC4(:,2) <= min(QLC4(:,2)) + (endPos(end)+1) * GridWidth & ...
            QLC4(:,2) >= min(QLC4(:,2)) + (begPos(1)-1) * GridWidth,:);
        if isempty(InsInO)
            continue
        end
        % drowPts(QLC3,'.b',InsInO,'.r');
        % Extraction of insulators from electrical towers and power lines
        allPts = [T;CL2];% drowPts(allPts,'.b',InsInO,'.r');
        Ins = allPts( ...
            allPts(:,3) < min(T(:,3)) + (Loc(2,1)-1) * GridWidth & ...
            allPts(:,3) > min(InsInO(:,3)) & ...
            allPts(:,2) < max(InsInO(:,2)) & allPts(:,2) > min(InsInO(:,2)) & ...
            allPts(:,1) < max(InsInO(:,1)) & allPts(:,1) > min(InsInO(:,1)) ...
            ,:);% drowPts(T,'.y',CL2,'.b',Ins,'.r')
        if length(begPos) == length(endPos) && length(begPos) == 2 && (begPos(2) - endPos(1)) > 0
            midPos = endPos(1) + ceil((begPos(2) - endPos(1)) / 2);
            Ins1 = Ins(Ins(:,2) >= min(QLC4(:,2)) & Ins(:,2) <= min(QLC4(:,2)) + midPos * GridWidth,:);
            Ins2 = Ins(Ins(:,2) > min(QLC4(:,2)) + midPos * GridWidth & Ins(:,2) <= max(QLC4(:,2)),:);
%             Len1 = max(Ins1(:,3)) - min(Ins1(:,3));
%             Len2 = max(Ins2(:,3)) - min(Ins2(:,3));
            InsPts{8 + 2 * (i - 1) + 1} = Ins1;
            InsPts{8 + 2 * (i - 1) + 2} = Ins2;
%             InsLen(8 + 2 * (i - 1) + 1) = Len1;
%             InsLen(8 + 2 * (i - 1) + 2) = Len2;
        else
%             Len = max(Ins(:,3)) - min(Ins(:,3));
            InsPts{8 + 2 * (i - 1) + 1} = Ins;
            InsPts{8 + 2 * (i - 1) + 2} = [0,0,0];
%             InsLen(8 + 2 * (i - 1) + 1) = Len;
        end
        % drowPts(Ins,'.g',Ins1,'.b',Ins2,'.r')
    end
end

end

function [begPos,endPos] = findMuta(Bin,cutThre)
% clipping value
BinC = Bin - cutThre;% barh(BinC)
BinC(BinC<0) = 0;
% Translate one unit
DBinC = zeros(size(BinC));
DBinC(2:end) = BinC(1:end-1);% barh(DBinC)
% Dislocation addition
DF = DBinC + BinC;
% Identify starting and ending positions
begPos = find(BinC == DF & BinC ~= 0);% barh(BinC)
endPos = find(DBinC == DF & DBinC ~= 0);% barh(DBinC)
% Calculate the interval between each mutation interval
gap1 = begPos(2:end) - endPos(1:end-1);
% Merge interval that is too small
begPos(find(gap1 <= 2) + 1) = [];
endPos(gap1 <= 2) = [];
% barh(Bin)
begPos = begPos - 1;
endPos = endPos - 1;
% Delete mutations that are too small and close to both ends
delgap = find(begPos < length(BinC) / 4 | endPos > length(BinC) / 4 * 3);
begPos(delgap) = [];
endPos(delgap) = [];
% barh(Bin)
end