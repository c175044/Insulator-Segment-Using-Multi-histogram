function [InsPts,InsLen] = InsExtractType51(T,L,Loc,GridWidth,Type)
% Extraction of tensile drum tower insulators
InsPts = cell(24,1);
InsPts = cellfun(@(x) [0,0,0], InsPts, 'UniformOutput', false);
% InsLen = zeros(24,1);
% try
Loc = Loc(end-2:end,:);
for k = 1:3
    CL = [L{k}]; % The power line point at the i-th cross arm % drowPts(CL,'.r')
    % -------------------------------Cut off the crossover line--------------------------------
    CrossBeg = min(T(:,3)) + GridWidth * Loc(k,2); %The starting point of the second cross arm
    CrossEnd = min(T(:,3)) + GridWidth * Loc(k,1); %The starting point of the second cross arm
    CP = T(T(:,3) < CrossEnd + 0.5 & T(:,3) > CrossEnd,:);
    % drowPts(T,'.b',CP,'.r')
    [fleft,fright] = fitPlane1(CP,Type); %Extract two boundary lines
    fitLine = [fleft;fright];
    % -------------Transverse insulator extraction
    midYT = min(CP(:,2)) + (max(CP(:,2)) - min(CP(:,2))) / 2;
    halfLenY = abs(min(CL(:,2)) - midYT);
    OverInOneCross = zeros(0,3);
    l = 1;
    for i = 1:2
        % a transmission direction
        HL = CL( ...
            CL(:,2) >= min(CL(:,2)) + halfLenY * (i - 1) & ...
            CL(:,2) <= min(CL(:,2)) + halfLenY * i ...
            ,:);% drowPts(CL,'.b',HL,'.r')
        % Take the largest cluster to rotate the power lines
        labels = dbscan(HL,1,5);
        HLOne = HL(labels==MaxLabel(labels),:);% drowPts(HL,'.b',HLOne,'.r')
        [~,theta] = RotawithAxle(HLOne,3);
        HLR3 = HL * rotz(rad2deg(theta));
        CP2R3 = CP * rotz(rad2deg(theta));
        % drowPts(CP2R3,'.b',HLR3,'.r')
        % Intercept half of the rotated point cloud
        midYTR = min(CP2R3(:,2)) + (max(CP2R3(:,2)) - min(CP2R3(:,2))) / 2;
        QLRcell = cell(0,1);
        QLRcell = [QLRcell;HLR3(HLR3(:,2) < midYTR,:)];
        QLRcell = [QLRcell;HLR3(HLR3(:,2) >= midYTR,:)];
        for j = 1:2
            QLR = QLRcell{j};% drowPts(CP2R3,'.y',HLR3,'.b',QLR,'.r')
            QL = QLR * rotz(rad2deg(-theta));% drowPts(CP,'.b',QL,'.r')
            % Eliminate discrete points
            labels = dbscan(QL,2,20);
            QLC = QL(labels == MaxLabel(labels),:);
            % drowPts(QL,'.b',QLC,'.r')
            % ---------split jumper
            [QLC1,QLC2] = Cut_OverLine(QLC,GridWidth,fitLine(i,:),midYT);
            QLC2 = [QLC2;QL(labels ~= MaxLabel(labels),:)];
            % drowPts(QLC2,'.b',QLC1,'.r')
            % ---------Insulator extraction (single transverse power line insulator extraction)
            [Ins,Len] = ExtraInsWithLineH1(QLC1,CP,fitLine(i,:),0,GridWidth);
            % drowPts(QLC1,'.b',Ins,'.r')
            OverInOneCross = [OverInOneCross;[QLC2,repmat(l,size(QLC2,1),1)]];
            InsPts{(k-1) * 8 + (i-1) * 4 + j} = Ins;
            InsLen((k-1) * 8 + (i-1) * 4 + j) = Len;
            l = l + 1;
        end
    end
    
    % -------------Longitudinal insulator extraction
    % drowPts(CL,'.b',OverInOneCross,'.r');
    midXCL = (max(CL(:,1)) - min(CL(:,1))) / 2;
    for i = 1:2
        QLC3 = OverInOneCross(...
            OverInOneCross(:,1) >= min(CL(:,1)) + midXCL*(i-1) & ...
            OverInOneCross(:,1) < min(CL(:,1)) + midXCL*i,1:3);
        % drowPts(OverInOneCross(:,1:3),'.b',QLC3,'.r');
        labels = dbscan(QLC3,1,5);
        QLC4 = QLC3(labels == MaxLabel(labels),:);
        % drowPts(QLC3,'.b',QLC4,'.r');
        BinYZ = BinProjection(QLC4,GridWidth,2,3);% drowZbarh(BinYZ,-2);
        wid1 = drowZbarh(BinYZ,-2,'wid');% bar(wid1);xlabel('width');ylabel('count')
        cutThre = 10;
        try
            [begPos,endPos] = findMuta(wid1,cutThre);
        catch
            continue
        end
        if ~isempty(begPos)
            wid2 = endPos(end) - begPos(1) + 1;
            try
                HBinYZ = BinYZ(:,begPos(1) - wid2:endPos(end) + wid2);% drowZbarh(HBinYZ,1)
            catch
                continue
            end
            cutPosH2 = find(drowZbarh(HBinYZ,1,'wid') > wid2,1,"last") + 1;% drowZbarh(BinYZ,1)
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
            allPts = [T;CL];% drowPts(allPts,'.b',InsInO,'.r');
            Ins = allPts( ...
                allPts(:,3) < min(T(:,3)) + (Loc(k,1)-1) * GridWidth & ...
                allPts(:,3) > min(InsInO(:,3)) & ...
                allPts(:,2) < max(InsInO(:,2)) & allPts(:,2) > min(InsInO(:,2)) & ...
                allPts(:,1) < max(InsInO(:,1)) & allPts(:,1) > min(InsInO(:,1)) ...
                ,:);% drowPts(T,'.y',CL,'.b',Ins,'.r')

        if (length(begPos) == 2 && (begPos(2) - endPos(1)) > 0) || ...
                length(begPos) == 1 && (endPos(1) - begPos(1)) * GridWidth > 1
            midPos = endPos(1) + ceil((begPos(end) - endPos(1)) / 2);
            Ins1 = Ins(Ins(:,2) >= min(QLC4(:,2)) & Ins(:,2) <= min(QLC4(:,2)) + midPos * GridWidth,:);
            Ins2 = Ins(Ins(:,2) > min(QLC4(:,2)) + midPos * GridWidth & Ins(:,2) <= max(QLC4(:,2)),:);
%             Len1 = max(Ins1(:,3)) - min(Ins1(:,3));
%             Len2 = max(Ins2(:,3)) - min(Ins2(:,3));
            InsPts{(k-1)*8 + 2 + 4*(i-1) + 1} = Ins1;
            InsPts{(k-1)*8 + 2 + 4*(i-1) + 2} = Ins2;
%             InsLen((k-1)*8 + 4 + (i-1) + 1) = Len1;
%             InsLen((k-1)*8 + 4 + (i-1) + 2) = Len2;
        else
%             Len = max(Ins(:,3)) - min(Ins(:,3));
            InsPts{(k-1)*8 + 2 + 4*(i-1) + 1} = Ins;
%             InsLen((k-1)*8 + 4 + (i-1) + 1) = Len;
        end
        end
    end
    % InsPts = [InsPts;InsPts1;InsPts2];
end
% catch
%     return
% end
% writematrix(TowerPts,'TowerPts.txt');writematrix([LinePts{1};LinePts{2};LinePts{3}],'LinePts.txt');writematrix(InsPts,'InsPts.txt');
% drowPts(TowerPts,'.y',[LinePts{1};LinePts{2};LinePts{3}],'.b',InsPts,'.r')
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
% Delete mutations that are too small near both ends
gap2 = endPos - begPos;
delgap = find(gap2 <= 3 | (begPos < length(BinC) / 4 | endPos > length(BinC) / 4 * 3));
begPos(delgap) = [];
endPos(delgap) = [];
% barh(Bin)
end
