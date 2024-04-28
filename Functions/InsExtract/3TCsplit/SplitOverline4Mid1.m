function LinePts = SplitOverline4Mid1(CrossPts,GridWidth)
%INSINOVERLINE Separate power lines and crossover lines located in the middle of the tower
halflen = (max(CrossPts(:,2)) - min(CrossPts(:,2))) / 2;
midYL =  min(CrossPts(:,2)) + halflen;
LinePts = zeros(0,3);
for i = 1:2
    halfCross = CrossPts(CrossPts(:,2) > min(CrossPts(:,2)) + halflen * (i-1) & ...
        CrossPts(:,2) < min(CrossPts(:,2)) + halflen * i,:);% drowPts(CrossPts,'.b',halfCross,'.r')
    % ---------Intercept the 1/4 part of the power line away from the power tower and calculate the main direction of the power line
    dMin = abs(midYL - min(halfCross(:,2)));
    dMax = abs(midYL - max(halfCross(:,2)));
    partlen = (max(halfCross(:,2)) - min(halfCross(:,2))) / 3;
    if dMin < dMax %The smaller Y coordinate value is closer to the power tower
        edgeline = halfCross(halfCross(:,2) > max(halfCross(:,2)) - partlen,:);
    else %The larger Y coordinate value is closer to the power tower
        edgeline = halfCross(halfCross(:,2) < min(halfCross(:,2)) + partlen,:);
    end % drowPts(halfCross,'.b',edgeline,'.r')
    [edgelineR3,theta1] = RotawithAxle(edgeline,3);
    [edgelineR32,theta2] = RotawithAxle(edgelineR3,2);
    halfCrossR32 = halfCross * rotz(rad2deg(theta1)) * roty(rad2deg(theta2));
    % drowPts(halfCrossR32,'.b',edgelineR32,'.r')
    % ---------Change axis
    halfCrossR32R = halfCrossR32(:,[1,3,2]);% drowPts(halfCrossR32R,'.r')
    BinXZ = BinProjection(halfCrossR32R,GridWidth,1,3);% drowZbarh(BinXZ,1)
    [~,maxWidInd] = max(drowZbarh(BinXZ,1,'wid'));
    if maxWidInd > size(BinXZ,1) / 2
        % LineR32 = Cut_Recursion(halfCrossR32R,GridWidth);
        LineR32 = cut1(halfCrossR32R,GridWidth);
        LineR32 = cut2(LineR32,GridWidth);
    else
        halfCrossR32R(:,3) = -halfCrossR32R(:,3);
        % LineR32 = Cut_Recursion(halfCrossR32R,GridWidth);% drowPts(Cross,'.b',QLineCut,'.r')
        LineR32 = cut1(halfCrossR32R,GridWidth);
        LineR32 = cut2(LineR32,GridWidth);
        LineR32(:,3) = -LineR32(:,3);
    end
    Line = LineR32(:,[1,3,2]) * roty(-rad2deg(theta2)) * rotz(-rad2deg(theta1));
    % drowPts(halfCross,'.b',Line,'.r')
    LinePts = [LinePts;Line];
end
end

function [l,c] = cut1(L,GridWidth)
    BinXZ = BinProjection(L,GridWidth,1,3);% drowZbarh(BinXZ,1)
    ZWid = drowZbarh(BinXZ,1,'wid');% barh(ZWid)
    DZWid = diff(ZWid);% barh(DZWid)
    NDZWid = DetectMutaInBin(DZWid);
    % subplot(1,2,1);barh(DZWid);subplot(1,2,2);barh(NDZWid)
    cutpos = find(NDZWid > 35,1,"first");
    l = L(L(:,3) > min(L(:,3)) + cutpos * GridWidth,:);
    c = L(L(:,3) <= min(L(:,3)) + cutpos * GridWidth,:);
    % drowPts(L,'.y',c,'.b',l,'.r')
end

function [l,c] = cut2(L,GridWidth)
% drowPts(L,'.r')
    BinXY = BinProjection(L,GridWidth,1,2);% drowZbarh(BinXY,1)
    ZWid = drowZbarh(BinXY,1,'wid');% barh(ZWid)
    DZWid = diff(ZWid);% barh(DZWid)
    NDZWid = DetectMutaInBin(DZWid);
    % subplot(1,2,1);barh(DZWid);subplot(1,2,2);barh(NDZWid)
    [~,cutpos] = max(NDZWid);
    if ~isempty(cutpos) && cutpos ~= 1
        l = L(L(:,2) > min(L(:,2)) + cutpos * GridWidth,:);
        c = L(L(:,2) <= min(L(:,2)) + cutpos * GridWidth,:);
    else
        l = L;
        c = [];
    end
    % drowPts(L,'.y',c,'.b',l,'.r')
end
