function [InsPts,theta1,theta2] = InsInHLine3(T,L,GridWidth)
%INSINHLINE 
% drowPts(T,'.b',L,'.r')
InsPts = zeros(0,3);
[LR3,theta1] = RotawithAxle(L,3); % drowPts(LR3,'.r')
[LR32,theta2] = RotawithAxle(LR3,2); % drowPts(LR32,'.r')
TR32 = T * rotz(theta1*180/pi) * roty(theta2*180/pi);
TXMid = min(TR32(:,1)) + (max(TR32(:,1)) - min(TR32(:,1))) / 2;
%% XY plane extraction insulator
BinXY = BinProjection(LR32,GridWidth,1,2); % drowZbarh(BinXY,-2)
XYWid = drowZbarh(BinXY,-2,'wid');% barh(XYWid)
% XYSum = drowZbarh(BinXY,-2,'sum');% barh(XYSum)
FXYWid = fillBin(XYWid);% barh(FXYWid)
[~,maxInd] = max(XYWid);
midPos = ceil(size(FXYWid,1) / 2);
if abs(TXMid - min(LR32(:,1))) > abs(TXMid - max(LR32(:,1)))
% if maxInd > size(XYWid,1) / 2 % Insulator on top
    % remove noise
    maxV = max(FXYWid(1:midPos));
    if sum(FXYWid(1:midPos) == maxV) <= 3
        FXYWid(FXYWid(1:midPos) == maxV) = maxV - 1;
    end
    % Calculate threshold
    Thre = max(FXYWid(1:midPos)) + 1;
    cutPos = midPos + find(FXYWid(midPos+1:end) > Thre,1,"first") - 1;
    %  b = bar(XYWid);b.FaceColor = 'flat';b.CData(cutPos+1,:) = [1 0 0];xlabel('width');ylabel('count')
    if ~isempty(cutPos)
        InsPts = LR32(LR32(:,1) > min(LR32(:,1)) + cutPos * GridWidth,:);
        isCut = true;
    else
        isCut = false;
    end
else % Insulator below
    % remove noise
    maxV = max(FXYWid(midPos+1:end));
    if sum(FXYWid(midPos+1:end) == maxV) <= 3
        FXYWid(FXYWid(midPos+1:end) == maxV) = maxV - 1;
    end
    Thre = max(FXYWid(midPos+1:end)) + 1;
    cutPos = find(FXYWid(1:midPos) > Thre,1,"last");
    if ~isempty(cutPos)
        InsPts = LR32(LR32(:,1) < min(LR32(:,1)) + cutPos * GridWidth,:);
        isCut = true;
    else
        isCut = false;
    end
end
%% The XY plane cannot be divided, so use the XZ plane
if ~isCut
    BinXZ = BinProjection(LR32,GridWidth,1,3); % drowZbarh(BinXZ,-2)
    XZWid = drowZbarh(BinXZ,-2,'wid');% barh(XZWid)
    FXZWid = fillBin(XZWid);% barh(FXZWid)
    if abs(TXMid - min(LR32(:,1))) > abs(TXMid - max(LR32(:,1))) % 
        Thre = 4;
        cutPos = find(FXZWid > Thre,1,"last");
        if ~isempty(cutPos)
            InsPts = LR32(LR32(:,1) > min(LR32(:,1)) + cutPos * GridWidth,:);
        end
    else
        Thre = 4;
        cutPos = find(FXZWid > Thre,1,"first");
        if ~isempty(cutPos)
            InsPts = LR32(LR32(:,1) < min(LR32(:,1)) + cutPos * GridWidth,:);
        end
    end
end
% drowPts(LR32,'.b',InsPts,'.r')
% writematrix(LR32,'LR32.txt');writematrix(InsPts,'InsPts.txt');
end