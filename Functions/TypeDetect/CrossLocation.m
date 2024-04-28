function Loc = CrossLocation(Img,ratio)
% Calculate the position of the cross arm of the binary image tower

% drowZbarh(Img,1)
ZD = sum(Img,2); %point density
% take the upper part
halfLen = floor(length(ZD)/2);
DZD = ZD(length(ZD)-halfLen:end);
% Cut off the first third of the length
DZD = DZD - max(DZD)/ratio;
DZD(DZD<0) = 0;
% Dislocation histogram
DZMove = zeros(length(DZD),1);
DZMove(2:length(DZD)) = DZD(1:end-1); %move down
% subplot(1,3,1); barh(ZD)
% subplot(1,3,2); barh(DZD)
% subplot(1,3,3); barh(DZMove)
% Dislocation addition
SumDZ = DZD + DZMove;
% Calculate the start and end positions of each cross arm
IndBegin = find(DZD == SumDZ & SumDZ ~= 0) + halfLen - 1;
IndEnd = find(DZMove == SumDZ & SumDZ ~= 0) + halfLen - 1;% barh(ZD)
% remove noise
maxDZ = max(ZD);
k = 1;
while k <= length(IndEnd)
    if max(ZD(IndBegin(k):IndEnd(k))) * 1.8 < maxDZ
        IndBegin(k) = [];
        IndEnd(k) = [];
    else
        k = k+1;
    end
end

if length(IndBegin) ~=length(IndEnd)
    Loc = flipud([IndBegin,[IndEnd;size(Img,1)]]);
else
    % The first cross arm is at the top
    Loc = flipud([IndBegin,IndEnd]);
end

end