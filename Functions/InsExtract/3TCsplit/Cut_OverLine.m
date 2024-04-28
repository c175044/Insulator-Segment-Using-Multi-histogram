function [Lres,Ores] = Cut_OverLine(L,GridWidth,line,midYT)
%INSINHLINE 
% drowPts(L,'.r')
% scatter3(L(:, 1), L(:, 2), L(:, 3), '.r');hold on;axis equal
% [X,Z] = meshgrid(min(L(:,1)):0.5:max(L(:,1)),min(L(:,3)):0.5:max(L(:,3)));
% Y = line(1) * X + line(2) ;mesh(X, Y, Z)
Xd = min(L(:,1)) + (max(L(:,1)) - min(L(:,1))) / 2; 
Yd = min(L(:,2)) + (max(L(:,2)) - min(L(:,2))) / 2;
if line(1) * Xd - Yd + line(2) < 0 %The power line is to the left of the fitted straight line
    LC = L(line(1) * L(:,1) - L(:,2) + line(2) < 0,:);
else
    LC = L(line(1) * L(:,1) - L(:,2) + line(2) > 0,:);
end
% drowPts(L,'.b',LC,'.r')
% ---------Intercept the 1/3 part of the power line away from the power tower and calculate the main direction of the power line
dMin = abs(midYT - min(LC(:,2)));
dMax = abs(midYT - max(LC(:,2)));
partlen = (max(LC(:,2)) - min(LC(:,2))) / 3;
if dMin < dMax %The smaller Y coordinate value is closer to the power tower
    edgeline = LC(LC(:,2) > max(LC(:,2)) - partlen,:);
else %The larger Y coordinate value is closer to the power tower
    edgeline = LC(LC(:,2) < min(LC(:,2)) + partlen,:);
end % drowPts(LC,'.b',edgeline,'.r')
[edgelineR3,theta1] = RotawithAxle(edgeline,3);
[edgelineR32,theta2] = RotawithAxle(edgelineR3,2);
LCR32 = LC * rotz(rad2deg(theta1)) * roty(rad2deg(theta2));
LR32 = L * rotz(rad2deg(theta1)) * roty(rad2deg(theta2));
% drowPts(QLR32,'.b',QLR32(labels==1,:),'.r')
% drowPts(LCR32,'.b',edgelineR32,'.r')
%% XY plane extraction insulator
BinXZ = BinProjection(LCR32,GridWidth,1,3); % drowZbarh(BinXZ,1)
XZWid = drowZbarh(BinXZ,1,'wid');% barh(XZWid)
FXZWid = fillBin(XZWid);% barh(FXZWid)
[~,maxInd] = max(FXZWid);
midPos = ceil(size(FXZWid,1) / 2);
if maxInd > midPos % Insulator on top
    % Thre = max(FXZWid(1:midPos));
    Thre = max(FXZWid) / 2;
    cutPos = midPos + find(FXZWid(midPos+1:end) > Thre,1,"first") - 1;
    LresR32 = LR32(LR32(:,3) > min(LCR32(:,3)) + cutPos * GridWidth,:);
    OresR32 = LR32(LR32(:,3) <= min(LCR32(:,3)) + cutPos * GridWidth,:);
else
    % Thre = max(FXZWid(midPos+1:end));
    Thre = max(FXZWid) / 2;
    cutPos = find(FXZWid(1:midPos) > Thre,1,"last");
    LresR32 = LR32(LR32(:,3) < min(LCR32(:,3)) + cutPos * GridWidth,:);
    OresR32 = LR32(LR32(:,3) >= min(LCR32(:,3)) + cutPos * GridWidth,:);
end
%  b = barh(XZWid);b.FaceColor = 'flat';b.CData(cutPos,:) = [1 0 0];
Lres = LresR32 * roty(-rad2deg(theta2)) * rotz(-rad2deg(theta1));
Ores = OresR32 * roty(-rad2deg(theta2)) * rotz(-rad2deg(theta1));
% drowPts(L,'.b',OverLine,'.r')
end

function Bin = fillBin(Bin)
% barh(Bin)
binNum = size(Bin,1);
for i = 1:binNum
    if Bin(i) == 0
        % Previous non-zero bin
        lastBin = 0; %Not found, may be at the first position, default is 0
        for j = i:-1:1
            if Bin(j) ~= 0
                lastBin = Bin(j);
                break
            end
        end

        % next non-zero bin
        nextBin = 0; %Not found, may be at the end, default is 0
        for j = i:binNum
            if Bin(j) ~= 0
                nextBin = Bin(j);
                break
            end
        end
        Bin(i) = floor((lastBin + nextBin) / 2);
    end
end
% barh(Bin)
end