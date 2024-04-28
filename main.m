clc;clear
DataPath = '.\Data\';
addpath(genpath('.\Functions'))
TowerIds = getTowerID(DataPath);
TowerNum = size(TowerIds,1);
% Traverse the tower
for i = 1:TowerNum
    TowerID = TowerIds(i,:);
    % read pts
    TowerPts = importdata([DataPath,TowerID,'Tower.txt']);
    LinePts = importdata([DataPath,TowerID,'Line.txt']);
    % redirect
    [TowerPtsR,theta] = RTower(TowerPts);
    LinePtsR = LinePts * rotz(theta*180/pi);
    % drowPts(TowerPtsR,'.b',LinePtsR,'.r')
    GridWidth = 0.05:0.01:0.15;
%     GridWidth = 0.11;
    GridNum = length(GridWidth);
    InsPtsInGrids = cell(1,0);
    LenPtsInGrids = zeros(1,0);
    for j = 1:GridNum
        Inscell = TypeInsdeTree(TowerPtsR,LinePtsR,GridWidth(j));
        [InsPts,Lens] = mergeCell3(Inscell);
        InsPtsInGrids = [InsPtsInGrids,Inscell];
        LenPtsInGrids = [LenPtsInGrids,Lens];
        % drowPts(TowerPtsR,'.g',LinePtsR,'.b',InsPts,'.r')
    end
    [fine_Ins,fine_Len,fine_grid] = adaptive_grid_tension(InsPtsInGrids, LenPtsInGrids);
    % drowPts(fine_Ins,0)
    drowPts(TowerPtsR,'.g',LinePtsR,'.b',fine_Ins(:,1:3),'.r')
    keyboard
end