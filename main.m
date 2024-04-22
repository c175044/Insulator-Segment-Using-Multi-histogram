% 识别杆塔类型
clc;clear
DataPath = '.\Data\电塔序号1\';
addpath(genpath('.\Functions'))
Types = importdata('TowerTypes.txt');
% 读取杆塔号
TowerIds = getTowerID(DataPath);
TowerNum = size(TowerIds,1);
% 判断杆塔类型并识别耐张塔
for i = 1:TowerNum
    TowerID = TowerIds(i,:);
    % 读取电力线和杆塔点云
    TowerPts = importdata([DataPath,TowerID,'杆塔.txt']);
    LinePts = importdata([DataPath,TowerID,'电力线.txt']);
    % 重定向
    [TowerPtsR,theta] = RTower(TowerPts);
    LinePtsR = LinePts * rotz(theta*180/pi);
    % drowPts(TowerPtsR,'.b',LinePtsR,'.r')
    GridWidth = 0.05:0.01:0.15;
%     GridWidth = 0.11;
    GridNum = length(GridWidth);
    InsPtsInGrids = cell(1,0);
    LenPtsInGrids = zeros(1,0);
    TypeID = Types(Types(:,1) == str2double(TowerID),2);
    for j = 1:GridNum
        Inscell = TypeInsdeTree(TowerPtsR,LinePtsR,GridWidth(j),TypeID);
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

keyboard
movefile([DataPath,TowerID,'杆塔.txt'],['.\Data\电塔序号1\',TowerID,'杆塔.txt'])
movefile([DataPath,TowerID,'电力线.txt'],['.\Data\电塔序号1\',TowerID,'电力线.txt'])