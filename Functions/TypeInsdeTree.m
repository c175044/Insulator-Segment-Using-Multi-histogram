function [InsPts,isCable,InsLen] = TypeInsdeTree(T,L,GridWidth)
% drowPts(T,'.r',L,'.b')
%% Tower cross arm identification
Type = TypeDetect(T,L);
BinYZ = BinProjection(T,GridWidth,1,3);% drowZbarh(BinYZ,1)
Loc = CrossLocation(BinYZ,3);
InsPts = zeros(0,3); isCable = 0;
switch Type
    case {1,8} %Wine glass tower, portal tower
        CrossLine = SplitPowerLine_2D(L,0.1,1,10);
        % drowPts(T,'.b',L,'.g',CrossLine{1},'.r')
        [InsPts,InsLen] = InsExtract_ZL(T,CrossLine,Loc,GridWidth,Type);
    case 2 %cat head tower
        CrossLine = SplitPowerLine_C4(L,2,0.6); %Separating power lines using clustering methods
        % drowPts(T,'.y',L,'.y',CrossLine{1},'.b',[CrossLine{2}],'.r')
        [InsPts,isCable,InsLen] = InsExtract_ZL1(T,CrossLine,Loc,GridWidth); %The insulator is positioned vertically
    case 3 %
        CrossLine = SplitPowerLine_C4(L,2,0.5);
        % drowPts(T,'.y',L,'.y',[CrossLine{1}],'.b',[CrossLine{2}],'.r')
        LocP = appendLoc(BinYZ,Loc);
        [InsPts,InsLen] = InsExtractType4(T,CrossLine,LocP,GridWidth,Type); %The insulator is positioned horizontally
    case 4 %Tension resistant dry type electricity tower
        LC = SplitPowerLine_C5(L,4,0.5);% drowPts(L,'.b',LC,'.r')
        CrossLine = SplitPowerLine_C41(LC,2,1);
        % drowPts(T,'.y',L,'.y',[CrossLine{1}],'.b',[CrossLine{2}],'.r')
        [InsPts,InsLen] = InsExtractType4(T,CrossLine,Loc,GridWidth,Type);
    case 5 %Tension type drum tower
        CrossLine = SplitPowerLine_C4(L,3,0.8);
        % drowPts(T,'.y',L,'.y',[CrossLine{1}],'.g',[CrossLine{2}],'.b',[CrossLine{3}],'.r')
        [InsPts,InsLen] = InsExtractType51(T,CrossLine,Loc,GridWidth,Type); %The insulator is positioned horizontally
    case 6 %DC drum tower
        CrossLine = SplitPowerLine_2D(L,0.1,3,10);
        % drowPts(T,'.y',L,'.y',[CrossLine{1}],'.b',[CrossLine{2}],'.r',[CrossLine{3}],'.g')
        [InsPts,InsLen] = InsExtract_ZL(T,CrossLine,Loc,GridWidth,Type); %The insulator is positioned vertically
end
end