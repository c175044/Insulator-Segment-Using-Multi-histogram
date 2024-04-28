function [InsPts,InsLen] = ExtraInsWithLineH1(L,T,Line,isUpCross,GridWidth)
% -------------------Cut power lines and towers for the first time based on the straight line fitted to the tower-----------------
% Cut power line points and electric tower points according to the fitted straight line

InsPts = zeros(0,3);
InsLen = 0;
if isUpCross
%     % Drawing
%     scatter3(T(:, 1), T(:, 2), T(:, 3), '.b');hold on;axis equal
%     scatter3(L(:, 1), L(:, 2), L(:, 3), '.r');
%     [X,Z] = meshgrid(min(T(:,1)):0.5:max(T(:,1)),min(T(:,3)):0.5:max(T(:,3)));
%     Y =  (Z - Line(2)) / Line(1);mesh(X, Y, Z)
    Yd = min(L(:,2)) + (max(L(:,2)) - min(L(:,2))) / 2;
    Zd = min(L(:,3)) + (max(L(:,3)) - min(L(:,3))) / 2;
    if Line(1) * Yd - Zd + Line(2) < 0 %The power line is to the left of the fitted straight line
        LP = L(Line(1) * L(:,2) - L(:,3) + Line(2) < 0,:);
        TP = T(Line(1) * T(:,2) - T(:,3) + Line(2) < 0,:);
    else
        LP = L(Line(1) * L(:,2) - L(:,3) + Line(2) > 0,:);
        TP = T(Line(1) * T(:,2) - T(:,3) + Line(2) > 0,:);
    end
else
    % Drawing
    % scatter3(T(:, 1), T(:, 2), T(:, 3), '.b');hold on;axis equal
    % scatter3(L(:, 1), L(:, 2), L(:, 3), '.r');
    % [X,Z] = meshgrid(min(T(:,1)):0.5:max(T(:,1)),min(T(:,3)):0.5:max(T(:,3)));
    % Y = Line(1) * X + Line(2) ;mesh(X, Y, Z)
    Xd = min(L(:,1)) + (max(L(:,1)) - min(L(:,1))) / 2;
    Yd = min(L(:,2)) + (max(L(:,2)) - min(L(:,2))) / 2;
    if Line(1) * Xd - Yd + Line(2) < 0 %The power line is to the left of the fitted straight line
        LP = L(Line(1) * L(:,1) - L(:,2) + Line(2) < 0,:);
        TP = T(Line(1) * T(:,1) - T(:,2) + Line(2) < 0,:);
    else
        LP = L(Line(1) * L(:,1) - L(:,2) + Line(2) > 0,:);
        TP = T(Line(1) * T(:,1) - T(:,2) + Line(2) > 0,:);
    end
end
% drowPts(L,'.y',T,'.y',LP,'.r',TP,'.b')
% -------------------------Cut off the other direction of the insulator-------------------
[InsPts1R32,theta1,theta2] = InsInHLine3(T,LP,GridWidth);
if ~isempty(InsPts1R32)
    % ----------------------Extract electric tower point cloud based on insulator extraction results-------------------
    TPR32 = TP * rotz(theta1*180/pi) * roty(theta2*180/pi);% drowPts(TowerPtsCut1R,'.b',LinePtsCut3R32,'.r')
    InsPts2R32 = TPR32(TPR32(:,2) < max(InsPts1R32(:,2)) & TPR32(:,2) > min(InsPts1R32(:,2)) ...
        & TPR32(:,3) < max(InsPts1R32(:,3)) & TPR32(:,3) > min(InsPts1R32(:,3)),:);
    InsPtsR32 = [InsPts1R32;InsPts2R32];
    InsLen = max(InsPtsR32(:,1)) - min(InsPtsR32(:,1));
    % Merge results
    InsPts = InsPtsR32 * roty(-theta2*180/pi) * rotz(-theta1*180/pi); % Rotate to original coordinates
    % drowPts(L,'.y',T,'.y',InsPts2,'.b',InsPts1,'.r')
end
end

