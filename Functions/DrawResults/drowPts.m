function drowPts(varargin)
%DROWPTS 
% plot3(TowerPtsR(:,1),TowerPtsR(:,2),TowerPtsR(:,3),'.','Color', [0.5, 0.5, 0.5]); hold on
% plot3(LinePtsRCut(:,1),LinePtsRCut(:,2),LinePtsRCut(:,3),'.','Color', [0.5, 0.5, 0.5])
% Check the number of input variables
Pts1 = [varargin{1}];
if size(Pts1,2) == 4
    classNum = [varargin{end}];
    if classNum == 0
        classNum = length(unique(Pts1(:,4)));
    end
    colorMap = hsv(classNum);
    for i = 1:classNum
        color = colorMap(i, :);
        clusterUni = unique(Pts1(:,4));
        CurCluster = clusterUni(i);
        plot3(Pts1(Pts1(:,4)==CurCluster,1), Pts1(Pts1(:,4)==CurCluster,2), Pts1(Pts1(:,4)==CurCluster,3), '.', 'Color', color); hold on
    end
elseif isnumeric([varargin{end}])
    colorMap = hsv(nargin);
    for i = 1:nargin
        Pts = [varargin{i}]; color = colorMap(i, :);
        plot3(Pts(:,1), Pts(:,2), Pts(:,3), '.', 'Color', color); hold on
    end
else
    for i = 1:nargin/2
        k = (i-1)*2+1;
        Pts = [varargin{k}]; Style = [varargin{k+1}];
        plot3(Pts(:,1),Pts(:,2),Pts(:,3),Style); hold on
    end
end
% Draw each set of point clouds in turn
axis equal
hold off
end

