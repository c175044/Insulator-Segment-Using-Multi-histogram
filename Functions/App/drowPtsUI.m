function drowPtsUI(varargin)
%DROWPTS 
cla(varargin{end});
% Check the number of input variables
Pts1 = [varargin{1}];
if size(Pts1,2) == 4
    classNum = [varargin{end}];
    colorMap = hsv(classNum);
    for i = 1:classNum
        color = colorMap(i, :);
        clusterUni = unique(Pts1(:,4));
        CurCluster = clusterUni(i);
        plot3(varargin{end},Pts1(Pts1(:,4)==CurCluster,1), Pts1(Pts1(:,4)==CurCluster,2), Pts1(Pts1(:,4)==CurCluster,3), '.', 'Color', color); hold on
    end
elseif isnumeric([varargin{end-1}])
    colorMap = hsv(nargin);
    for i = 1:nargin
        Pts = [varargin{i}]; color = colorMap(i, :);
        plot3(varargin{end},Pts(:,1), Pts(:,2), Pts(:,3), '.', 'Color', color); hold on
    end
elseif nargin > 0
    for i = 1:(nargin-1)/2
        k = (i-1)*2+1;
        Pts = [varargin{k}]; Style = [varargin{k+1}];
        plot3(varargin{end},Pts(:,1),Pts(:,2),Pts(:,3),Style); hold(varargin{end},"on");
    end
end
% Draw each set of point clouds in turn
axis(varargin{end},'equal')
hold(varargin{end},"off");
end

