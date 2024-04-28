function [Img,Grid] = BinProjection(Pts,GridWidth,axleX,axleY)
%BINPROJECTION Project the point cloud onto any plane formed by the coordinate axes
%   input：
%       Pts:Electric tower point cloud
%       GridWidth:Projection grid size
%       axleX:Projected x-axis
%       axleY:Projected y-axis
%   output：
%       BinGraph:Binary image (data is organized in the form of original data, 
% the lower left corner is the coordinate origin; to the right is the x-axis; up is the y-axis)

%% Calculate the index of each grid midpoint
x = Pts(:,axleX); y = Pts(:,axleY);
GridW = ceil((max(x) - min(x)) / GridWidth); %Grid width
GridH = ceil((max(y) - min(y)) / GridWidth); %grid height
Grid = cell(GridH,GridW); %Save the index of each grid midpoint
Ind = [floor((y - min(y))/GridWidth) + 1,...
    floor((x - min(x))/GridWidth) + 1]; %The grid index corresponding to each point
% The point exactly on the right boundary of the last grid
Ind(Ind(:,1) > GridH,1) = GridH;
Ind(Ind(:,2) > GridW,2) = GridW;
for i = 1:length(Pts) %Traverse points
    % The grid index corresponding to this point
    Grid{Ind(i,1),Ind(i,2)} = [Grid{Ind(i,1),Ind(i,2)},i]; % Save (expand) the index of the point in the corresponding grid
end
%% Generate binary image
Img = zeros(GridH,GridW);
for i = 1:GridH
    for j = 1:GridW
        if ~isempty(Grid{i,j})
            Img(i,j) = 1;
        end
    end
end
% imshow(flipud(Img));
end

