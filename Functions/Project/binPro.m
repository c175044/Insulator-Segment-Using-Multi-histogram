function Img = binPro(Pts,GridWidth,axleX,axleY)
%BINPRO Project the point cloud onto any plane formed by the coordinate axes (3D to 2D)

%% Calculate the index of each grid midpoint
x = Pts(:,axleX); y = Pts(:,axleY);
GridW = ceil((max(x) - min(x)) / GridWidth); %Grid width
GridH = ceil((max(y) - min(y)) / GridWidth); %grid height
Ind = [floor((y - min(y))/GridWidth) + 1,floor((x - min(x))/GridWidth) + 1]; %The grid index corresponding to each point
%% Generate binary image
Img = zeros(GridH,GridW);
for i = 1:GridH
    for j = 1:GridW
        if sum(Ind(:,1) == i & Ind(:,2) == j) > 0
            Img(i,j) = 1;
        end
    end
end
% imshow(flipud(Img));
end

