function [ptsD,ptsInd] = sinPro(pts,Dir,samGap)
%SINPRO Project the point cloud onto any coordinate axis (two-dimensional to one-dimensional) 
% and return the number of points in each grid

samlen = ceil((max(pts(:,Dir)) - min(pts(:,Dir))) / samGap); %Number of grids
ptsInd = floor((pts(:,Dir) - min(pts(:,Dir))) / samGap) + 1; %index of each point
ptsInd(ptsInd > samlen) = samlen;
ptsD = zeros(samlen,1);
for i = 1:length(ptsInd)
    ptsD(ptsInd(i)) = ptsD(ptsInd(i)) + 1;
end

end

