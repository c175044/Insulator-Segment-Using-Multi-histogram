function bestline = RanSaC_Fitline(pts,Iter,t)
% Fitting 4 vertices of a two-dimensional planar rectangle
% Input parameters: points, Iter (number of sampling), t (distance threshold from point to straight line)
% Output parameters: bestline (plane straight line parameter)

% figure;plot3(points(1,:),points(2,:),points(3,:),'.');hold on;
points(:,1) = pts(:,1) - min(pts(:,1));
points(:,2) = pts(:,2) - min(pts(:,2));
maxNum = 0; %The number of optimal fitting interior points
for j = 1:Iter %Cycles
    midY = min(points(:,2)) + (max(points(:,2)) - min(points(:,2))) / 2; %Y midpoint coordinate
    %Randomly sample fitting points (lower part)
    DPts = points(points(:,2) < midY,:); %Points in the lower half
    UPts = points(points(:,2) > midY,:); %The number of points in the upper half (according to the input rules, it is impossible to have points equal to minY)
    sampleNum = min(ceil(size(DPts,1) / 3),ceil(size(UPts,1) / 3)); %The number of sampling points in the upper and lower halves
    sampleidxD = randperm(size(DPts,1),sampleNum); % Index of random sampling points in the lower half
    sampleidxU = randperm(size(UPts,1),sampleNum); % Index of random sampling points in the upper half
    samPts = [DPts(sampleidxD,:);UPts(sampleidxU,:)];
    %     plot(points(:,1),points(:,2),'.r',samPts(:,1),samPts(:,2),'.b');axis equal;
    % preliminary line
    f = polyfit(samPts(:,1),samPts(:,2),1);
    distance = abs(f(1)*points(:,1) - points(:,2) + f(2)) ./ sqrt(f(1)^2 + 1); %The distance from each point to the straight line
    % Number of points near the straight line
    nearSum = sum(distance < t);
    % Number of points near the straight line
    if nearSum > sampleNum
        nearPts = points(distance < t,:);
        f = polyfit(nearPts(:,1),nearPts(:,2),1);
        distance = abs(f(1)*points(:,1) - points(:,2) + f(2)) ./ sqrt(f(1)^2 + 1);
        nearSum = sum(distance < t); %Update point count
    end
    % Update optimal planes and proportions
    if nearSum > maxNum
        finDist = distance;
        maxNum = nearSum;
    end
end
bestpts = pts(finDist < t,:);
bestline =  polyfit(bestpts(:,1),bestpts(:,2),1);
if abs(bestline(1)) < 20
    midY = min(bestpts(:,2)) + (max(bestpts(:,2)) - min(bestpts(:,2))) / 2;
    DPts = bestpts(bestpts(:,2) < midY,:); %Points in the lower half
    UPts = bestpts(bestpts(:,2) > midY,:); %The number of points in the upper half (according to the input rules, it is impossible to have points equal to minY)
    sampleNum = min(ceil(size(DPts,1) / 3),ceil(size(UPts,1) / 3)); %The number of sampling points in the upper and lower halves
    sampleidxD = randperm(size(DPts,1),sampleNum); % Index of random sampling points in the lower half
    sampleidxU = randperm(size(UPts,1),sampleNum); % Index of random sampling points in the upper half
    bestpts = [DPts(sampleidxD,:);UPts(sampleidxU,:)];
    bestline = polyfit(bestpts(:,1),bestpts(:,2),1);
end
% Plot the fitting results
% plot(pts(:,1),pts(:,2),'.b',bestpts(:,1),bestpts(:,2),'.r');hold on
% yright = min(pts(:,2)):0.1:max(pts(:,2));
% xright = (yright - bestline(2)) ./ bestline(1);
% plot(xright,yright,'-g','LineWidth',1);axis equal;
end

