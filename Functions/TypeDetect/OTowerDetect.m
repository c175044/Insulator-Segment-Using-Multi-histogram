function isOTower = OTowerDetect(Img,CutLoca,autoThreshold)
%OTowerDetect Detect binary image holes and calculate their area size. If the area is greater than the threshold, it is an O-shaped tower.
%   input：
%       BGI：The image after redirection and binarization of the power tower
%       CutLoca：Partial power tower interception locations
%       autoThreshold：Hole area threshold
%   output：
%       isOTower：Returns 1 if it is an O-type tower, otherwise returns 0
%% Only judge part of the above
Img = Img(ceil(size(Img,1) * CutLoca):end,:);
%% Identify electric towers according to the largest void area
Img(1,:) = 1;
[Graph,L,n] = bwboundaries(Img,8);
ObjFea = regionprops(L,'Area','Centroid');
Area = zeros(1,length(Graph));
% Drawing
% imshow(Img);hold on 
for i = n+1:length(Graph)
    Area(i) = ObjFea(i).Area;
    % Drawing
%     if Area(i) > 100 
%         Center = ObjFea(i).Centroid;
%         text(Center(1),Center(2),num2str(Area(i)),'Color','y')
%         boundary = Graph{i};
%         plot(boundary(:,2), boundary(:,1), 'r', 'LineWidth', 1);
%     end
end
MaxArea = max(Area);
if nargin == 2
    autoThreshold = size(Img,1)*size(Img,2)*0.07;
end

if MaxArea > autoThreshold && MaxArea > 300
    isOTower = 1;
else
    isOTower = 0;
end
%% Calculate the corner coordinates of the largest internal cavity
% DistThreshold = 5; %Grid unit length (3.3 means 0.5m at 0.15m grid resolution)
% boundary = Graph{MaxAreaInd};
% half1b = DouglasPeucker(boundary(1:floor(end/2),:)',DistThreshold);
% half2b = DouglasPeucker(boundary([floor(end/2):end,1],:)',DistThreshold);
% AnglePoints = [half1b,half2b];
% plot(AnglePoints(2,:)', AnglePoints(1,:)', '--ob');


end