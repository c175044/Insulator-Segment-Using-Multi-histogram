function [PtsR,Angle] = RotawithAxle(Pts,RotaAxle)
%ROTAWITHAXLE In the three-dimensional coordinate system, 
% project the point cloud to the two coordinate axes except the RotaAxle axis, 
% and make it parallel to the X-axis
%   According to the tower reorientation results, the direction of the power lines is roughly parallel to the x-axis.

% Calculate direction vector using 2D points
% drowPts(Pts,'.r')
Axle = [1,2,3];Axle(RotaAxle) = [];
Pts2D = Pts(:,Axle);
Pts2DDownPC = pcdownsample(pointCloud([Pts2D,zeros(length(Pts),1)]),'gridAverage',0.2);
% try
%     Pts2DDownPC = pcdownsample(pointCloud([Pts2D,zeros(length(Pts),1)]),'gridAverage',0.2);
% catch %The input point cloud is empty, which usually raises this error
%     keyboard
% end 
Pts2DDown = Pts2DDownPC.Location(:,1:2);% drowPts(Pts2DDownPC.Location,'.r')
center = mean(Pts2DDown); %Plane center point
M = Pts2DDown - repmat(center,[size(Pts2DDown,1),1]); %coordinate difference
MM = (M' * M)/size(Pts2DDown,1); %covariance matrix
[V,~] = eig(MM); % Feature vector
Dire_vector = V(:,2); %direction vector of point cloud
%% Drawing
% % Rotate point cloud
% X = Pts2D(:,1)-min(Pts2D(:,1)); Y = Pts2D(:,2)-min(Pts2D(:,2));
% plot(X,Y,'.r','LineWidth',0.1);hold on;axis equal
% writematrix([X,Y],'Pts.txt')
% % first eigenvector
% Xbegin = 1.2; Ybegin = -0.5; Xend = 2.8; %Set start and end parameters
% k1 = V(2,1) / V(1,1); b1 = Ybegin - k1 * Xbegin;
% Yend = k1 * Xend + b1;
% plot([Xbegin,Xend],[Ybegin,Yend],'-r');
% writematrix([[Xbegin;Xend],[Ybegin;Yend]],'v1.txt')
% % second eigenvector
% Xbegin = 1.2; Ybegin = -0.5; Xend = -0.9; %Set start and end parameters
% k2 = V(2,2) / V(1,2); b2 = Ybegin - k2 * Xbegin;
% Yend = k2 * Xend + b2;
% plot([Xbegin,Xend],[Ybegin,Yend],'-r');
% writematrix([[Xbegin;Xend],[Ybegin;Yend]],'v2.txt')
% %% rotate
Angle = acos(abs(Dire_vector(1))/norm(Dire_vector)); %The angle between the direction vector and the projected x-axis
if RotaAxle == 1
    if Dire_vector(1) * Dire_vector(2) < 0%Rotation from y to z is positive, rotation to the y axis requires a negative sign
        Angle = -Angle;
    end
    PtsR = Pts*rotx(Angle*180/pi); 
elseif RotaAxle == 2
    if Dire_vector(1) * Dire_vector(2) > 0%Clockwise from x to z, a negative sign is required to rotate toward the x-axis
        Angle = -Angle;
    end
    PtsR = Pts*roty(Angle*180/pi);
else
    if Dire_vector(1) * Dire_vector(2) < 0 %clockwise from y to x
        Angle = -Angle;
    end
    PtsR = Pts*rotz(Angle*180/pi); 
end
end

