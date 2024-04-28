function bigPts = removeDuplicatePoints(bigPts, smallPts,Thre)
% point_set2 is a subset of point_set1, removing point_set2 in point_set1
[Ind,dist] = knnsearch(bigPts,smallPts,"K",8);
% drowPts(point_set1,'.g',point_set1(ind,:),'.r')
bigPts(Ind(dist<Thre),:) = [];% drowPts(point_set1,'.r')
% writematrix(point_set1,"Ins1.txt")
% writematrix(point_set2,"Ins.txt")
% keyboard
end


