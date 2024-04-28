function LocP = appendLoc(img,Loc)
%APPENDLOC 
% drowZbarh(img,1);
upimg = img(Loc(2):end,:);% drowZbarh(upimg,1);
[~,LocInd] = maxk(drowZbarh(upimg,1,'Dwid'),2);
LocP = [min(LocInd)+Loc(2)-1,max(LocInd)+Loc(2)-1;Loc];

end

