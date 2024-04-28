function TowerIDs = getTowerID(filepath)
%GETTOWERID Get the number of each tower of the transmission line

ReadDir = dir(filepath);
ReadDir = ReadDir(3:end);
FileNum = length(ReadDir);
TowerIDs = [];
for j = 1:FileNum
    filename = ReadDir(j).name;
    if ~contains(filename,'Tower.txt')
        continue % Skip non-pole tower files
    end
    TowerIDs = [TowerIDs;filename(1:strfind(filename,'Tower')-1)];% Tower number
end
end

