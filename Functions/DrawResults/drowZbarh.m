function Para = drowZbarh(Img,Direction,Type)
%DROWZBARH 
if Direction == -1 %from right to left
    Img = fliplr(Img);
elseif Direction == 2 %from bottom to top
    Img = flipud(Img');
elseif Direction == -2 %From top to bottom
    Img = fliplr(Img');
elseif Direction ~= 1
    error('undefined projection type')
end
% imshow(flipud(Img))
barLen = size(Img,1);
ImgS = sum(Img,2);
ImgW = zeros(barLen,1);
ImgF = zeros(barLen,1);
ImgE = zeros(barLen,1);
ImgEy = zeros(barLen,1);
for i = barLen:-1:1
    isTrue = find(Img(i,:));
    try
        ImgF(i) = isTrue(1);
        ImgE(i) = isTrue(end);
        ImgW(i) = isTrue(end) - isTrue(1) + 1;
    catch %blank line
        continue;
    end
    ImgEy(i) = length(find(Img(i,isTrue(1):isTrue(end)) == 0));
end
if nargin == 3
    if strcmp(Type,'sum')
        Para = ImgS;
    elseif strcmp(Type,'Dsum')
        Para = abs(diff(ImgS));
    elseif strcmp(Type,'fir')
        Para = ImgF;
    elseif strcmp(Type,'Dfir')
        Para = abs(diff(ImgF));
    elseif strcmp(Type,'end')
        Para = ImgE;
    elseif strcmp(Type,'Dend')
        Para = abs(diff(ImgE));
    elseif strcmp(Type,'wid')
        Para = ImgW;
    elseif strcmp(Type,'Dwid')
        Para = abs(diff(ImgW));
    elseif strcmp(Type,'epy')
        Para = ImgEy;
    else
        error('undefined projection type')
    end
else
%     subplot(1,3,1);imshow(flipud(Img));title('Original Image')
%     subplot(1,3,2);barh(ImgF);title('First non-zero pixel')
%     subplot(1,3,3);barh(abs(diff(ImgF)));title('Diff first non-zero pixel')
    subplot(2,4,1);imshow(flipud(Img));title('original image') 
    subplot(2,4,2);barh(ImgW);title('wide');% xlabel('x','FontSize',8);
    subplot(2,4,3);barh(ImgF);title('first')
    subplot(2,4,4);barh(ImgS);title('density')
    subplot(2,4,5);barh(ImgEy);title('non-sum')
    subplot(2,4,6);barh(abs(diff(ImgW)));title('diff-wide')
    subplot(2,4,7);barh(abs(diff(ImgF)));title('diff-first')
    subplot(2,4,8);barh(abs(diff(ImgS)));title('diff-sum')
end

end

