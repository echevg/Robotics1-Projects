function [d,th] = parsePOV(imPOV,tform,region,color)

% Process image
im_transform = imclose(undoPOV(imPOV,tform)==color,strel('diamond',5));

% Restrict field of view at first
im = im_transform.*[zeros(ceil(size(im_transform,1)*3/4),size(im_transform,2)); 
    ones(floor(size(im_transform,1)/4),size(im_transform,2))];
% Camera position
iperp = size(im,1)-region(3,1);
jperp = size(im,2)/2;

% Calculate line
[jj,ii] = meshgrid(1:size(im,2),1:size(im,1));
if sum(im==1) == 0
    d = nan;
    th = nan;
else
y = jj(im==1);
A = [ones(size(y,1),1) ii(im==1)];
soln = A\y;
if soln(1) == 0 && soln(2) == 0
    d = nan;
    th = nan;
else
% 
% figure
% figure(2)
% hold on
% imshow(im)
% line(soln(1)+soln(2)*[0;size(im,1)],[0;size(im,1)])
% hold off

th = atand(soln(2));
i_guess = (iperp+soln(2)*(jperp-soln(1)))/(1+soln(2));
p = [i_guess-iperp soln(2)*i_guess+soln(1)-jperp];
d = sqrt(p(1)^2 + p(2)^2)*sign(-p(2));
end
end
end