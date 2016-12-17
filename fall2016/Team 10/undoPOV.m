function out = undoPOV(imPOV,tform)

imleft = imPOV(:,1:round(size(imPOV,2)/2));
imright = imPOV(:,round(size(imPOV,2)/2+1):end);

outright = imwarp(imright,tform.invert);
outleft = fliplr(imwarp(fliplr(imleft),tform.invert));

out = [outleft outright];

end