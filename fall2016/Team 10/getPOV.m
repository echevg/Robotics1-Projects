function im = getPOV(imsmall,cc,tform)
% angle = atand(24/2/228/2);
% d = round(24/2/cosd(angle));

imright = imsmall(:,round(size(imsmall,2)/2)+1:end);
outright = imwarp(imright,tform);
%outright = imrotate(imwarp(imright,tform,'FillValues',0),-angle);

imleft = fliplr(imsmall(:,1:round(size(imsmall,2)/2)));
outleft = fliplr(imwarp(imleft,tform));
%outleft = imrotate(fliplr(imwarp(imleft,tform,'FillValues',0)),angle);

im = [outleft(1:cc(2)*2,end-cc(1):end) outright(1:cc(2)*2,1:cc(1))];
%im = [outleft(1:cc(2)*2,end-cc(1):end-d-4) outright(1:cc(2)*2,d+4:cc(1))];

end

