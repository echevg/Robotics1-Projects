function block = makeBlock(id,ornt,s,dotted,w)
% ID can be 
%   0 - straight
%   1 - turn (right to up)
% ORNT is an integer for number of 90 degree CCW rotations
% S is the size of the block
% DOTTED is boolean for dotted line (default = False)
if nargin < 5
    w = s/15;
    if nargin < 4
        dotted = false;
    end
end

block = zeros(s);

if id == 0 % Straight block    
    block(:,[1:round(w) round(s-w):s]) = 1;
    block(:,round((s-w)/2):round((s+w)/2)) = .5;
    
else % Turn Block
    %block(:,round(s-w):s) = 1;
    %block(round(s-w):s,:) = 1;
    for i = 1:s
        for j = 1:s
            t = i^2+j^2;
            if (t<=(w)^2) || ((t>=(s-w)^2)&&(t<s^2))
                block(i,j)=1;
            elseif (t>=((s-w)/2)^2 && t<=((s+w)/2)^2)
                block(i,j)=.5;
            end
        end
    end
end

block = imrotate(block,90*ornt);

end