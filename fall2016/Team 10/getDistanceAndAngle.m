function [d,th] = getDistanceAndAngle(pos,angle,s,occfn)
% pos is vertical,horizontal in board image

row = ceil(pos(1)/s);
col = ceil(pos(2)/s);

block_pos = pos - [(row-1)*s; (col-1)*s];

occ = occfn(row,col);

direction_val = mod(occ,4)+1;
piece_val = floor(occ/4);
if piece_val == 2
    direction_val = mod(direction_val,4) + 1;
end

theta = -pi()/2*(direction_val - 1);

R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
block_pos_rot = s*.5*[1;1] + R*(block_pos - s*.5*[1;1]);
angle_rot = mod(angle + theta*180/pi() + 180,360)-180;

if piece_val == 0
    th = mod(90*(direction_val - 1) - angle + 180,360) - 180;
    d = block_pos_rot(2) - s*.75;
else
    x = s - block_pos_rot(1);
    y = block_pos_rot(2);
    th = atand(x/y)-angle_rot;
    d = sqrt(x^2 + y^2) - s*.75;
end    

end