function [region,tform] = getCameraVision(f,cc,cam_angle,cam_height)
% f = camera focus
% cc = camera principal point
% cam_angle = angle at which camera is pointing
% cam_height = height of camera on car

phi = atan(cc(2)/f);    % camera vertical range angle
theta = atan(cc(1)/f);  % camera horizontal range angle

phi_up = cam_angle + phi;   % camera angle absolute max
phi_down = cam_angle - phi; % camera angle absolute min

% Define range of camera's vision
d_up = cam_height/tan(-phi_up); % farthest distance camera can see in road plane
h_up = cam_height/sin(-phi_up); % hypotenuse for ^this distance

d_down = cam_height/tan(-phi_down);
h_down = cam_height/sin(-phi_down);

d_up_side = tan(theta)*h_up;    % farthest horizontal span for d_up
d_down_side = tan(theta)*h_down;

region = -[d_up d_up_side;
    d_up -d_up_side;
    d_down d_down_side;
    d_down -d_down_side];

p = [0 0 1; d_up_side 0 1; d_down_side d_up-d_down 1; 0 d_up-d_down 1];
q = [0 0 1; cc(1) 0 1; cc(1) 2*cc(2) 1; 0 2*cc(2) 1];

A = [];
y = [];
for i = 1:4
    A = [A; [p(i,:) zeros(1,3) -q(i,1)*p(i,1:2)]];
    A = [A; [zeros(1,3) p(i,:) -q(i,2)*p(i,1:2)]];
    y = [y;q(i,1);q(i,2)];
end

soln = A\y;
T = [soln(1) soln(4) soln(7);
    soln(2) soln(5) soln(8);
    soln(3) soln(6) 1];

tform = projective2d(T);
end