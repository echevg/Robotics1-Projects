%% Initialize
clear

% Simulation size parameters
s = 100;        % Block size in pixels
ipp = 12/s;     % Inches per pixel (Assuming block are 1x1 foot)

% Build Track
board = [];
occfn = -1;

[board,occfn] = addToBoard(s,'straight',board,occfn);
[board,occfn] = addToBoard(s,'straight',board,occfn);
[board,occfn] = addToBoard(s,'straight',board,occfn);
[board,occfn] = addToBoard(s,'left',board,occfn);
[board,occfn] = addToBoard(s,'left',board,occfn);
[board,occfn] = addToBoard(s,'straight',board,occfn);
[board,occfn] = addToBoard(s,'straight',board,occfn);
[board,occfn] = addToBoard(s,'straight',board,occfn);
[board,occfn] = addToBoard(s,'left',board,occfn);
[board,occfn] = addToBoard(s,'left',board,occfn);

% Camera parameters
cam_height = 3/ipp;
phi = -30*pi()/180;     % Camera Angle (upward) from horizontal [should be negative]

f = 666;        % Camera focus
cc = [291,228]; % Camera principal point

% Use camera parameters to get region in which camera can see
[region,tform] = getCameraVision(f,cc,phi,cam_height);

% Car parameters
car_width = 6/ipp;  % Car width, in pixels
car_length = 8/ipp; % Car length, in pixels
v_max = 4/ipp;      % Max linear velocity of wheels

gain = 1; % Wheel calibration results
trim = 0;

%% Initialize Driving Parameters

% Position, angle, and their deriviatives
campos = s*[3; 1.75];    % Initial position (y,x) in pixels - The reason 
                        % for this is that in images, y is the first index
                        
v0 = 4/ipp;             % constant tangential velocity in pixels/s

theta = 0;              % Angle in degrees CCW from the vertical

omega = 0;              % Angular velocity in degrees/s
omega_max = (2*(v_max - v0)/car_width) * 180/pi();  % Max omega based on 
                                                    % constant velocity


% During the driving simulation, keep logs of everything useful
clear timelog
clear poslog
clear thlog
clear omegalog
clear imPOVlog


%% Simulation
timestep = 1;
time_total = 0;
while campos(2) > s
    tic     % Keep time from beginning of each loop
    
    % Update logs 
    poslog(:, timestep) = campos;
    thlog(timestep) = theta;
    omegalog(timestep) = omega;
        
    % To simulate camera view, first need to rotate the board around the
    % camera such that it points upward.  In order to do this, the camera
    % position must be centered on the board.
    im = transformBoard(board,round(campos),theta);
    
    % Isolate the trapezoidal window seen by the camera
    bbox = round(region + ones(4,1)*size(im)/2);
    
    % if the window exceeds the dimensions of the board, add more blank
    % space to allow for complete viewing window
    if bbox(1,1)<0
        im = [zeros(-bbox(1,1)+1,size(im,2));...
            im;...
            zeros(-bbox(1,1)+1,size(im,2))];
    end
    if min(bbox(1:2,2))<0
        im = [zeros(size(im,1),-min(bbox(1:2,2))+1)...
            im...
            zeros(size(im,1),-min(bbox(1:2,2))+1)];
    end
    bbox = round(region + ones(4,1)*size(im)/2);
    
    % Crop to only the region enclosing the trapezoid
    imsmall = im(bbox(1,1):bbox(3,1),bbox(1,2):bbox(2,2));
    
    % Generate what the camera sees
    imPOV = getPOV(imsmall,cc,tform);
    
    % Update POV images log
    imPOVlog(:,:,timestep) = imPOV;
    
    % Open loop instructions
    if campos(1) > s
        omega = 0;
    else
        omega = v0/(s*.75);
    end    
    
    % The wheel voltages can be obtained from (omega,v0)
    voltage_right = (gain + trim)*(v0 + .5*omega*car_width);
    voltage_left = (gain - trim)*(v0 - .5*omega*car_width);
    
    % Convert omega to degrees/s
    omega = omega * 180/pi();
    
    % Update position
    if omega == 0
        dpos = v0*toc*[cosd(theta); sind(theta)];
    else
        dpos = v0*180/pi()/omega*[sind(omega*toc + theta) - sind(theta); ...
            cosd(theta) - cosd(omega*toc + theta)];
    end
    campos = campos - dpos;
    
    % Update angle
    theta = mod(theta + omega*toc + 180,360)-180;
    
    % Update timelog
    time_total = time_total + toc;
    timelog(timestep) = time_total;  
    
    % Increment timestep
    timestep = timestep + 1;    
    
    % Display images
    figure(1)
    imshow(imPOV)
        
end
    







