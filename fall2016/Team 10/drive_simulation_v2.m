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
campos = s*[1.05; 1.75];    % Initial position (y,x) in pixels - The reason 
                        % for this is that in images, y is the first index
                        
v0 = 1/ipp;             % constant tangential velocity in pixels/s

theta = 0;              % Angle in degrees CCW from the vertical

omega = 0;              % Angular velocity in degrees/s
omega_max = (2*(v_max - v0)/car_width) * 180/pi();  % Max omega based on 
                                                    % constant velocity

% Desired position, angle
d_goal = s*.25;
th_goal = 0;

% Proportional and Integral Gains
Kp = [0.08*ipp 0.2]*v0*ipp*1.8;
Ki = Kp/9;

% During the driving simulation, keep logs of everything useful
clear timelog
clear poslog
clear thlog
clear omegalog
clear imPOVlog
clear error_d_log
clear error_th_log
clear error_d_actual_log
clear error_th_actual_log


%% Simulation
timestep = 1;
time_total = 0;
while time_total < 500
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
    
    % Now, read image and attempt to learn the car's position and angle by
    % running the POV image through the inverse transform
    [d_guess_gray, th_guess_gray] = parsePOV(imPOV,tform,region,.5);
    [d_guess_white, th_guess_white] = parsePOV(imPOV,tform,region,1);
    
    if isnan(d_guess_gray)
        if isnan(d_guess_white)
            case_log(timestep) = 1;
            d_guess = 0;
            th_guess = 0;
        else
            case_log(timestep) = 2;
            d_guess = d_guess_white + d_goal;
            th_guess = th_guess_white;
        end
    else
        if isnan(d_guess_white)
            case_log(timestep) = 3;
            d_guess = d_guess_gray - d_goal;
            th_guess = th_guess_gray;
        else
            case_log(timestep) = 4;
            d_guess = .5*(d_guess_gray + d_guess_white);
            th_guess = .5*(th_guess_gray + th_guess_white);
        end
    end
    
    % d_guess is the estimated perpendicular distance to the left of the
    % center of the lane, in pixels
    
    % th_guess is the estimated angle with respect to the lane, in degrees
    
    error_d = d_guess;
    error_th = th_guess;
    
    %error_d = d_guess - d_goal; % Guessed error from desired lane position
    %error_th = th_guess - th_goal; % Guess error from desired angle
    
    % Update error logs
    error_d_log(timestep) = error_d;
    error_th_log(timestep) = error_th;
    
    % Obtain actual distance and angle error
    [error_d_actual, error_th_actual]...
        = getDistanceAndAngle(campos,theta,s,occfn);
    
    % error_d_actual is in pixels
    % error_th_actual is in degrees
    
    % Update error logs
    error_d_actual_log(timestep) = error_d_actual;
    error_th_actual_log(timestep) = error_th_actual;
    
    % Use error to drive angular velocity
    % Using guessed error values:
    omega = max(min(omega_max, ...
        Kp*[error_d; error_th*pi()/180] + Ki*[sum(error_d_log); ...
        sum(error_th_log)*pi()/180]/timestep), -omega_max);
%     
%     % Using correct error values:
%     omega = max(min(omega_max, ...
%         Kp*[error_d_actual; error_th_actual*pi()/180] ...
%         + Ki*[sum(error_d_actual_log);sum(thlog)*pi()/180]/timestep), ...
%         -omega_max);
    
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
    
    % Text Outputs
    fprintf('Vertical Position:   %.2f\n',campos(1))
    fprintf('Horizontal Position: %.2f\n',campos(2))
    fprintf('Distance Error: %.2f\n',error_d_actual)
    fprintf('Angle Error:    %.2f\n',error_th_actual)
    fprintf('Distance Guess: %.2f\n',error_d)
    fprintf('Angle Guess:    %.2f\n',error_th)
    fprintf('Omega: %.2f\n',omega)
    fprintf('\n')
    
end
    







