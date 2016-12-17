clear

baseline = 6;   % Baseline in inches is the distance between the wheels

smallest_interval = 1/8; % Smallest measuring unit is one millimeter
% smallest_interval = 1/16;   % Smallest measuring unit is 1/16 inch

% gain_actual = 1.0042;
% trim_actual = .0133;

for j = 1:10000
gain_actual = .5 + rand();
trim_actual = -.15 + .3*rand();
% gain_actual = .5;
% trim_actual = -.2;

% Always start assuming perfect calibration
gain = 1;
trim = 0;

v_target = 1;
omega_target = 0;

i = 1;
% while abs(trim_actual - trim)>1e-6 && abs(gain_actual - gain)>1e-5
while i<100
% fprintf('---------------------------------\nIteration %.0f\n',i)
% fprintf('Guess Gain: %.4f\nGuess Trim: %.4f\n\n',gain,trim)
    
% Relationship between voltage, linear velocity, and angular velocity
% voltage_right = (gain + trim)*(v + omega*.5*baseline)
% voltage_left = (gain - trim)*(v - omega*.5*baseline)

% Guess the correct voltage
voltage_right = (gain + trim)*(v_target + omega_target*.5*baseline);
voltage_left = (gain - trim)*(v_target - omega_target*.5*baseline);

% Calculate actual result of these voltages
omega_actual = (voltage_left/(gain_actual - trim_actual) - ...
    voltage_right/(gain_actual + trim_actual))/baseline;

v_actual = (voltage_left/(gain_actual - trim_actual) + ...
    voltage_right/(gain_actual + trim_actual))/2;

% Drive for dt seconds
dt = 10;

% Actual results of driving
if omega_actual ~= 0
    endpos = v_actual*[sin(omega_actual*dt); ...
        cos(omega_actual*dt)-1]/omega_actual; 
else
    endpos = v_actual*dt*[1; 0];
end

% Introduce measurement noise then round off
if abs(endpos(2)) > smallest_interval
    endpos_guess = round((endpos - (smallest_interval/2 - ...
        smallest_interval*rand(2,1)))/smallest_interval)*smallest_interval;
else
    endpos_guess = round(endpos/smallest_interval)*smallest_interval;
end
% fprintf('endpos guess: %.4f %.4f\nendpos actual: %.4f %.4f\n',endpos_guess(1),endpos_guess(2),endpos(1),endpos(2))

% Calculate guesses for v_actual and omega_actual
omega_guess = -1/dt*asin(2*endpos_guess(1)*endpos_guess(2)/(endpos_guess(1)^2+endpos_guess(2)^2));

if omega_guess ~= 0
    v_guess = omega_guess*(endpos_guess(1)^2+endpos_guess(2)^2)/(-2*endpos_guess(2));
else
    v_guess = endpos_guess(1)/dt;
end

% fprintf('Velocity Error: %.4f\nOmega Error: %.4f\n',...
%     v_guess-v_actual,omega_guess-omega_actual)

% Use these to guess gain and trim
gain_guess = .5*(voltage_right/(v_guess + .5*omega_guess*baseline) + ...
    voltage_left/(v_guess - .5*omega_guess*baseline));

trim_guess = -.5*(voltage_right/(v_guess + .5*omega_guess*baseline) - ...
    voltage_left/(v_guess - .5*omega_guess*baseline));

if sign(trim) ~= sign(trim_guess) && trim ~= 0
    trim_guess = -trim_guess;
end

gain_error = gain_guess - gain_actual;
trim_error = trim_guess - trim_actual;

% fprintf('Gain Error: %.4f\nTrim Error: %.4f\n',gain_error,trim_error)
% fprintf('\n')

gain = gain_guess;
trim = trim_guess;

if omega_guess == 0
    break
end

i = i + 1;
end

trim_error_log(j) = trim_error;
gain_error_log(j) = gain_error;
iterations_log(j) = i;
actual_trim_log(j) = trim_actual;
actual_gain_log(j) = gain_actual;
end

ind = abs(trim_error_log)<1;
fprintf('\nPercentage of failed trials:      %.1f%%\n',100*sum(1-ind)/numel(ind))

figure
plot(trim_error_log(ind))
xlabel('Trial Number')
ylabel('Trim Error')
figure
xlabel('Trial Number')
ylabel('Trim Error')
plot(gain_error_log(ind))

fprintf('Mean and Stdev of Trim Error:     %.7f    %.7f\n',mean(trim_error_log(ind)),std(trim_error_log(ind)))
fprintf('log10(Min) and Max of Trim Error: %.7f    %.7f\n',log10(min(abs(trim_error_log(ind)))),max(abs(trim_error_log(ind))))
fprintf('Mean and Stdev of Gain Error:     %.7f    %.7f\n',mean(gain_error_log(ind)),std(gain_error_log(ind)))
fprintf('log10(Min) and Max of Gain Error: %.7f    %.7f\n',log10(min(abs(gain_error_log(ind)))),max(abs(gain_error_log(ind))))

