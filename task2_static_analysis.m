%% Task 2 - Static sensor analysis
%first, to load data in command window:
%[xhatStatic, measStatic] = filterTemplate(); 
% then save with:
%save('static_calibration.mat', 'xhatStatic', 'measStatic');
clear; clc; close all;

load('static_calibration.mat');   % Loads xhatStatic and measStatic


%% Trim beginning and end of the recording
trimTime = 1.0;   % seconds to remove from start and end

t = measStatic.t;

keepIdx = (t >= t(1) + trimTime) & (t <= t(end) - trimTime);

% Trim measurement data
measStatic.t      = measStatic.t(keepIdx);
measStatic.acc    = measStatic.acc(:, keepIdx);
measStatic.gyr    = measStatic.gyr(:, keepIdx);
measStatic.mag    = measStatic.mag(:, keepIdx);
measStatic.orient = measStatic.orient(:, keepIdx);

% Reset time so the trimmed data starts at zero
measStatic.t = measStatic.t - measStatic.t(1);


meas = measStatic;

%trim xhatStatic too, only if it has the same number of samples
if exist('xhatStatic', 'var') && length(xhatStatic.t) == length(t)
    xhatStatic.t = xhatStatic.t(keepIdx);
    xhatStatic.x = xhatStatic.x(:, keepIdx);
    xhatStatic.P = xhatStatic.P(:, :, keepIdx);

    xhatStatic.t = xhatStatic.t - xhatStatic.t(1);
end

%% Clean data: remove columns with NaNs
acc = cleanSensor(meas.acc);
gyr = cleanSensor(meas.gyr);
mag = cleanSensor(meas.mag);

%% Time vector
t = meas.t;

%% Compute means and covariances
calAcc.m = mean(acc, 2);
calAcc.R = cov(acc.');

calGyr.m = mean(gyr, 2);
calGyr.R = cov(gyr.');

calMag.m = mean(mag, 2);
calMag.R = cov(mag.');

%% Also compute variances per axis
varAcc = var(acc, 0, 2);
varGyr = var(gyr, 0, 2);
varMag = var(mag, 0, 2);

%% Norms
accNorm = vecnorm(acc);
gyrNorm = vecnorm(gyr);
magNorm = vecnorm(mag);

%% Gravity and magnetic field estimates
g_est = norm(calAcc.m);

% Nominal gravity vector, assuming the initial flat position corresponds
% approximately to the initial quaternion orientation.
g0 = calAcc.m;

% Nominal magnetic field according to the project model.
% This removes the unknown yaw direction when the phone lies flat.
m = calMag.m;
m0 = [0;
      sqrt(m(1)^2 + m(2)^2);
      m(3)];

%% Print results
disp('Accelerometer mean [m/s^2]:');
disp(calAcc.m);

disp('Accelerometer covariance:');
disp(calAcc.R);

disp('Gyroscope mean [rad/s]:');
disp(calGyr.m);

disp('Gyroscope covariance:');
disp(calGyr.R);

disp('Magnetometer mean [microT]:');
disp(calMag.m);

disp('Magnetometer covariance:');
disp(calMag.R);

disp('Estimated gravity magnitude:');
disp(g_est);

disp('Normal gravity vector:');
disp(g0);

disp('Estimated nominal magnetic field m0:');
disp(m0);

disp('Nominal magnetic field magnitude:');
disp(norm(m0));

%% Save calibration structs
save('calibration_static.mat',  'calAcc', 'calGyr', 'calMag', 'g0', ...
    'm0','varAcc', 'varGyr', 'varMag', ...
     'accNorm', 'gyrNorm', 'magNorm');

%% Plot sensor signals over time
figure;
tiledlayout(3,1);

nexttile;
plot(acc.');
grid on;
title('Accelerometer measurements');
xlabel('Sample');
ylabel('Acceleration [m/s^2]');
legend('x','y','z');

nexttile;
plot(gyr.');
grid on;
title('Gyroscope measurements');
xlabel('Sample');
ylabel('Angular velocity [rad/s]');
legend('x','y','z');

nexttile;
plot(mag.');
grid on;
title('Magnetometer measurements');
xlabel('Sample');
ylabel('Magnetic field [\muT]');
legend('x','y','z');

%% Plot measurement norms
figure;
tiledlayout(3,1);

nexttile;
plot(accNorm);
grid on;
title('Accelerometer norm');
xlabel('Sample');
ylabel('||a|| [m/s^2]');

nexttile;
plot(gyrNorm);
grid on;
title('Gyroscope norm');
xlabel('Sample');
ylabel('||\omega|| [rad/s]');

nexttile;
plot(magNorm);
grid on;
title('Magnetometer norm');
xlabel('Sample');
ylabel('||m|| [\muT]');

%% Histograms
figure;
tiledlayout(3,3);

sensorNames = {'Accelerometer', 'Gyroscope', 'Magnetometer'};
axisNames = {'x', 'y', 'z'};
sensorData = {acc, gyr, mag};

for s = 1:3
    for ax = 1:3
        nexttile;
        histogram(sensorData{s}(ax,:), 40);
        grid on;
        title([sensorNames{s}, ' ', axisNames{ax}, '-axis']);
    end
end

%% Local helper function
function y = cleanSensor(x)
    y = x(:, ~any(isnan(x), 1));
end