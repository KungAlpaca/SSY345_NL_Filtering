function [xhat, meas] = filter_complete(calAcc, calGyr, calMag)
% FILTERTEMPLATE  Filter template
%
% This is a template function for how to collect and filter data
% sent from a smartphone live.  Calibration data for the
% accelerometer, gyroscope and magnetometer assumed available as
% structs with fields m (mean) and R (variance).
%
% The function returns xhat as an array of structs comprising t
% (timestamp), x (state), and P (state covariance) for each
% timestamp, and meas an array of structs comprising t (timestamp),
% acc (accelerometer measurements), gyr (gyroscope measurements),
% mag (magnetometer measurements), and orint (orientation quaternions
% from the phone).  Measurements not availabe are marked with NaNs.
%
% As you implement your own orientation estimate, it will be
% visualized in a simple illustration.  If the orientation estimate
% is checked in the Sensor Fusion app, it will be displayed in a
% separate view.
%
% Note that it is not necessary to provide inputs (calAcc, calGyr, calMag).

  %% Setup necessary infrastructure
  import('com.liu.sensordata.*');  % Used to receive data.


  %% Load calibration data:
    if exist('calibration_static.mat', 'file')
        tmp = load('calibration_static.mat', 'calAcc', 'calGyr', 'calMag', 'g0', 'm0');

    if nargin < 1 || isempty(calAcc)
        calAcc = tmp.calAcc;
    end

    if nargin < 2 || isempty(calGyr)
        calGyr = tmp.calGyr;
    end

    if nargin < 3 || isempty(calMag)
        calMag = tmp.calMag;
    end

    g0 = tmp.g0;
    m0 = tmp.m0;

else
    warning('No calibration file found. Using default values.');

    if nargin < 1 || isempty(calAcc)
        calAcc.m = [0; 0; 9.81];
        calAcc.R = 1e-2 * eye(3);
    end

    if nargin < 2 || isempty(calGyr)
        calGyr.m = zeros(3,1);
        calGyr.R = 1e-6 * eye(3);
    end

    if nargin < 3 || isempty(calMag)
        calMag.m = [0; 20; -40];
        calMag.R = eye(3);
    end

    g0 = [0; 0; 9.81];
    m0 = [0; 20; -40];
end
  %% Filter settings
  t0 = [];  % Initial time (initialize on first data received)
  t_prev= [];
  nx = 4;   % Assuming that you use q as state variable.
  % Add your filter settings here.

  % Current filter state.
  x = [1; 0; 0 ;0];
  P = eye(nx, nx);

  accThreshold = 0.5; % m/s^2
  gRef = norm(g0); % calibrated gravity magnitude


magThreshold = 4.0; % [microT]
alphaMag = 0.01; % AR(1) adaptation factor
magRef = norm(m0); % initial expected magnetic field magnitude

  % Saved filter states.
  xhat = struct('t', zeros(1, 0),...
                'x', zeros(nx, 0),...
                'P', zeros(nx, nx, 0));

  meas = struct('t', zeros(1, 0),...
                'acc', zeros(3, 0),...
                'gyr', zeros(3, 0),...
                'mag', zeros(3, 0),...
                'orient', zeros(4, 0));
  try
    %% Create data link
    server = StreamSensorDataReader(3400);
    % Makes sure to resources are returned.
    sentinel = onCleanup(@() server.stop());

    server.start();  % Start data reception.

    % Used for visualization.
    figure(1);
    subplot(1, 2, 1);
    ownView = OrientationView('Own filter', gca);  % Used for visualization.
    googleView = [];
    counter = 0;  % Used to throttle the displayed frame rate.

    %% Filter loop
    while server.status()  % Repeat while data is available
      % Get the next measurement set, assume all measurements
      % within the next 5 ms are concurrent (suitable for sampling
      % in 100Hz).
      data = server.getNext(5);

      if isnan(data(1))  % No new data received
        continue;        % Skips the rest of the look
      end
      t = data(1)/1000;  % Extract current time

      if isempty(t0)  % Initialize t0
        t0 = t;
        t_prev = t;
      end

      T = t- t_prev;

      acc = data(1, 2:4)';
      gyr = data(1, 5:7)';
      mag = data(1, 8:10)';

      % Default disturbance flags
      accOut = false;
      magOut = false;

      % Time update using gyroscope
    if T > 0
        if ~any(isnan(gyr))
        
        % Bias-corrected gyroscope input
        omega = gyr - calGyr.m;
        
        % Gyroscope time update
        [x, P] = tu_qw(x, P, omega, T, calGyr.R);
        
        else
        
        % No gyroscope measurement available
        [x, P] = tu_qw_no_omega(x, P, T, calGyr.R);
        
        end
    end
    
    % ------------------------------------------------------------
    % 2. Accelerometer update with outlier rejection
    % ------------------------------------------------------------
    if ~any(isnan(acc))
    
        accNorm = norm(acc);
        accOut = abs(accNorm - gRef) > accThreshold;
    
        if ~accOut
        [x, P] = mu_g(x, P, acc, calAcc.R, g0);
        end
    end
    
    setAccDist(ownView, accOut);
    
    % ------------------------------------------------------------
    % 3. Magnetometer update with outlier rejection
    % ------------------------------------------------------------
    if ~any(isnan(mag))
    
        magNorm = norm(mag);
        magOut = abs(magNorm - magRef) > magThreshold;
    
        if ~magOut
        [x, P] = mu_m(x, P, mag, m0, calMag.R);
        
        % Slowly adapt expected magnetic field magnitude
        magRef = (1 - alphaMag) * magRef + alphaMag * magNorm;
        end
    end
    
    setMagDist(ownView, magOut);

      orientation = data(1, 18:21)';  % Google's orientation estimate.

      % Visualize result
      if rem(counter, 10) == 0
        setOrientation(ownView, x(1:4));
        title(ownView, 'OWN', 'FontSize', 16);
        if ~any(isnan(orientation))
          if isempty(googleView)
            subplot(1, 2, 2);
            % Used for visualization.
            googleView = OrientationView('Google filter', gca);
          end
          setOrientation(googleView, orientation);
          title(googleView, 'GOOGLE', 'FontSize', 16);
        end
      end
      counter = counter + 1;

      % update previous time step
        t_prev = t;

      % Save estimates
      xhat.x(:, end+1) = x;
      xhat.P(:, :, end+1) = P;
      xhat.t(end+1) = t - t0;

      meas.t(end+1) = t - t0;
      meas.acc(:, end+1) = acc;
      meas.gyr(:, end+1) = gyr;
      meas.mag(:, end+1) = mag;
      meas.orient(:, end+1) = orientation;
    end
  catch e
    fprintf(['Unsuccessful connecting to client!\n' ...
      'Make sure to start streaming from the phone *after*'...
             'running this function!']);
  end
end
