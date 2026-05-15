function [x, P] = tu_qw_no_omega(x, P, T, Rw)
%TU_QW_NO_OMEGA Time update when no gyroscope measurement is available.
%
% In this case, the orientation is assumed constant during the sampling
% interval, but the uncertainty is still increased.

    x = x(:);

    % No angular-rate measurement, so assume zero angular velocity.
    omega = zeros(3,1);

    % State transition matrix
    F = eye(4) + (T/2) * Somega(omega);

    % Since omega = 0, F is simply eye(4), but this keeps the structure clear.
    G = (T/2) * Sq(x);

    % State prediction
    x = F * x;

    % Covariance prediction
    P = F * P * F' + G * Rw * G';

    % Keep quaternion normalized
    [x, P] = mu_normalizeQ(x, P);
end