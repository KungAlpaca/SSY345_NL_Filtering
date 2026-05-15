function [x, P] = tu_qw(x, P, omega, T, Rw)
%TU_QW Time update for quaternion EKF using gyroscope input.
%
% Inputs:
%   x     [4 x 1] current quaternion estimate
%   P     [4 x 4] current state covariance
%   omega [3 x 1] angular velocity measurement - constant_bias  [rad/s]
%   T     [1 x 1] sampling time [s]
%   Rw    [3 x 3] gyroscope/process noise covariance
%
% Outputs:
%   x     [4 x 1] predicted quaternion estimate
%   P     [4 x 4] predicted covariance

    % Make sure vectors have the correct shape
    x = x(:);
    omega = omega(:);

    % State transition matrix:
    % F = I + T/2 * S(omega)
    F = eye(4) + (T/2) * Somega(omega);

    % Process-noise mapping:
    % G = T/2 * Sbar(q_hat)
    % Use the current estimate before prediction.
    G = (T/2) * Sq(x);

    % State prediction
    x = F * x;

    % Covariance prediction
    P = F * P * F' + G * Rw * G';

    % Keep quaternion normalized
    [x, P] = mu_normalizeQ(x, P);
end