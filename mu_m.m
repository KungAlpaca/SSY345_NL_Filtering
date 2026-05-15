function [x, P] = mu_m(x, P, mag, m0, Rm)
%MU_M Magnetometer measurement update for quaternion EKF.
%
% Inputs:
%   x   [4 x 1] current quaternion estimate
%   P   [4 x 4] current state covariance
%   mag [3 x 1] magnetometer measurement [microT]
%   m0  [3 x 1] nominal magnetic field vector [microT]
%   Rm  [3 x 3] magnetometer measurement noise covariance
%
% Outputs:
%   x   [4 x 1] updated quaternion estimate
%   P   [4 x 4] updated state covariance

    x = x(:);
    mag = mag(:);
    m0 = m0(:);

    q = x(1:4);

    % Predicted magnetometer measurement
    yhat = Qq(q)' * m0;

    % Measurement Jacobian H = d(Q(q)'*m0)/dq
    [Q0, Q1, Q2, Q3] = dQqdq(q);
    H = [Q0' * m0, ...
         Q1' * m0, ...
         Q2' * m0, ...
         Q3' * m0];

    % Innovation
    innov = mag - yhat;

    % Innovation covariance
    S = H * P * H' + Rm;

    % Kalman gain
    K = (P * H') / S;

    % State update
    x = x + K * innov;

    % Covariance update
    P = P - K * S * K';

    % Keep quaternion normalized
    [x, P] = mu_normalizeQ(x, P);
end