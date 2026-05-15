function [x, P] = mu_g(x, P, yacc, Ra, g0)
%MU_G Accelerometer measurement update for quaternion EKF.
%
% Inputs:
%   x    [4 x 1] current quaternion estimate
%   P    [4 x 4] current state covariance
%   yacc [3 x 1] accelerometer measurement [m/s^2]
%   Ra   [3 x 3] accelerometer measurement noise covariance
%   g0   [3 x 1] nominal gravity vector
%
% Outputs:
%   x    [4 x 1] updated quaternion estimate
%   P    [4 x 4] updated state covariance

    x = x(:);
    yacc = yacc(:);
    g0 = g0(:);

    q = x(1:4);

    % Predicted accelerometer measurement
    yhat = Qq(q)' * g0;

    % Measurement Jacobian H = d(Q(q)'*g0)/dq
    [Q0, Q1, Q2, Q3] = dQqdq(q);
    H = [Q0' * g0,  Q1' * g0, Q2' * g0, Q3' * g0];

    % Innovation
    innov = yacc - yhat;

    % Innovation covariance
    S = H * P * H' + Ra;

    % Kalman gain
    K = (P * H') / S;

    % State update
    x = x + K * innov;

    % Joseph covariance update for numerical stability
    P = P - K * S * K';

    % Keep quaternion normalized
    [x, P] = mu_normalizeQ(x, P);
end