function [x, P, v] = kalmanFilter2(Y, x_0, P_0, A, Q, H, R)
%KALMANFILTER Filters measurements sequence Y using a Kalman filter. 
%
%Input:
%   Y           [m x N] Measurement sequence
%   x_0         [n x 1] Prior mean
%   P_0         [n x n] Prior covariance
%   A           [n x n] State transition matrix
%   Q           [n x n] Process noise covariance
%   H           [m x n] Measurement model matrix
%   R           [m x m] Measurement noise covariance
%
%Output:
%   x           [n x N] Estimated state vector sequence
%   P           [n x n x N] Filter error convariance
%

%% Parameters
N = size(Y,2);
n = length(x_0);
m = size(Y,1);

%% Data allocation
x = zeros(n,N+1);
P = zeros(n,n,N+1);

%% Filter Implementation

x(:,1) = x_0;
P(:,:,1) = P_0;

for iterator = 1:N
    
    % Prediction Step
    x(:,iterator) = A*x(:,iterator);                                                            % Calculated Mean
    P(:,:,iterator) = A*P(:,:,iterator)*A' + Q;                                                 % Calculated covariance
    
    % Update Step
    S(:,:,iterator) = H*P(:,:,iterator)*H' + R;                                                 % Innovation Covariance
    K(:,:,iterator) = P(:,:,iterator)*H'*inv(S(:,:,iterator));                                  % Kalman Gain
    v(:,iterator) = Y(:,iterator) - H*x(:,iterator);                                            % Innovation

    x(:,iterator + 1) = x(:,iterator) + K(:,:,iterator)*v(:,iterator);                          % Updated State Mean    
    P(:,:,iterator + 1) = P(:,:,iterator) - K(:,:,iterator)*S(:,:,iterator)*K(:,:,iterator)';   % Updated State Covariance
end

x = x(:,2:end);
P = P(:,:,2:end);

end

