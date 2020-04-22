function [x, P] = kalmanFilter(Y, x_0, P_0, A, Q, H, R)
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
x = zeros(n,N);
P = zeros(n,n,N);

%% Filter Implementation

[x(:,1), P(:,:,1)] = linearPrediction(x_0, P_0, A, Q);
[x(:,1), P(:,:,1)] = linearUpdate(x(:,1), P(:,:,1), Y(:,1), H, R);

for iterator = 2:N
    [x(:,iterator), P(:,:,iterator)] = linearPrediction(x(:,iterator - 1), P(:,:,iterator - 1), A, Q);
    [x(:,iterator), P(:,:,iterator)] = linearUpdate(x(:,iterator), P(:,:,iterator), Y(:,iterator), H, R);
end
end

