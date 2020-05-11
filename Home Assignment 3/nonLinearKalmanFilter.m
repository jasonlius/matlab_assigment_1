function [xf, Pf, xp, Pp] = nonLinearKalmanFilter(Y, x_0, P_0, f, Q, h, R, type)
    % NONLINEARKALMANFILTER Filters measurement sequence Y using a 
    % non-linear Kalman filter. 
    %
    % Input:
    %   Y           [m x N] Measurement sequence for times 1,...,N
    %   x_0         [n x 1] Prior mean for time 0
    %   P_0         [n x n] Prior covariance
    %   f                   Motion model function handle
    %                       [fx,Fx]=f(x) 
    %                       Takes as input x (state) 
    %                       Returns fx and Fx, motion model and Jacobian evaluated at x
    %   Q           [n x n] Process noise covariance
    %   h                   Measurement model function handle
    %                       [hx,Hx]=h(x,T) 
    %                       Takes as input x (state), 
    %                       Returns hx and Hx, measurement model and Jacobian evaluated at x
    %   R           [m x m] Measurement noise covariance
    %
    % Output:
    %   xf          [n x N]     Filtered estimates for times 1,...,N
    %   Pf          [n x n x N] Filter error convariance
    %   xp          [n x N]     Predicted estimates for times 1,...,N
    %   Pp          [n x n x N] Predicted error convariance

%% Parameters
N = size(Y,2);
n = length(x_0);
m = size(Y,1);

%% Data allocation
xp = zeros(n,N);
Pp = zeros(n,n,N);

xf = zeros(n,N+1);
Pf = zeros(n,n,N+1);

%% Filter Implementation

xf(:,1)   = x_0;
Pf(:,:,1) = P_0;

for iterator = 1:N
    [xp(:,iterator), Pp(:,:,iterator)] = nonLinKFprediction(xf(:,iterator), Pf(:,:,iterator), f, Q, type);
    [xf(:,iterator+1), Pf(:,:,iterator+1)] = nonLinKFupdate(xp(:,iterator), Pp(:,:,iterator), Y(:,iterator), h, R, type);
end
    
xf = xf(:,2:end);
Pf = Pf(:,:,2:end);
    
end