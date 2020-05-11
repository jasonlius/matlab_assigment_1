function Y = genNonLinearMeasurementSequence(X, h, R)
%GENNONLINEARMEASUREMENTSEQUENCE generates ovservations of the states 
% sequence X using a non-linear measurement model.
%
%Input:
%   X           [n x N+1] State vector sequence
%   h           Measurement model function handle
%   h           Measurement model function handle
%               [hx,Hx]=h(x) 
%               Takes as input x (state) 
%               Returns hx and Hx, measurement model and Jacobian evaluated at x
%   R           [m x m] Measurement noise covariance
%
%Output:
%   Y           [m x N] Measurement sequence
%

% Your code here
Y = zeros(size(R,1),size(X,2)-1);                       % Initialize observation sequence
r = (mvnrnd(zeros(size(R,1),1),R,size(X,2)-1))';        % Draw measurement noise from multidimensional Gaussian Distribution
for iterator = 1:(size(X,2)-1)
    [hx,~] = h(X(:,iterator+1));                        % Generate observations 
    Y(:,iterator) = hx+r(:,iterator);                   % Generate observation sequence
end
end